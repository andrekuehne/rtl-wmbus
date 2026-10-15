/* Keep private decoder/DSP helpers testable without changing the production API. */
#define main rtl_wmbus_program_main
#include "../rtl_wmbus.c"
#undef main
#include "../include/mode_s_util.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return EXIT_FAILURE; \
    } \
} while (0)

static int test_crc(void)
{
    /* CRC-16/EN-13757 check value from the CRC RevEng catalogue. */
    const uint8_t check[] = "123456789";
    CHECK(calc_crc_wmbus(check, 9) == 0xc2b7);
    CHECK(calc_crc_wmbus(check, 0) == 0xffff);

    /* Ten-byte headers with CRCs independently calculated using polynomial 0x3d65. */
    uint8_t frame_a[] = {9, 0x44, 0x65, 0x32, 0x41, 0x85, 0x99, 4, 1, 8, 0x33, 0x58};
    uint8_t frame_b[] = {11, 0x44, 0x65, 0x32, 0x41, 0x85, 0x99, 4, 1, 8, 0x44, 0x7e};
    CHECK(check_calc_crc_wmbus(frame_a, sizeof(frame_a)));
    CHECK(check_calc_crc_wmbus_b_frame_type(frame_b, sizeof(frame_b)));
    for (size_t length = 0; length < 12; ++length)
        CHECK(!check_calc_crc_wmbus(frame_a, length));
    frame_a[2] ^= 1;
    CHECK(!check_calc_crc_wmbus(frame_a, sizeof(frame_a)));
    frame_a[2] ^= 1;
    frame_a[11] ^= 1;
    CHECK(!check_calc_crc_wmbus(frame_a, sizeof(frame_a)));
    frame_a[11] ^= 1;
    CHECK(cook_pkt(frame_a, sizeof(frame_a)) == 10);
    CHECK(cook_pkt_b_frame_type(frame_b, sizeof(frame_b)) == 10);
    CHECK(memcmp(frame_a, frame_b, 10) == 0);
    return EXIT_SUCCESS;
}

static int test_manchester(void)
{
    const uint8_t payload[] = {0x00, 0xff, 0xa5};
    const uint8_t expected[] = {0xaa, 0xaa, 0x55, 0x55, 0x66, 0x99};
    uint8_t encoded[sizeof(expected) + 1] = {0};
    CHECK(encode_channel_inv_manchester(encoded, payload, 0) == 0);
    CHECK(encoded[0] == 0);
    CHECK(encode_channel_inv_manchester(encoded, payload, sizeof(payload)) == sizeof(expected));
    CHECK(memcmp(encoded, expected, sizeof(expected)) == 0);
    CHECK(encoded[sizeof(expected)] == 0);
    return EXIT_SUCCESS;
}

static int test_filters(void)
{
    int average_history[3] = {0};
    MAVGI_FILTER average = {.hist = average_history, .length = 3};
    const int samples[] = {3, 6, 9, 12, -3, -6, 0};
    const float expected[] = {1, 3, 6, 9, 6, 1, -3};
    for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i)
        CHECK(mavgi(samples[i], &average) == expected[i]);

    float coefficients[] = {0.25f, 0.5f, 0.25f};
    float fir_history[3] = {0};
    FIRF_FILTER filter = {.length = 3, .b = coefficients, .hist = fir_history};
    const float response[] = {0.25f, 0.5f, 0.25f, 0, 0, 0};
    for (size_t i = 0; i < sizeof(response) / sizeof(response[0]); ++i)
        CHECK(firf(i == 0 ? 1.0f : 0.0f, &filter) == response[i]);
    return EXIT_SUCCESS;
}

/* Preserve the original evaluator as the state and bit-exact output reference. */
static float reference_mavgi(int sample, MAVGI_FILTER *filter)
{
    filter->sum = filter->sum - filter->hist[filter->i] + sample;
    filter->hist[filter->i++] = sample;
    if (filter->i >= filter->length) filter->i = 0;
    return (float)filter->sum / filter->length;
}

static int test_fixed_moving_average(size_t length,
                                    float (*fixed_filter)(float, size_t))
{
    int reference_history[2][16] = {{0}};
    int candidate_history[2][16] = {{0}};
    MAVGI_FILTER reference[2] = {
        {.length = length, .hist = reference_history[0]},
        {.length = length, .hist = reference_history[1]}
    };
    MAVGI_FIXED_FILTER candidate[2] = {{0}};
    uint32_t random = 1;
    for (size_t i = 0; i < 32768; ++i)
    {
        random = random * 1664525u + 1013904223u;
        const size_t channel = random >> 31;
        float input;
        switch ((i / 128) % 7)
        {
            case 0: input = i == 0 ? 127.5f : 0.f; break;
            case 1: input = 127.5f; break;
            case 2: input = -127.5f; break;
            case 3: input = channel ? -180.99f : 180.99f; break;
            case 4: input = ((int)((random >> 16) % 11585) - 5792) / 32.f; break;
            case 5: input = channel ? -0.9999f : 0.9999f; break;
            default: input = channel ? -1048576.75f : 1048576.75f; break;
        }
        const float expected = reference_mavgi((int)input, &reference[channel]);
        const float actual = length == 8
            ? mavgi_power_of_two((int)input, candidate_history[channel], &candidate[channel], 8)
            : mavgi_power_of_two((int)input, candidate_history[channel], &candidate[channel], 16);
        const float fixed = fixed_filter(input, channel);
        CHECK(memcmp(&actual, &expected, sizeof(float)) == 0);
        CHECK(memcmp(&fixed, &expected, sizeof(float)) == 0);
        for (size_t q = 0; q < 2; ++q)
        {
            CHECK(candidate[q].sum == reference[q].sum);
            CHECK(candidate[q].i == reference[q].i);
            CHECK(memcmp(candidate_history[q], reference_history[q],
                         length * sizeof(int)) == 0);
        }
    }
    return EXIT_SUCCESS;
}

/* Original full tap sets also check that the fixed wrappers keep their taps. */
static int test_symmetric_filter(size_t length, float *coefficients,
                                 float (*fixed_filter)(float))
{
    float reference_history[46] = {0};
    float folded_history[46] = {0};
    float mirrored_storage[2 * length + 2];
    float folded_coefficients[(length + 1) / 2];
    FIRF_FILTER reference = {.length = length, .b = coefficients,
                            .hist = reference_history};
    FIRF_FILTER folded = {.length = length, .b = folded_coefficients,
                         .hist = folded_history};
    FIRF_FILTER mirrored = {.length = length, .b = folded_coefficients,
                           .hist = mirrored_storage + 1};
    uint32_t random = 1;
    double max_error_ratio = 0;
    double max_sign_magnitude = 0;
    size_t sign_differences = 0;

    memcpy(folded_coefficients, coefficients, sizeof(folded_coefficients));
    memset(mirrored_storage, 0, sizeof(mirrored_storage));
    mirrored_storage[0] = 12345.25f;
    mirrored_storage[2 * length + 1] = -9876.5f;
    for (size_t pattern = 0; pattern < 10; ++pattern)
    {
        for (size_t n = 0; n < 256 * length; ++n)
        {
            float input = 0;
            random = random * UINT32_C(1664525) + UINT32_C(1013904223);
            const float noise = (float)(random >> 8) / 8388608.f - 1.f;
            /* Zero flushes preserve continuous state between patterns. Impulses
             * spaced N+1 apart exercise every ring alignment after startup.
             */
            if (n >= length)
            {
                switch (pattern)
                {
                    case 0: input = (n % (length + 1) == 0) ? 1.f : 0.f; break;
                    case 1: input = 1.f; break;
                    case 2: input = -1.f; break;
                    case 3: input = n & 1 ? 1.f : -1.f; break;
                    case 4: input = noise; break;
                    case 5: input = noise * 32768.f; break;
                    case 6: input = noise * 1e-10f; break;
                    case 7: input = n & 1 ? 1.f : -0.99999988f; break;
                    case 8: input = n & 1 ? 0.f : -0.f; break;
                    case 9: input = noise * 0x1p-140f; break;
                }
            }
            const float expected = firf(input, &reference);
            const float actual = firf_symmetric(input, &folded);
            const float contiguous = firf_symmetric_mirrored(input, &mirrored);
            const float fixed = fixed_filter(input);
            double magnitude = 0;
            for (size_t k = 0; k < length; ++k)
            {
                const size_t at = (reference.i + length - 1 + length - k) % length;
                magnitude += fabs((double)coefficients[k] * reference_history[at]);
            }
            const double bound = 2e-6 * magnitude + 1e-7;
            const double error = fabs((double)actual - expected);
            if (error / bound > max_error_ratio) max_error_ratio = error / bound;
            if ((actual >= 0) != (expected >= 0))
            {
                ++sign_differences;
                if (fabs((double)expected) > max_sign_magnitude)
                    max_sign_magnitude = fabs((double)expected);
            }
            CHECK(isfinite(actual) && isfinite(fixed));
            CHECK(memcmp(&contiguous, &actual, sizeof(float)) == 0);
            CHECK(memcmp(&fixed, &actual, sizeof(float)) == 0);
            CHECK(error <= bound);
            CHECK(fabs((double)fixed - expected) <= bound);
            CHECK(folded.i == reference.i);
            CHECK(mirrored.i == folded.i);
            CHECK(memcmp(reference_history, folded_history,
                         length * sizeof(float)) == 0);
            CHECK(memcmp(folded_history, mirrored.hist,
                         length * sizeof(float)) == 0);
            CHECK(memcmp(mirrored.hist, mirrored.hist + length,
                         length * sizeof(float)) == 0);
            CHECK(mirrored_storage[0] == 12345.25f);
            CHECK(mirrored_storage[2 * length + 1] == -9876.5f);
        }
    }
    printf("PASS symmetric FIR N=%zu: max error/bound %.6f, %zu sign differences (reference magnitude <= %.9g)\n",
           length, max_error_ratio, sign_differences, max_sign_magnitude);
    return EXIT_SUCCESS;
}

static int test_mirrored_filter_alignment(size_t length, float *coefficients)
{
    float reference_history[length];
    float storage[2 * length + 2];
    FIRF_FILTER reference = {.length = length, .b = coefficients,
                            .hist = reference_history};
    FIRF_FILTER mirrored = {.length = length, .b = coefficients,
                           .hist = storage + 1};
    uint32_t random = 1;

    for (size_t start = 0; start < length; ++start)
    {
        reference.i = mirrored.i = start;
        storage[0] = 12345.25f;
        storage[2 * length + 1] = -9876.5f;
        for (size_t k = 0; k < length; ++k)
        {
            random = random * UINT32_C(1664525) + UINT32_C(1013904223);
            reference_history[k] = (float)(random >> 8) / 8388608.f - 1.f;
            mirrored.hist[k] = mirrored.hist[k + length] = reference_history[k];
        }
        for (size_t n = 0; n < 8 * length; ++n)
        {
            random = random * UINT32_C(1664525) + UINT32_C(1013904223);
            const float input = (float)(random >> 8) / 8388608.f - 1.f;
            const float expected = firf_symmetric(input, &reference);
            const float actual = firf_symmetric_mirrored(input, &mirrored);
            CHECK(memcmp(&actual, &expected, sizeof(float)) == 0);
            CHECK(mirrored.i == reference.i);
            CHECK(memcmp(reference_history, mirrored.hist,
                         sizeof(reference_history)) == 0);
            CHECK(memcmp(mirrored.hist, mirrored.hist + length,
                         sizeof(reference_history)) == 0);
            CHECK(storage[0] == 12345.25f);
            CHECK(storage[2 * length + 1] == -9876.5f);
        }
    }
    return EXIT_SUCCESS;
}

static int test_symmetric_filters(void)
{
    float t1_c1[] = {-0.00456638213, -0.002571450348, 0.02689425925, 0.1141330398, 0.2264456422, 0.2793297826, 0.2264456422, 0.1141330398, 0.02689425925, -0.002571450348, -0.00456638213, };
    float s1[] = {-0.000649081282, -0.0009491938209, -0.001361601657, -0.001910785234, -0.002570133495, -0.003251218426, -0.003801634695, -0.004012672882, -0.003636803575, -0.002413585945, -0.0001013597693, 0.003488892085, 0.008461671287, 0.01481127545, 0.02240598045, 0.03098477999, 0.0401679839, 0.04948137286, 0.05839197924, 0.06635211627, 0.07284719662, 0.07744230649, 0.07982251613, 0.07982251613, 0.07744230649, 0.07284719662, 0.06635211627, 0.05839197924, 0.04948137286, 0.0401679839, 0.03098477999, 0.02240598045, 0.01481127545, 0.008461671287, 0.003488892085, -0.0001013597693, -0.002413585945, -0.003636803575, -0.004012672882, -0.003801634695, -0.003251218426, -0.002570133495, -0.001910785234, -0.001361601657, -0.0009491938209, -0.000649081282, };
    CHECK(test_symmetric_filter(11, t1_c1, lp_fir_butter_800kHz_100kHz_160kHz) == EXIT_SUCCESS);
    CHECK(test_symmetric_filter(46, s1, lp_fir_butter_800kHz_32kHz_36kHz) == EXIT_SUCCESS);
    CHECK(test_mirrored_filter_alignment(11, t1_c1) == EXIT_SUCCESS);
    CHECK(test_mirrored_filter_alignment(46, s1) == EXIT_SUCCESS);
    return EXIT_SUCCESS;
}

/* Full-rate reference for the C1 fast-path decimator: the three-fold
 * convolution of length-R boxcars, evaluated directly in 64-bit integers.
 */
static size_t cic3_reference_taps(unsigned rate, int64_t *taps)
{
    const size_t length = 3 * (rate - 1) + 1;
    int64_t box[3 * 8] = {0}, next[3 * 8];
    box[0] = 1;
    for (unsigned stage = 0; stage < 3; ++stage)
    {
        memset(next, 0, sizeof(next));
        for (size_t n = 0; n < length; ++n)
            for (unsigned k = 0; k < rate && k <= n; ++k)
                next[n] += box[n - k];
        memcpy(box, next, sizeof(box));
    }
    memcpy(taps, box, length * sizeof(*taps));
    return length;
}

static int test_cic3_impulse_and_dc(unsigned rate)
{
    int64_t taps[3 * 8];
    const size_t length = cic3_reference_taps(rate, taps);
    int64_t sum = 0;
    for (size_t n = 0; n < length; ++n) sum += taps[n];
    CHECK(sum == (int64_t)rate * rate * rate);
    CHECK(taps[0] == 1 && taps[length - 1] == 1);

    /* An impulse at input phase p yields full-rate taps R-1-p, 2R-1-p, ... */
    for (unsigned p = 0; p < rate; ++p)
    {
        struct cic3_decimator cic;
        cic3_reset(&cic, rate);
        size_t outputs = 0;
        for (size_t n = 0; n < 4 * rate + length; ++n)
        {
            float i, q;
            const int x = n == p ? 1 : 0;
            if (!cic3_decimate(&cic, x, -x, &i, &q)) continue;
            const size_t at = (outputs + 1) * rate - 1;
            const int64_t expected = at >= p && at - p < length ? taps[at - p] : 0;
            CHECK(i == (float)expected * cic.scale);
            CHECK(q == -(float)expected * cic.scale);
            ++outputs;
        }
        CHECK(outputs == (4 * rate + length) / rate);
    }

    /* Unity DC gain after normalization, for the full centered input range. */
    for (int level = -127; level <= 127; level += 127)
    {
        struct cic3_decimator cic;
        cic3_reset(&cic, rate);
        float i = 0, q = 0;
        for (size_t n = 0; n < 8 * rate; ++n) cic3_decimate(&cic, level, level / 2, &i, &q);
        CHECK(fabsf(i - (float)level) <= 1e-5f * 127);
        CHECK(fabsf(q - (float)(level / 2)) <= 1e-5f * 127);
    }
    return EXIT_SUCCESS;
}

/* Wrapped unsigned integrators must match the exact FIR across 2048-sample
 * input blocks, whose length is not a multiple of every rate.
 */
static int test_cic3_wraparound_and_blocks(unsigned rate)
{
    enum { BLOCK = 2048, BLOCKS = 300 };
    int64_t taps[3 * 8];
    const size_t length = cic3_reference_taps(rate, taps);
    static int history[2][BLOCK * BLOCKS];
    struct cic3_decimator cic;
    cic3_reset(&cic, rate);
    uint32_t random = 7;
    size_t n = 0, outputs = 0;
    int wrapped = 0;
    for (size_t block = 0; block < BLOCKS; ++block)
    {
        for (size_t k = 0; k < BLOCK; ++k, ++n)
        {
            random = random * UINT32_C(1664525) + UINT32_C(1013904223);
            int i, q;
            switch ((n / 4096) % 4)
            {
                case 0: i = 127; q = -127; break;
                case 1: i = center_sample(random >> 24); q = center_sample((random >> 16) & 255u); break;
                case 2: i = -127; q = 127; break;
                default: i = (n & 1) ? 127 : -127; q = center_sample(random >> 24); break;
            }
            history[0][n] = i;
            history[1][n] = q;
            float i_out, q_out;
            if (!cic3_decimate(&cic, i, q, &i_out, &q_out)) continue;
            CHECK((n + 1) % rate == 0);
            int64_t expected[2] = {0, 0};
            for (size_t t = 0; t < length && t <= n; ++t)
                for (size_t c = 0; c < 2; ++c) expected[c] += taps[t] * history[c][n - t];
            CHECK(i_out == (float)expected[0] * cic.scale);
            CHECK(q_out == (float)expected[1] * cic.scale);
            ++outputs;
        }
        if (cic.integrator[0][2] < cic.integrator[0][1]) wrapped = 1;
    }
    CHECK(outputs == (size_t)BLOCK * BLOCKS / rate);
    CHECK(wrapped);
    return EXIT_SUCCESS;
}

/* Complex tones: |H(f)| = |sin(pi f R / fs) / (R sin(pi f / fs))|^3. */
static int test_cic3_magnitude(unsigned rate, const double *table_db)
{
    const double fs = 400e3 * rate;
    const double frequencies[] = {100e3, 300e3, 500e3};
    for (size_t t = 0; t < 3; ++t)
    {
        const double f = frequencies[t];
        const double ratio = sin(M_PI * f * rate / fs) / (rate * sin(M_PI * f / fs));
        const double expected_db = 20 * log10(fabs(ratio * ratio * ratio));
        struct cic3_decimator cic;
        cic3_reset(&cic, rate);
        const double amplitude = 1 << 20;
        float i = 0, q = 0;
        for (size_t n = 0; n < 64 * rate; ++n)
            cic3_decimate(&cic, (int)lrint(amplitude * cos(2 * M_PI * f * n / fs)),
                          (int)lrint(amplitude * sin(2 * M_PI * f * n / fs)), &i, &q);
        const double measured_db = 20 * log10(fmax(hypot(i, q), 1e-9) / amplitude);
        CHECK(fabs(measured_db - expected_db) <= 0.1);
        if (table_db) CHECK(fabs(measured_db - table_db[t]) <= 0.5);
        printf("PASS CIC R=%u at %.0f kHz: %.2f dB (analytic %.2f dB)\n",
               rate, f / 1e3, measured_db, expected_db);
    }
    return EXIT_SUCCESS;
}

static int test_cic3(void)
{
    for (unsigned u = 0; u < 256; ++u)
        CHECK(center_sample((uint8_t)u) == (int)((float)u - 127.5f));
    /* #27 table for R = 4 (droop, near and far alias edges). */
    const double table_r4[] = {-2.6, -30, -40};
    for (unsigned rate = C1_FAST_MIN_DECIMATION_RATE; rate <= C1_FAST_MAX_DECIMATION_RATE; ++rate)
    {
        CHECK(test_cic3_impulse_and_dc(rate) == EXIT_SUCCESS);
        CHECK(test_cic3_wraparound_and_blocks(rate) == EXIT_SUCCESS);
        if (rate == 3 || rate == 4 || rate == 6)
            CHECK(test_cic3_magnitude(rate, rate == 4 ? table_r4 : NULL) == EXIT_SUCCESS);
    }
    puts("PASS CIC impulse, DC gain, wrap-around and block phase for R=3..8");
    return EXIT_SUCCESS;
}

/* Keep the original idle behavior as an independent transition reference.
 * Compare every byte after every bit, including storage cleared by real resets.
 */
static struct t1_c1_packet_decoder_work idle_t1[2], reference_t1[2];
static struct s1_packet_decoder_work idle_s1[2], reference_s1[2];

static int decoder_bit(unsigned channel, unsigned bit, unsigned rssi)
{
    struct t1_c1_packet_decoder_work *t = &reference_t1[channel];
    struct s1_packet_decoder_work *s = &reference_s1[channel];
    if (t->state == t1_c1_decoder_states && !(bit & PACKET_PREAMBLE_DETECTED_MASK))
        reset_t1_c1_packet_decoder(t);
    else
        t1_c1_packet_decoder(bit, rssi, t, "");
    t1_c1_packet_decoder(bit, rssi, &idle_t1[channel], "");
    CHECK(memcmp(t, &idle_t1[channel], sizeof(*t)) == 0);

    if (s->state == s1_decoder_states && !(bit & PACKET_PREAMBLE_DETECTED_MASK))
        reset_s1_packet_decoder(s);
    else
        s1_packet_decoder(bit, rssi, s, "");
    s1_packet_decoder(bit, rssi, &idle_s1[channel], "");
    CHECK(memcmp(s, &idle_s1[channel], sizeof(*s)) == 0);
    return EXIT_SUCCESS;
}

static int decoder_word(unsigned channel, unsigned value, unsigned bits)
{
    while (bits--)
        CHECK(decoder_bit(channel, (value >> bits) & 1u,
                          PACKET_CAPTURE_THRESHOLD) == EXIT_SUCCESS);
    return EXIT_SUCCESS;
}

static int decoder_manchester_byte(unsigned channel, unsigned byte)
{
    for (unsigned bit = 8; bit-- > 0;)
        CHECK(decoder_word(channel, (byte >> bit) & 1u ? 1u : 2u, 2) == EXIT_SUCCESS);
    return EXIT_SUCCESS;
}

static unsigned encode_3outof6(unsigned nibble)
{
    for (unsigned code = 0; code < 64; ++code)
        if (LOW_NIBBLE_3OUTOF6[code] == nibble) return code;
    return 0;
}

static int decoder_t1_byte(unsigned channel, unsigned byte)
{
    CHECK(decoder_word(channel, encode_3outof6(byte >> 4), 6) == EXIT_SUCCESS);
    CHECK(decoder_word(channel, encode_3outof6(byte & 15u), 6) == EXIT_SUCCESS);
    return EXIT_SUCCESS;
}

static int reset_test_decoders(void)
{
    for (unsigned channel = 0; channel < 2; ++channel)
    {
        /* Poison old payload/timestamp and control state before real entry to
         * idle: startup, abort and completion must still fully initialize it.
         */
        memset(&idle_t1[channel], 0xa5, sizeof(idle_t1[channel]));
        memset(&idle_s1[channel], 0x5a, sizeof(idle_s1[channel]));
        reset_t1_c1_packet_decoder(&idle_t1[channel]);
        reset_s1_packet_decoder(&idle_s1[channel]);
        reset_t1_c1_packet_decoder(&reference_t1[channel]);
        reset_s1_packet_decoder(&reference_s1[channel]);
        CHECK(memcmp(&idle_t1[channel], &reference_t1[channel], sizeof(idle_t1[channel])) == 0);
        CHECK(memcmp(&idle_s1[channel], &reference_s1[channel], sizeof(idle_s1[channel])) == 0);
    }
    return EXIT_SUCCESS;
}

static void erase_test_timestamp(char *line)
{
    /* Only wall-clock field 3 is nondeterministic, as in capture replays. */
    for (unsigned field = 0; field < 3; ++field)
    {
        line = strchr(line, ';');
        if (!line) return;
        ++line;
    }
    char *end = strchr(line, ';');
    if (end) memset(line, ' ', (size_t)(end - line));
}

static int test_idle_decoders(void)
{
    FILE *output = tmpfile();
    CHECK(output != NULL);
    fflush(stdout);
    const int saved_stdout = dup(STDOUT_FILENO);
    CHECK(saved_stdout >= 0);
    CHECK(dup2(fileno(output), STDOUT_FILENO) >= 0);
    CHECK(reset_test_decoders() == EXIT_SUCCESS);
    for (unsigned i = 0; i < 4096; ++i)
        CHECK(decoder_bit(i & 1u, i & 1u, i % (PACKET_CAPTURE_THRESHOLD + 2)) == EXIT_SUCCESS);

    /* Valid frames, CRC-invalid frames, shortest/longest L-fields and repeated
     * frames immediately after completion. Both instances interleave bits.
     */
    const uint8_t valid[] = {9, 0x44, 0x65, 0x32, 0x41, 0x85, 0x99, 4, 1, 8, 0x33, 0x58};
    const unsigned lengths[] = {9, 0, 1, 10, 255};
    for (unsigned mode = 0; mode < 4; ++mode)
        for (size_t length = 0; length < sizeof(lengths) / sizeof(lengths[0]); ++length)
        {
            CHECK(reset_test_decoders() == EXIT_SUCCESS);
            for (unsigned repeat = 0; repeat < 2; ++repeat)
            {
                for (unsigned channel = 0; channel < 2; ++channel)
                    CHECK(decoder_bit(channel, PACKET_PREAMBLE_DETECTED_MASK,
                                      PACKET_CAPTURE_THRESHOLD) == EXIT_SUCCESS);
                if (mode >= 2)
                    for (unsigned channel = 0; channel < 2; ++channel)
                    {
                        CHECK(decoder_word(channel, mode == 2 ? C1_MODE_A : C1_MODE_B, 12) == EXIT_SUCCESS);
                        CHECK(decoder_word(channel, C1_MODE_AB_TRAILER, 4) == EXIT_SUCCESS);
                    }
                const unsigned L = lengths[length];
                const unsigned bytes = mode == 3 ? (unsigned)get_mode_b_tlg_length(L)
                                                : FULL_TLG_LENGTH_FROM_L_FIELD[L];
                for (unsigned i = 0; i < bytes; ++i)
                    for (unsigned channel = 0; channel < 2; ++channel)
                    {
                        unsigned byte = L == 9 && i < sizeof(valid) ? valid[i] : i & 255u;
                        if (i == 0) byte = L;
                        if (repeat && i == 2) byte ^= 1u;
                        CHECK((mode == 0 ? decoder_t1_byte(channel, byte) :
                               mode == 1 ? decoder_manchester_byte(channel, byte) :
                                           decoder_word(channel, byte, 8)) == EXIT_SUCCESS);
                    }
            }
        }

    CHECK(reset_test_decoders() == EXIT_SUCCESS);
    for (unsigned channel = 0; channel < 2; ++channel)
    {
        CHECK(decoder_bit(channel, PACKET_PREAMBLE_DETECTED_MASK,
                          PACKET_CAPTURE_THRESHOLD) == EXIT_SUCCESS);
        CHECK(decoder_word(channel, 0, 12) == EXIT_SUCCESS);
        CHECK(!in_rx_t1_c1_packet_decoder(&idle_t1[channel]));
        CHECK(!in_rx_s1_packet_decoder(&idle_s1[channel]));
        CHECK(decoder_bit(channel, PACKET_PREAMBLE_DETECTED_MASK,
                          PACKET_CAPTURE_THRESHOLD) == EXIT_SUCCESS);
        CHECK(decoder_word(channel, C1_MODE_A, 12) == EXIT_SUCCESS);
        CHECK(decoder_word(channel, 0, 4) == EXIT_SUCCESS);
        CHECK(!in_rx_t1_c1_packet_decoder(&idle_t1[channel]));
    }

    /* Abort at every position of a truncated header/data sequence, including
     * below/at RSSI threshold, invalid Manchester, 3-of-6 and C1 trailer.
     */
    for (unsigned cut = 0; cut < 96; ++cut)
    {
        CHECK(reset_test_decoders() == EXIT_SUCCESS);
        for (unsigned channel = 0; channel < 2; ++channel)
            CHECK(decoder_bit(channel, PACKET_PREAMBLE_DETECTED_MASK,
                              PACKET_CAPTURE_THRESHOLD) == EXIT_SUCCESS);
        for (unsigned i = 0; i < cut; ++i)
            CHECK(decoder_bit(i & 1u, i % 3 == 0, PACKET_CAPTURE_THRESHOLD) == EXIT_SUCCESS);
        for (unsigned channel = 0; channel < 2; ++channel)
        {
            CHECK(decoder_bit(channel, PACKET_PREAMBLE_DETECTED_MASK,
                              PACKET_CAPTURE_THRESHOLD - 1) == EXIT_SUCCESS);
            CHECK(decoder_bit(channel, PACKET_PREAMBLE_DETECTED_MASK,
                              PACKET_CAPTURE_THRESHOLD) == EXIT_SUCCESS);
        }
    }
    uint32_t random = 9;
    for (unsigned i = 0; i < 65536; ++i)
    {
        random = random * 1664525u + 1013904223u;
        unsigned bit = random >> 31;
        if ((random & 127u) == 0) bit |= PACKET_PREAMBLE_DETECTED_MASK;
        CHECK(decoder_bit((random >> 8) & 1u, bit,
                          (random >> 16) % (PACKET_CAPTURE_THRESHOLD + 2)) == EXIT_SUCCESS);
    }
    fflush(stdout);
    CHECK(dup2(saved_stdout, STDOUT_FILENO) >= 0);
    close(saved_stdout);
    rewind(output);
    char reference[2048], candidate[2048];
    unsigned rows = 0;
    while (fgets(reference, sizeof(reference), output))
    {
        CHECK(fgets(candidate, sizeof(candidate), output) != NULL);
        erase_test_timestamp(reference);
        erase_test_timestamp(candidate);
        CHECK(strcmp(reference, candidate) == 0);
        ++rows;
    }
    CHECK(rows >= 40);
    fclose(output);
    printf("PASS idle decoder transitions and %u complete output pairs\n", rows);
    return EXIT_SUCCESS;
}

/* Steady-state gain of a fixed real filter for a cosine at f / fs, by
 * quadrature correlation over 4000 samples after settling.
 */
static double tone_gain(float (*filter)(float), double frequency, size_t settle)
{
    double re = 0, im = 0;
    const size_t window = 4000;
    for (size_t n = 0; n < settle + window; ++n)
    {
        const double w = 2 * M_PI * frequency * n;
        const float y = filter((float)cos(w));
        if (n < settle) continue;
        re += y * cos(w);
        im -= y * sin(w);
    }
    return (frequency == 0 ? 1 : 2) * hypot(re, im) / window;
}

static int test_c1_fast_fir(void)
{
    /* Flush, then read the impulse response through the production filter. */
    for (size_t n = 0; n < 16; ++n) lp_fir_butter_400kHz_100kHz_160kHz(0);
    float taps[8];
    double sum = 0;
    for (size_t n = 0; n < 8; ++n)
    {
        taps[n] = lp_fir_butter_400kHz_100kHz_160kHz(n == 0 ? 1.f : 0.f);
        sum += taps[n];
    }
    CHECK(taps[6] == 0 && taps[7] == 0);
    for (size_t n = 0; n < 3; ++n) CHECK(taps[n] == taps[5 - n]);
    CHECK(fabs(sum - 1) <= 1e-3);
    /* fs = 400 kHz: same pass band as the 800 kHz design (-4.8 dB at 100 kHz). */
    const struct { double f, low_db, high_db; } bands[] = {
        {0, -0.01, 0.01}, {50e3, -1.1, -0.8}, {100e3, -4.8, -4.2},
        {160e3, -15.5, -13.5}, {199e3, -200, -40},
    };
    for (size_t b = 0; b < sizeof(bands) / sizeof(bands[0]); ++b)
    {
        const double db = 20 * log10(tone_gain(lp_fir_butter_400kHz_100kHz_160kHz, bands[b].f / 400e3, 64) + 1e-12);
        CHECK(db >= bands[b].low_db && db <= bands[b].high_db);
        printf("PASS fast FIR %.0f kHz: %.2f dB\n", bands[b].f / 1e3, db);
    }
    return EXIT_SUCCESS;
}

static int test_c1_fast_resonator(void)
{
    const double settle = 2000;
    const double peak = tone_gain(bp_resonator_400kHz_100kHz, 0.25, settle);
    const double r2 = C1_FAST_CLOCK_POLE_RADIUS_SQUARED;
    CHECK(fabs(peak - 2 / (1 - r2)) <= 1e-3 * peak); // |H(fs/4)| = 2 / (1 - r^2)
    for (double offset = 100; offset <= 10e3; offset *= 10)
    {
        CHECK(tone_gain(bp_resonator_400kHz_100kHz, 0.25 + offset / 400e3, settle) < peak);
        CHECK(tone_gain(bp_resonator_400kHz_100kHz, 0.25 - offset / 400e3, settle) < peak);
    }
    /* 3 dB bandwidth by bisection on each side of the peak. */
    double edge[2];
    for (int side = 0; side < 2; ++side)
    {
        double inside = 0, outside = 20e3;
        for (int k = 0; k < 30; ++k)
        {
            const double mid = (inside + outside) / 2;
            const double f = 0.25 + (side ? mid : -mid) / 400e3;
            if (tone_gain(bp_resonator_400kHz_100kHz, f, settle) >= peak / sqrt(2)) inside = mid;
            else outside = mid;
        }
        edge[side] = inside;
    }
    const double bandwidth = edge[0] + edge[1];
    CHECK(fabs(edge[0] - edge[1]) <= 20);
    CHECK(bandwidth >= 1150 && bandwidth <= 1400);
    /* Settling: impulse response envelope falls to 1/e after 1/(1 - r) samples. */
    for (size_t n = 0; n < 4000; ++n) bp_resonator_400kHz_100kHz(0);
    float start = 0, envelope = 0;
    size_t settling = 0;
    for (size_t n = 0; n < 1000 && !settling; ++n)
    {
        const float y = fabsf(bp_resonator_400kHz_100kHz(n == 0 ? 1.f : 0.f));
        if (n < 4 && y > start) start = y;
        if (n >= 4 && n % 4 == 0) envelope = 0;
        if (y > envelope) envelope = y;
        if (n >= 8 && n % 4 == 3 && envelope < start / M_E) settling = n;
    }
    const double tau = 1 / (1 - sqrt(r2));
    CHECK(settling >= tau - 8 && settling <= tau + 8);
    printf("PASS fast clock resonator: peak %.1f at fs/4, 3 dB bandwidth %.0f Hz, 1/e settling %zu samples (%.1f bits)\n",
           peak, bandwidth, settling, settling / 4.0);
    return EXIT_SUCCESS;
}

/* Original 800 kS/s run-length evaluator, kept as the reference for the
 * samples-per-bit parametrization.
 */
static void reference_runlength_reset(struct runlength_algorithm_t1_c1 *algo)
{
    algo->run_length = 0;
    algo->bit_length = 8 * 256;
    algo->cum_run_length_error = 0;
    algo->state = 0u;
    algo->raw_bitstream = 0;
    algo->bitstream = 0;
    reset_t1_c1_packet_decoder(&algo->decoder);
}

static void reference_runlength(unsigned raw_bit, unsigned rssi, struct runlength_algorithm_t1_c1 *algo)
{
    algo->raw_bitstream = (algo->raw_bitstream << 1) | raw_bit;
    const unsigned state = deglitch_filter_t1_c1[algo->raw_bitstream & 0x3Fu];
    if (algo->state == state)
    {
        algo->run_length++;
        return;
    }
    if (algo->run_length < 5)
    {
        reference_runlength_reset(algo);
        algo->state = state;
        algo->run_length = 1;
        return;
    }
    algo->run_length *= 256;
    const int half_bit_length = algo->bit_length / 2;
    if (algo->run_length <= half_bit_length)
    {
        reference_runlength_reset(algo);
        algo->state = state;
        algo->run_length = 1;
        return;
    }
    int num_of_bits_rx;
    for (num_of_bits_rx = 0; algo->run_length > half_bit_length; num_of_bits_rx++)
    {
        algo->run_length -= algo->bit_length;
        unsigned bit = algo->state;
        algo->bitstream = (algo->bitstream << 1) | bit;
        if (count_set_bits((algo->bitstream & ACCESS_CODE_T1_C1_BITMASK) ^ ACCESS_CODE_T1_C1) <= ACCESS_CODE_T1_C1_ERRORS)
            bit |= (1u<<PACKET_PREAMBLE_DETECTED_SHIFT);
        t1_c1_packet_decoder(bit, rssi, &algo->decoder, "rla;");
    }
    algo->cum_run_length_error += algo->run_length;
    algo->bit_length += (algo->run_length + algo->cum_run_length_error / 16) / (32 * num_of_bits_rx);
    algo->state = state;
    algo->run_length = 1;
}

/* C1 mode A test frame (L = 25: two CRC blocks) whose data block holds runs
 * of exactly 16 zeros and 16 ones, as in 0x0000 and 0xFFFF data words.
 */
static size_t c1_test_frame_bits(uint8_t *bits, uint32_t serial, char *payload_hex)
{
    uint8_t data[30] = {25, 0x44, 0x2d, 0x2c,
                        serial & 255u, (serial >> 8) & 255u, (serial >> 16) & 255u, serial >> 24,
                        0x1b, 0x16};
    uint8_t block[16] = {0x55, 0x00, 0x00, 0xaa, 0xff, 0xff, 0x55, 0x7a, 0x01, 0x80, 0x3c, 0xc3, 0x0f, 0xf0, 0x5a, 0xa5};
    uint16_t crc = calc_crc_wmbus(data, 10);
    data[10] = crc >> 8;
    data[11] = crc & 255u;
    memcpy(&data[12], block, 16);
    crc = calc_crc_wmbus(&data[12], 16);
    data[28] = crc >> 8;
    data[29] = crc & 255u;
    size_t n = 0;
    for (unsigned k = 0; k < 32; ++k) bits[n++] = k & 1u;
    const unsigned words[2] = {0x543D, 0x54CD};
    for (unsigned w = 0; w < 2; ++w)
        for (unsigned k = 16; k-- > 0;) bits[n++] = (words[w] >> k) & 1u;
    for (unsigned byte = 0; byte < sizeof(data); ++byte)
        for (unsigned k = 8; k-- > 0;) bits[n++] = (data[byte] >> k) & 1u;
    for (unsigned k = 0; k < 8; ++k) bits[n++] = k & 1u;
    char *out = payload_hex;
    out += sprintf(out, "0x");
    for (unsigned byte = 0; byte < 10; ++byte) out += sprintf(out, "%02x", data[byte]);
    for (unsigned byte = 12; byte < 28; ++byte) out += sprintf(out, "%02x", data[byte]);
    return n;
}

/* Expand bits to chips of samples_per_bit samples with timing jitter: every
 * edge falls up to window samples after its nominal sample, so a run of m bits
 * lasts m * samples_per_bit +- window samples. At 4 samples per bit, window 1
 * is +-1 sample; two samples short would be the half-bit decision tie.
 */
static size_t jittered_chips(const uint8_t *bits, size_t count, uint8_t *chips, uint32_t *random,
                             int samples_per_bit, int window)
{
    size_t n = 0;
    int offset = 0;
    for (size_t k = 0; k < count; ++k)
    {
        *random = *random * UINT32_C(1664525) + UINT32_C(1013904223);
        const int next = (int)((*random >> 16) % (unsigned)(window + 1));
        const int length = samples_per_bit + next - offset;
        for (int s = 0; s < length; ++s) chips[n++] = bits[k];
        offset = next;
    }
    return n;
}

static int count_rows(FILE *output, const char *prefix, uint32_t serial, const char *payload, unsigned *crc_ok)
{
    char line[1024], serial_text[16];
    int rows = 0;
    snprintf(serial_text, sizeof(serial_text), ";%08X;", (unsigned)serial);
    rewind(output);
    while (fgets(line, sizeof(line), output))
    {
        if (strncmp(line, prefix, strlen(prefix)) || !strstr(line, serial_text)) continue;
        ++rows;
        const char *last = strrchr(line, ';');
        if (strncmp(line + strlen(prefix), "C1;1;1;", 7) == 0 && last &&
            strncmp(last + 1, payload, strlen(payload)) == 0) ++*crc_ok;
    }
    return rows;
}

/* NRZ FSK (+-45 kHz) through a chain; edges fall half a sample after the
 * jittered sample grid, so samples spanning an edge mix both frequencies.
 */
static void fsk_frames(int fast, uint32_t base, unsigned frames, int window, uint32_t *random,
                       struct time2_algorithm_t1_c1 *t2, struct runlength_algorithm_t1_c1 *rl)
{
    static uint8_t bits[512], chips[8192];
    char payload[128];
    const int samples_per_bit = fast ? 4 : 8;
    const double rate = fast ? 400e3 : 800e3;
    double phase = 0;
    for (unsigned f = 0; f < frames; ++f)
    {
        const size_t count = c1_test_frame_bits(bits, base + f, payload);
        const size_t length = jittered_chips(bits, count, chips, random, samples_per_bit, window);
        const size_t lead = 100 * samples_per_bit;
        for (size_t n = 0; n < lead + length + lead / 2; ++n)
        {
            const int on = n >= lead && n < lead + length;
            if (on)
            {
                const size_t k = n - lead;
                const double now = chips[k] ? 1 : -1;
                const double before = k ? (chips[k - 1] ? 1 : -1) : now;
                phase += 2 * M_PI * 45e3 * (now + before) / 2 / rate;
            }
            const float i = on ? 40 * (float)cos(phase) : 0.f;
            const float q = on ? 40 * (float)sin(phase) : 0.f;
            if (fast) c1_fast_signal_chain(i, q, t2, rl);
            else t1_c1_signal_chain(i, q, t2, rl, polar_discriminator_t1_c1);
        }
    }
}

static void frame_results(FILE *output, uint32_t base, unsigned frames, unsigned *rla, unsigned *t2a)
{
    static uint8_t bits[512];
    char payload[128];
    for (unsigned f = 0; f < frames; ++f)
    {
        c1_test_frame_bits(bits, base + f, payload);
        count_rows(output, "rla;", base + f, payload, rla);
        count_rows(output, "t2a;", base + f, payload, t2a);
    }
}

static int test_c1_fast_bit_recovery(void)
{
    static uint8_t bits[512], chips[4096];
    char payload[128];
    const unsigned frames = 64;
    uint32_t random = 27;
    FILE *output = tmpfile();
    CHECK(output != NULL);
    fflush(stdout);
    const int saved_stdout = dup(STDOUT_FILENO);
    CHECK(saved_stdout >= 0);
    CHECK(dup2(fileno(output), STDOUT_FILENO) >= 0);
    opts_show_used_algorithm = 1;
    opts_remove_dc_offset = 0;

    /* Run-length path on raw sliced bits, +-1 sample of jitter. */
    struct runlength_algorithm_t1_c1 rl;
    runlength_algorithm_reset_timed_t1_c1(&rl, &RUNLENGTH_TIMING_C1_FAST);
    for (unsigned f = 0; f < frames; ++f)
    {
        const size_t count = c1_test_frame_bits(bits, 0x27100000u + f, payload);
        const size_t length = jittered_chips(bits, count, chips, &random, 4, 1);
        for (size_t n = 0; n < 40 + f % 4; ++n) runlength_algorithm_timed_t1_c1(n & 1u, 40, &rl, &RUNLENGTH_TIMING_C1_FAST);
        for (size_t n = 0; n < length; ++n) runlength_algorithm_timed_t1_c1(chips[n], 40, &rl, &RUNLENGTH_TIMING_C1_FAST);
    }

    /* Both paths through the 400 kS/s chain without and with +-1 sample of
     * jitter, and the 800 kS/s chain with the same relative (+-2 sample) jitter.
     */
    struct time2_algorithm_t1_c1 t2;
    time2_algorithm_t1_c1_reset(&t2);
    runlength_algorithm_reset_timed_t1_c1(&rl, &RUNLENGTH_TIMING_C1_FAST);
    fsk_frames(1, 0x27200000u, frames, 0, &random, &t2, &rl);
    fsk_frames(1, 0x27300000u, frames, 1, &random, &t2, &rl);
    struct time2_algorithm_t1_c1 t2_800;
    struct runlength_algorithm_t1_c1 rl_800;
    time2_algorithm_t1_c1_reset(&t2_800);
    runlength_algorithm_reset_t1_c1(&rl_800);
    fsk_frames(0, 0x27400000u, frames, 2, &random, &t2_800, &rl_800);

    fflush(stdout);
    CHECK(dup2(saved_stdout, STDOUT_FILENO) >= 0);
    close(saved_stdout);
    opts_show_used_algorithm = 0;

    unsigned raw = 0, clean[2] = {0, 0}, jitter[2] = {0, 0}, reference[2] = {0, 0}, unused = 0;
    frame_results(output, 0x27100000u, frames, &raw, &unused);
    frame_results(output, 0x27200000u, frames, &clean[0], &clean[1]);
    frame_results(output, 0x27300000u, frames, &jitter[0], &jitter[1]);
    frame_results(output, 0x27400000u, frames, &reference[0], &reference[1]);
    fclose(output);
    printf("PASS fast bit recovery, CRC-OK frames of %u with 16-bit runs: raw-bit run-length +-1 jitter %u; "
           "chain run-length/time2 %u/%u, +-1 jitter %u/%u; 800 kS/s chain +-2 jitter %u/%u\n",
           frames, raw, clean[0], clean[1], jitter[0], jitter[1], reference[0], reference[1]);
    CHECK(raw == frames && clean[0] == frames && clean[1] == frames && jitter[1] == frames);
    CHECK(jitter[0] >= reference[0] && jitter[1] >= reference[1]);
    return EXIT_SUCCESS;
}

/* The 8-samples-per-bit descriptor reproduces the original evaluator state
 * after every raw bit, including all resets and decoded frames.
 */
static int test_runlength_parametrization(void)
{
    static struct runlength_algorithm_t1_c1 reference, candidate;
    static uint8_t bits[512];
    char payload[128];
    FILE *output = tmpfile();
    CHECK(output != NULL);
    fflush(stdout);
    const int saved_stdout = dup(STDOUT_FILENO);
    CHECK(saved_stdout >= 0);
    CHECK(dup2(fileno(output), STDOUT_FILENO) >= 0);
    memset(&reference, 0x5a, sizeof(reference));
    memset(&candidate, 0x5a, sizeof(candidate));
    reference_runlength_reset(&reference);
    runlength_algorithm_reset_t1_c1(&candidate);
    CHECK(memcmp(&reference, &candidate, sizeof(reference)) == 0);
    uint32_t random = 3;
    unsigned frames = 0;
    for (unsigned block = 0; block < 400; ++block)
    {
        if (block % 4 == 0)
        {
            /* A frame at 8 samples per bit with +-1 sample of edge jitter. */
            const size_t count = c1_test_frame_bits(bits, 0x28000000u + block, payload);
            int offset = 0;
            for (size_t k = 0; k < count; ++k)
            {
                random = random * UINT32_C(1664525) + UINT32_C(1013904223);
                const int next = (int)((random >> 16) % 3) - 1;
                for (int s = 0; s < 8 + next - offset; ++s)
                {
                    reference_runlength(bits[k], 40, &reference);
                    runlength_algorithm_t1_c1(bits[k], 40, &candidate);
                    CHECK(memcmp(&reference, &candidate, sizeof(reference)) == 0);
                }
                offset = next;
            }
            ++frames;
            continue;
        }
        for (unsigned n = 0; n < 2048; ++n)
        {
            random = random * UINT32_C(1664525) + UINT32_C(1013904223);
            /* Noise, glitches and runs of every length, low and high RSSI. */
            const unsigned bit = block % 4 == 1 ? random >> 31 : (n / (1 + (random >> 29) + block % 9)) & 1u;
            const unsigned rssi = (random >> 8) % 12;
            reference_runlength(bit, rssi, &reference);
            runlength_algorithm_t1_c1(bit, rssi, &candidate);
            CHECK(memcmp(&reference, &candidate, sizeof(reference)) == 0);
        }
    }
    fflush(stdout);
    CHECK(dup2(saved_stdout, STDOUT_FILENO) >= 0);
    close(saved_stdout);
    rewind(output);
    char reference_line[1024], candidate_line[1024];
    unsigned rows = 0;
    while (fgets(reference_line, sizeof(reference_line), output))
    {
        CHECK(fgets(candidate_line, sizeof(candidate_line), output) != NULL);
        erase_test_timestamp(reference_line);
        erase_test_timestamp(candidate_line);
        CHECK(strcmp(reference_line, candidate_line) == 0);
        ++rows;
    }
    fclose(output);
    CHECK(rows >= frames);
    printf("PASS 8-samples-per-bit run-length parametrization: identical state after every bit, %u output pairs\n", rows);
    return EXIT_SUCCESS;
}

int main(void)
{
    if (test_crc() || test_manchester() || test_filters() ||
        test_fixed_moving_average(8, moving_average_t1_c1) ||
        test_fixed_moving_average(16, moving_average_s1) ||
        test_symmetric_filters() || test_cic3() || test_idle_decoders() ||
        test_c1_fast_fir() || test_c1_fast_resonator() || test_runlength_parametrization() ||
        test_c1_fast_bit_recovery()) return EXIT_FAILURE;
    puts("PASS CRC, Manchester, moving average, and FIR checks");
    return EXIT_SUCCESS;
}
