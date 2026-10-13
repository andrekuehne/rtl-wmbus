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
    float folded_coefficients[(length + 1) / 2];
    FIRF_FILTER reference = {.length = length, .b = coefficients,
                            .hist = reference_history};
    FIRF_FILTER folded = {.length = length, .b = folded_coefficients,
                         .hist = folded_history};
    uint32_t random = 1;
    double max_error_ratio = 0;
    double max_sign_magnitude = 0;
    size_t sign_differences = 0;

    memcpy(folded_coefficients, coefficients, sizeof(folded_coefficients));
    for (size_t pattern = 0; pattern < 8; ++pattern)
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
                }
            }
            const float expected = firf(input, &reference);
            const float actual = firf_symmetric(input, &folded);
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
            CHECK(error <= bound);
            CHECK(fabs((double)fixed - expected) <= bound);
            CHECK(folded.i == reference.i);
            CHECK(memcmp(reference_history, folded_history,
                         length * sizeof(float)) == 0);
        }
    }
    printf("PASS symmetric FIR N=%zu: max error/bound %.6f, %zu sign differences (reference magnitude <= %.9g)\n",
           length, max_error_ratio, sign_differences, max_sign_magnitude);
    return EXIT_SUCCESS;
}

static int test_symmetric_filters(void)
{
    float t1_c1[] = {-0.00456638213, -0.002571450348, 0.02689425925, 0.1141330398, 0.2264456422, 0.2793297826, 0.2264456422, 0.1141330398, 0.02689425925, -0.002571450348, -0.00456638213, };
    float s1[] = {-0.000649081282, -0.0009491938209, -0.001361601657, -0.001910785234, -0.002570133495, -0.003251218426, -0.003801634695, -0.004012672882, -0.003636803575, -0.002413585945, -0.0001013597693, 0.003488892085, 0.008461671287, 0.01481127545, 0.02240598045, 0.03098477999, 0.0401679839, 0.04948137286, 0.05839197924, 0.06635211627, 0.07284719662, 0.07744230649, 0.07982251613, 0.07982251613, 0.07744230649, 0.07284719662, 0.06635211627, 0.05839197924, 0.04948137286, 0.0401679839, 0.03098477999, 0.02240598045, 0.01481127545, 0.008461671287, 0.003488892085, -0.0001013597693, -0.002413585945, -0.003636803575, -0.004012672882, -0.003801634695, -0.003251218426, -0.002570133495, -0.001910785234, -0.001361601657, -0.0009491938209, -0.000649081282, };
    CHECK(test_symmetric_filter(11, t1_c1, lp_fir_butter_800kHz_100kHz_160kHz) == EXIT_SUCCESS);
    CHECK(test_symmetric_filter(46, s1, lp_fir_butter_800kHz_32kHz_36kHz) == EXIT_SUCCESS);
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

int main(void)
{
    if (test_crc() || test_manchester() || test_filters() ||
        test_fixed_moving_average(8, moving_average_t1_c1) ||
        test_fixed_moving_average(16, moving_average_s1) ||
        test_symmetric_filters() || test_cic3() || test_idle_decoders()) return EXIT_FAILURE;
    puts("PASS CRC, Manchester, moving average, and FIR checks");
    return EXIT_SUCCESS;
}
