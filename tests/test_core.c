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

int main(void)
{
    if (test_crc() || test_manchester() || test_filters() || test_symmetric_filters()) return EXIT_FAILURE;
    puts("PASS CRC, Manchester, moving average, and FIR checks");
    return EXIT_SUCCESS;
}
