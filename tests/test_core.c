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

int main(void)
{
    if (test_crc() || test_manchester() || test_filters()) return EXIT_FAILURE;
    puts("PASS CRC, Manchester, moving average, and FIR checks");
    return EXIT_SUCCESS;
}
