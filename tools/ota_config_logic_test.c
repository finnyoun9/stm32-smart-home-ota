/**
 * @file    ota_config_logic_test.c
 * @brief   Host-side test for the ping-pong config *selection* logic.
 *
 * Why this test exists
 * --------------------
 * `shared/ota_config.c` cannot be linked on a host: it calls the STM32 HAL
 * flash API and its erase/program helpers live in `.ramfunc`. But the part that
 * decides *which copy wins* is the safety-critical half — a bug there means
 * picking a torn or stale config after a power loss, which is exactly the
 * failure the two-copy scheme exists to prevent. That logic is pure: given two
 * candidate records (each valid or not) it must pick the valid one with the
 * higher sequence number.
 *
 * This file re-implements that selection rule and tests it against the same
 * cases the firmware must handle. It is a *model*, not the firmware: if the
 * rule in `ota_config.c` changes, this test must change with it. The
 * corresponding function is `ota_config_read()` / `next_write_page()`.
 *
 * Build:
 *   gcc -std=c11 -Wall -Wextra -Werror -Ishared \
 *       tools/ota_config_logic_test.c shared/protocol.c -o ota_config_logic_test
 */

#include "protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/*---------------------------------------------------------------------------
 * Minimal test harness
 *---------------------------------------------------------------------------*/

static unsigned g_checks;
static unsigned g_failures;

#define CHECK(cond, ...)                                            \
    do {                                                            \
        g_checks++;                                                 \
        if (!(cond)) {                                              \
            g_failures++;                                           \
            printf("  [FAIL] %s:%d: ", __FILE__, __LINE__);         \
            printf(__VA_ARGS__);                                    \
            printf("\n");                                           \
        }                                                           \
    } while (0)

/*---------------------------------------------------------------------------
 * Model of the firmware's selection rule
 *---------------------------------------------------------------------------*/

/**
 * @brief Model of `read_copy()`: a copy counts only if magic and CRC agree.
 */
static bool copy_is_valid(const BootConfig_t *cfg) {
    if (cfg->magic != BOOT_CONFIG_MAGIC) {
        return false;
    }
    uint32_t computed = proto_crc32_buf((const uint8_t *)cfg,
                                        sizeof(BootConfig_t) - sizeof(uint32_t));
    return computed == cfg->cfg_crc32;
}

/**
 * @brief Model of `ota_config_read()`: of two copies, pick the valid one with
 *        the higher sequence number. Returns false when neither is usable.
 */
static bool select_copy(const BootConfig_t *a, const BootConfig_t *b,
                        BootConfig_t *out) {
    bool a_ok = copy_is_valid(a);
    bool b_ok = copy_is_valid(b);

    if (a_ok && b_ok) {
        *out = (a->config_seq >= b->config_seq) ? *a : *b;
        return true;
    }
    if (a_ok) { *out = *a; return true; }
    if (b_ok) { *out = *b; return true; }
    return false;
}

/*---------------------------------------------------------------------------
 * Fixtures
 *---------------------------------------------------------------------------*/

static BootConfig_t make_config(uint32_t seq, uint32_t version, uint32_t confirmed) {
    BootConfig_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic         = BOOT_CONFIG_MAGIC;
    cfg.boot_mode     = BOOT_MODE_APP;
    cfg.fw_version    = version;
    cfg.config_seq    = seq;
    cfg.boot_confirmed = confirmed;
    cfg.update_status = UPDATE_STATUS_OK;
    cfg.cfg_crc32 = proto_crc32_buf((const uint8_t *)&cfg,
                                    sizeof(BootConfig_t) - sizeof(uint32_t));
    return cfg;
}

/** An erased flash page reads as all-ones: magic and CRC both fail. */
static BootConfig_t make_erased(void) {
    BootConfig_t cfg;
    memset(&cfg, 0xFF, sizeof(cfg));
    return cfg;
}

/** A copy whose CRC no longer matches its contents (power loss mid-write). */
static BootConfig_t make_torn(uint32_t seq) {
    BootConfig_t cfg = make_config(seq, 0xDEADU, 0U);
    cfg.fw_version ^= 0x1U;   /* corrupt a field, leave cfg_crc32 stale */
    return cfg;
}

/*---------------------------------------------------------------------------
 * Cases
 *---------------------------------------------------------------------------*/

static void test_both_valid_newest_wins(void) {
    printf("-- Case 01: both copies valid -> higher seq wins --\n");
    BootConfig_t a = make_config(5U, 100U, 1U);
    BootConfig_t b = make_config(6U, 200U, 1U);
    BootConfig_t out;

    CHECK(select_copy(&a, &b, &out), "expected a selection");
    CHECK(out.fw_version == 200U, "expected v200 (seq 6), got v%u", out.fw_version);

    /* Order of the arguments must not matter. */
    CHECK(select_copy(&b, &a, &out), "expected a selection (swapped)");
    CHECK(out.fw_version == 200U, "swap changed the winner: v%u", out.fw_version);
}

static void test_one_erased(void) {
    printf("-- Case 02: one copy erased -> the other wins --\n");
    BootConfig_t valid = make_config(3U, 42U, 1U);
    BootConfig_t erased = make_erased();
    BootConfig_t out;

    CHECK(select_copy(&valid, &erased, &out), "valid+erased should select");
    CHECK(out.fw_version == 42U, "expected v42, got v%u", out.fw_version);

    CHECK(select_copy(&erased, &valid, &out), "erased+valid should select");
    CHECK(out.fw_version == 42U, "expected v42, got v%u", out.fw_version);
}

static void test_torn_copy_loses(void) {
    printf("-- Case 03: interrupted write -> old copy must win --\n");
    /* This is the whole point of the two-copy scheme: the page being written
     * fails its own CRC, so the untouched page stays authoritative. */
    BootConfig_t good = make_config(7U, 500U, 1U);
    BootConfig_t torn = make_torn(8U);
    BootConfig_t out;

    CHECK(select_copy(&good, &torn, &out), "good+torn should select");
    CHECK(out.fw_version == 500U, "torn copy won: got v%u", out.fw_version);
    CHECK(out.config_seq == 7U, "expected seq 7, got %u", out.config_seq);

    CHECK(select_copy(&torn, &good, &out), "torn+good should select");
    CHECK(out.fw_version == 500U, "torn copy won (swapped): v%u", out.fw_version);
}

static void test_both_erased(void) {
    printf("-- Case 04: neither copy valid -> report no config --\n");
    BootConfig_t erased = make_erased();
    BootConfig_t out;

    CHECK(!select_copy(&erased, &erased, &out),
          "both-erased must not produce a config");
}

static void test_both_torn(void) {
    printf("-- Case 05: both copies torn -> report no config --\n");
    BootConfig_t t1 = make_torn(1U);
    BootConfig_t t2 = make_torn(2U);
    BootConfig_t out;

    CHECK(!select_copy(&t1, &t2, &out), "both-torn must not produce a config");
}

static void test_bad_magic_ignored(void) {
    printf("-- Case 06: wrong magic -> ignored even with valid CRC --\n");
    BootConfig_t wrong = make_config(9U, 900U, 1U);
    wrong.magic = 0x12345678U;
    wrong.cfg_crc32 = proto_crc32_buf((const uint8_t *)&wrong,
                                      sizeof(BootConfig_t) - sizeof(uint32_t));
    BootConfig_t good = make_config(9U, 100U, 1U);
    BootConfig_t out;

    CHECK(select_copy(&wrong, &good, &out), "expected the good copy");
    CHECK(out.fw_version == 100U, "wrong-magic copy won: v%u", out.fw_version);
}

static void test_seq_equal_prefers_first(void) {
    printf("-- Case 07: equal seq, both valid -> deterministic (>= keeps A) --\n");
    BootConfig_t a = make_config(4U, 11U, 1U);
    BootConfig_t b = make_config(4U, 22U, 1U);
    BootConfig_t out;

    CHECK(select_copy(&a, &b, &out), "expected a selection");
    CHECK(out.fw_version == 11U, "equal seq should keep A, got v%u", out.fw_version);
}

static void test_seq_wraparound_note(void) {
    printf("-- Case 08: 32-bit sequence has no realistic wrap --\n");
    /* A page erase costs ~40ms; 2^32 writes is centuries. This case simply
     * pins the assumption down so a future uint16_t shrink breaks loudly. */
    CHECK(sizeof(((BootConfig_t *)0)->config_seq) == 4U,
          "config_seq must stay 32-bit");
}

static void test_struct_layout_is_stable(void) {
    printf("-- Case 09: struct layout contract --\n");
    CHECK(sizeof(BootConfig_t) == 56U, "BootConfig_t is %zu bytes",
          sizeof(BootConfig_t));
    CHECK(sizeof(BootConfig_t) <= CONFIG_RECORD_SIZE,
          "record (%zu) must fit CONFIG_RECORD_SIZE (%u)",
          sizeof(BootConfig_t), CONFIG_RECORD_SIZE);
    /* The two copies live on different pages, so a write must never span both. */
    CHECK(CONFIG_PAGE_A != CONFIG_PAGE_B, "the two copies need distinct pages");
}

int main(void) {
    printf("========================================================\n");
    printf(" OTA config ping-pong selection test (host side)\n");
    printf(" BootConfig_t=%zu B  CONFIG_RECORD_SIZE=%u\n",
           sizeof(BootConfig_t), CONFIG_RECORD_SIZE);
    printf("========================================================\n\n");

    test_both_valid_newest_wins();
    test_one_erased();
    test_torn_copy_loses();
    test_both_erased();
    test_both_torn();
    test_bad_magic_ignored();
    test_seq_equal_prefers_first();
    test_seq_wraparound_note();
    test_struct_layout_is_stable();

    printf("\n========================================================\n");
    printf(" checks: %u/%u passed\n", g_checks - g_failures, g_checks);
    printf(" RESULT: %s\n", g_failures == 0 ? "PASS" : "FAIL");
    printf("========================================================\n");
    return g_failures == 0 ? 0 : 1;
}
