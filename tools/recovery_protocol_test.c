/**
 * @file    recovery_protocol_test.c
 * @brief   Host-side test for the recovery protocol contract.
 *
 * What this covers
 * ----------------
 * The bridge decides whether to re-send firmware based on the STM32's answer to
 * CMD_RECOVERY_CHECK. Two things about that contract have already gone wrong
 * once each during development, and both are cheap to pin down here:
 *
 *   1. Answer routing. CMD_STATUS_RSP is answered by the application with a
 *      *firmware version number* and by the bootloader with a
 *      RECOVERY_STATUS_* code. Those two meanings collide (application v1 is
 *      indistinguishable from RECOVERY_STATUS_OTA_PENDING = 1), so recovery
 *      status got its own command. This test asserts the recovery answer only
 *      ever arrives on CMD_RECOVERY_RSP, and that a version response is never
 *      mistaken for a recovery status.
 *
 *   2. Command ID uniqueness. The protocol now has enough commands that a
 *      copy-pasted ID is a realistic mistake, and a duplicate would silently
 *      misroute frames on real hardware. The switch below lists every ID.
 *
 * The parsing path exercised here mirrors `stm32_query_recovery_status()` in
 * esp32-comm-bridge/src/main.cpp: feed bytes into the shared parser, return on
 * CMD_RECOVERY_RSP, and treat CMD_OTA_COMMITTED as a separate signal.
 *
 * Build:
 *   gcc -std=c11 -Wall -Wextra -Werror -Ishared \
 *       tools/recovery_protocol_test.c shared/protocol.c -o recovery_protocol_test
 */

#include "protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/*---------------------------------------------------------------------------
 * Harness
 *---------------------------------------------------------------------------*/

static unsigned g_checks;
static unsigned g_failures;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        g_checks++;                                             \
        if (!(cond)) {                                          \
            g_failures++;                                       \
            printf("  [FAIL] %s:%d: ", __FILE__, __LINE__);     \
            printf(__VA_ARGS__);                                \
            printf("\n");                                       \
        }                                                       \
    } while (0)

/*---------------------------------------------------------------------------
 * Model of the bridge's answer routing
 *---------------------------------------------------------------------------*/

typedef struct {
    bool     answered;          /* got a reply *to this query* */
    bool     saw_other_frame;   /* saw some unrelated frame */
    bool     committed;         /* CMD_OTA_COMMITTED seen */
    uint32_t committed_version;
    uint32_t recovery_status;   /* valid when answered */
} QueryResult_t;

/**
 * @brief Model of stm32_query_recovery_status(): feed a whole byte stream in.
 *
 * Same decision order as the firmware: a CMD_RECOVERY_RSP ends the query, a
 * CMD_OTA_COMMITTED is recorded but does not end it, and everything else is
 * ignored for this purpose.
 *
 * Note `answered` vs `saw_other_frame`: only a CMD_RECOVERY_RSP counts as
 * answering the query. An unrelated frame (say the application's version
 * response, which happens to be in flight) is observed but must not be read as
 * a recovery status.
 */
static void query_from_stream(const uint8_t *stream, size_t len,
                              QueryResult_t *out) {
    ProtoParser_t parser;
    proto_parser_init(&parser);

    memset(out, 0, sizeof(*out));
    out->recovery_status = RECOVERY_STATUS_NORMAL;

    for (size_t i = 0; i < len; i++) {
        const ProtoFrame_t *f = proto_parser_feed(&parser, stream[i]);
        if (f == NULL) {
            continue;
        }

        if (f->cmd == CMD_RECOVERY_RSP && f->len >= 4) {
            memcpy(&out->recovery_status, f->payload, 4);
            out->answered = true;
            return;
        }
        if (f->cmd == CMD_OTA_COMMITTED && f->len >= 4) {
            out->committed = true;
            memcpy(&out->committed_version, f->payload, 4);
        }
        out->saw_other_frame = true;
    }
}

/** @brief Build one frame into `buf`, returning its length. */
static size_t frame(uint8_t *buf, uint16_t buf_size, uint8_t cmd,
                    const void *payload, uint16_t len) {
    return proto_build_frame(buf, buf_size, cmd,
                             (const uint8_t *)payload, len);
}

/*---------------------------------------------------------------------------
 * Cases
 *---------------------------------------------------------------------------*/

static void test_recovery_status_reaches_caller(void) {
    printf("-- Case 01: CMD_RECOVERY_RSP delivers the status --\n");

    const uint32_t statuses[] = {
        RECOVERY_STATUS_NORMAL,
        RECOVERY_STATUS_OTA_PENDING,
        RECOVERY_STATUS_IMAGE_INVALID,
        RECOVERY_STATUS_BOOT_UNCONFIRMED,
        RECOVERY_STATUS_NO_APP,
    };
    uint8_t buf[64];

    for (size_t i = 0; i < sizeof(statuses) / sizeof(statuses[0]); i++) {
        size_t n = frame(buf, sizeof(buf), CMD_RECOVERY_RSP, &statuses[i], 4);
        QueryResult_t r;
        query_from_stream(buf, n, &r);

        CHECK(r.answered, "status %lu: no answer parsed",
              (unsigned long)statuses[i]);
        CHECK(r.recovery_status == statuses[i],
              "status %lu came back as %lu",
              (unsigned long)statuses[i], (unsigned long)r.recovery_status);
    }
}

static void test_version_response_is_not_a_recovery_status(void) {
    printf("-- Case 02: a version response must not trigger recovery --\n");

    /* This is the regression that motivated CMD_RECOVERY_RSP: if recovery
     * status rode on CMD_STATUS_RSP, an application reporting version 2 would
     * be read as RECOVERY_STATUS_IMAGE_INVALID and the bridge would overwrite a
     * perfectly healthy device. */
    uint8_t buf[64];
    uint32_t version = 2;

    size_t n = frame(buf, sizeof(buf), CMD_STATUS_RSP, &version, 4);
    QueryResult_t r;
    query_from_stream(buf, n, &r);

    CHECK(!r.answered,
          "CMD_STATUS_RSP was treated as a recovery answer (status=%lu)",
          (unsigned long)r.recovery_status);

    /* And the collision is real, not hypothetical: verify the numeric overlap
     * that makes sharing one command unsafe. */
    CHECK(version == RECOVERY_STATUS_IMAGE_INVALID ||
          version == RECOVERY_STATUS_OTA_PENDING,
          "expected app version 2 to collide with a recovery code; "
          "if versions are namespaced this check can be relaxed");
}

static void test_committed_is_separate_from_status(void) {
    printf("-- Case 03: CMD_OTA_COMMITTED is recorded, not read as status --\n");

    uint8_t buf[64];
    uint32_t version = 7;

    size_t n = frame(buf, sizeof(buf), CMD_OTA_COMMITTED, &version, 4);
    QueryResult_t r;
    query_from_stream(buf, n, &r);

    CHECK(r.committed, "commit announcement was not recorded");
    CHECK(r.committed_version == 7,
          "committed version was %lu", (unsigned long)r.committed_version);
    CHECK(r.recovery_status == RECOVERY_STATUS_NORMAL,
          "commit announcement leaked into recovery status (%lu)",
          (unsigned long)r.recovery_status);
}

static void test_committed_then_status_in_one_burst(void) {
    printf("-- Case 04: commit + status in one burst --\n");

    /* The application announces a confirmed boot; the very next answer in the
     * same read burst is a recovery status. Both must be attributed correctly. */
    uint8_t buf[128];
    uint32_t version = 9;
    uint32_t status = RECOVERY_STATUS_NORMAL;

    size_t n1 = frame(buf, sizeof(buf), CMD_OTA_COMMITTED, &version, 4);
    size_t n2 = frame(buf + n1, sizeof(buf) - n1, CMD_RECOVERY_RSP, &status, 4);

    QueryResult_t r;
    query_from_stream(buf, n1 + n2, &r);

    CHECK(r.committed, "commit announcement lost when followed by a status");
    CHECK(r.committed_version == 9,
          "committed version was %lu", (unsigned long)r.committed_version);
    CHECK(r.answered, "recovery status not parsed after commit frame");
    CHECK(r.recovery_status == RECOVERY_STATUS_NORMAL,
          "status was %lu", (unsigned long)r.recovery_status);
}

static void test_garbage_before_answer_is_survivable(void) {
    printf("-- Case 05: junk before the answer does not break the query --\n");

    uint8_t buf[128];
    uint32_t status = RECOVERY_STATUS_IMAGE_INVALID;

    memset(buf, 0x00, 4);   /* leading junk */
    size_t n = frame(buf + 4, sizeof(buf) - 4, CMD_RECOVERY_RSP, &status, 4);

    QueryResult_t r;
    query_from_stream(buf, 4 + n, &r);

    CHECK(r.answered, "answer after junk was not parsed");
    CHECK(r.recovery_status == RECOVERY_STATUS_IMAGE_INVALID,
          "status was %lu", (unsigned long)r.recovery_status);
}

static void test_command_ids_are_unique(void) {
    printf("-- Case 06: every command ID is distinct --\n");

    struct { const char *name; uint8_t id; } ids[] = {
        /* master → slave */
        { "CMD_OTA_BEGIN",           CMD_OTA_BEGIN },
        { "CMD_OTA_CHUNK",           CMD_OTA_CHUNK },
        { "CMD_OTA_END",             CMD_OTA_END },
        { "CMD_OTA_ABORT",           CMD_OTA_ABORT },
        { "CMD_OTA_AVAILABLE",       CMD_OTA_AVAILABLE },
        { "CMD_APP_MSG",             CMD_APP_MSG },
        { "CMD_GET_STATUS",          CMD_GET_STATUS },
        { "CMD_RESET",               CMD_RESET },
        { "CMD_GET_SENSOR_SNAPSHOT", CMD_GET_SENSOR_SNAPSHOT },
        { "CMD_DIAG_SNAPSHOT",       CMD_DIAG_SNAPSHOT },
        { "CMD_RECOVERY_CHECK",      CMD_RECOVERY_CHECK },
        { "CMD_OTA_COMMITTED",       CMD_OTA_COMMITTED },
        /* slave → master */
        { "CMD_OTA_BEGIN_ACK",       CMD_OTA_BEGIN_ACK },
        { "CMD_CHUNK_ACK",           CMD_CHUNK_ACK },
        { "CMD_NAK",                 CMD_NAK },
        { "CMD_OTA_RESULT",          CMD_OTA_RESULT },
        { "CMD_STATUS_RSP",          CMD_STATUS_RSP },
        { "CMD_OTA_READY",           CMD_OTA_READY },
        { "CMD_SENSOR_SNAPSHOT_RSP", CMD_SENSOR_SNAPSHOT_RSP },
        { "CMD_DIAG_SNAPSHOT_RSP",   CMD_DIAG_SNAPSHOT_RSP },
        { "CMD_RECOVERY_RSP",        CMD_RECOVERY_RSP },
    };
    const size_t count = sizeof(ids) / sizeof(ids[0]);

    for (size_t i = 0; i < count; i++) {
        for (size_t j = i + 1; j < count; j++) {
            CHECK(ids[i].id != ids[j].id,
                  "duplicate command ID 0x%02X: %s and %s",
                  ids[i].id, ids[i].name, ids[j].name);
        }
    }
    CHECK(count == 21, "expected 21 command IDs, listed %u", (unsigned)count);
}

static void test_recovery_codes_are_self_consistent(void) {
    printf("-- Case 07: recovery codes stay in their documented range --\n");

    /* These values are also written into logs and compared by the bridge, so
     * they are part of the wire contract, not an implementation detail. */
    CHECK(RECOVERY_STATUS_NORMAL == 0, "NORMAL must stay 0");
    CHECK(RECOVERY_STATUS_OTA_PENDING == 1, "OTA_PENDING must stay 1");
    CHECK(RECOVERY_STATUS_IMAGE_INVALID == 2, "IMAGE_INVALID must stay 2");
    CHECK(RECOVERY_STATUS_BOOT_UNCONFIRMED == 3, "BOOT_UNCONFIRMED must stay 3");
    CHECK(RECOVERY_STATUS_NO_APP == 4, "NO_APP must stay 4");
}

int main(void) {
    printf("========================================================\n");
    printf(" Recovery protocol contract test (host side)\n");
    printf("========================================================\n\n");

    test_recovery_status_reaches_caller();
    test_version_response_is_not_a_recovery_status();
    test_committed_is_separate_from_status();
    test_committed_then_status_in_one_burst();
    test_garbage_before_answer_is_survivable();
    test_command_ids_are_unique();
    test_recovery_codes_are_self_consistent();

    printf("\n========================================================\n");
    printf(" checks: %u/%u passed\n", g_checks - g_failures, g_checks);
    printf(" RESULT: %s\n", g_failures == 0 ? "PASS" : "FAIL");
    printf("========================================================\n");
    return g_failures == 0 ? 0 : 1;
}
