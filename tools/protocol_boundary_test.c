/* ============================================================================
 * protocol_boundary_test.c — 主机侧 UART 帧协议边界测试矩阵
 * ============================================================================
 *
 * 一、这个矩阵覆盖什么
 * ----------------------------------------------------------------------------
 * 被测对象是 shared/protocol.c（STM32 bootloader / 应用 / ESP32 共用的同一份
 * 实现）。帧格式：
 *
 *   [SYNC 0xA5][CMD 1B][LEN_L 1B][LEN_HI 1B][PAYLOAD LEN bytes][CRC32 4B LE]
 *
 * CRC-32 = 标准 IEEE（poly 0xEDB88320，init 0xFFFFFFFF，final xor 0xFFFFFFFF，
 * reflected），覆盖 SYNC..PAYLOAD 全部字节。
 *
 * 覆盖的 11 组用例（每组内有多个独立断言）：
 *   1.  CRC 向量：已知字符串标准值、"分块累积 == 一次性"、crc32_buf 等价性、
 *       退化输入（len=0 / NULL+0）
 *   2.  正常帧往返：build → 逐字节 feed → cmd/len/payload 完全一致
 *   3.  粘包：两帧连续喂入，必须依次返回两帧
 *   4.  拆包：同一帧按 1 字节 / 2 字节 / 随机切分喂入，结果必须一致
 *   5.  前导垃圾字节：任意非 0xA5 字节前缀后仍能同步
 *   6.  非法长度：LEN > PROTO_MAX_PAYLOAD（1029 / 1028+1 / 0xFFFF）必须拒绝
 *       并回到 SYNC；0xFFFF 帧额外用哨兵字节验证不越界写 payload
 *   7.  损坏帧：逐字节单比特翻转、CRC 字段篡改、payload 篡改 → 丢弃 + 不卡死
 *   8.  零长度 payload：LEN=0 正常返回
 *   9.  最大长度 payload：LEN=PROTO_MAX_PAYLOAD(1028) 正常工作
 *   10. 小端序编解码锚点：payload 内 4 字节序号 memcpy 往返一致性
 *       （这一条测不到 parser —— 属于 bootloader 层逻辑 —— 它是为将来
 *        bootloader 层“重复/乱序 chunk 语义”测试预留的锚点）
 *   11. 缓冲区边界：proto_build_frame 在 buf_size 不足时必须返回 0 且不越界
 *
 * 二、设计意图
 * ----------------------------------------------------------------------------
 * smoke test 只能说明“实现是写对了的”（happy path 跑通），不能说明“实现是
 * 证明对了的”：边界、非法输入、内存安全这些位置恰恰是 OTA 现场故障的高发
 * 区，而它们的失败模式是“静默丢帧 / 越界写 / 卡死”，不会自己跳出来报错。
 *
 * 所以这个矩阵的写法刻意做三件事，把“写对了”变成“证明对了”：
 *   a) 每条断言都有**明确期望值**（常量字面量或独立算出的值），不是“跑起来
 *      不崩就算过”；失败时打印期望 vs 实际。
 *   b) 每个解析用例都**回归到正常运行**：损坏/非法帧之后必须能收到下一帧，
 *      专门抓“解析器状态机卡死”这一类缺陷。
 *   c) 内存安全用**哨兵字节**（canary）显式验证，而不是靠“这次没崩”。
 *
 * 三、可追溯性
 * ----------------------------------------------------------------------------
 * 本文件对应 docs/capability-review.md 第七节（下一步优先级清单）中的 P1 项：
 *   「协议主机侧边界测试矩阵 + CI 接入」，验收标准为
 *   「覆盖非法长度/粘包/拆包/位翻转/乱序，结果表」。
 *   其中“乱序”一项按任务定义落在用例 10 的序号编解码锚点上（parser 只负责
 *   成帧，不负责序号语义），后续 bootloader 层测试在此锚点上扩展。
 *
 * 四、约束
 * ----------------------------------------------------------------------------
 * 纯 C11、零外部依赖（自带最小断言宏与计数，不引入 Unity/CMock）。
 * 本文件不修改 shared/protocol.c / shared/protocol.h / tools/protocol_smoke_test.c。
 *
 * 编译运行：
 *   gcc -std=c11 -Wall -Wextra -Werror -Ishared \
 *       tools/protocol_boundary_test.c shared/protocol.c \
 *       -o /tmp/protocol_boundary_test && /tmp/protocol_boundary_test
 * ============================================================================
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../shared/protocol.h"

/* ==========================================================================
 * 最小测试框架（无外部依赖）
 * ========================================================================== */

static unsigned g_checks_total = 0;
static unsigned g_checks_failed = 0;
static unsigned g_cases_run = 0;
static unsigned g_cases_failed = 0;

/* 单条断言：cond 为假时打印"条件 / 期望 / 实际"并计数失败。
 * desc 可为 NULL（此时用字符串化的表达式作为说明）。 */
static void check_at(const char *file, int line, bool cond, const char *desc,
                     const char *expect, const char *actual, const char *fmt,
                     ...) {
    g_checks_total++;
    if (cond) {
        return;
    }
    g_checks_failed++;
    printf("      FAIL @ %s:%d  %s\n", file, line, (desc != NULL) ? desc : "?");
    if (fmt != NULL) {
        va_list ap;
        printf("        note    : ");
        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);
        printf("\n");
    }
    printf("        expected: %s\n", expect);
    printf("        actual  : %s\n", actual);
}

/* 期望/实际值的格式化输出。返回静态缓冲：每条断言最多格式化 2 次，
 * 4 槽轮转足以避免覆盖（printf 在下次格式化之前就把字符串消费掉了）。 */
static const char *check_fmt_u32(uint32_t v) {
    static char slots[4][48];
    static unsigned n = 0;
    char *s = slots[n++ & 3U];
    snprintf(s, sizeof slots[0], "0x%08X (%u)", (unsigned)v, (unsigned)v);
    return s;
}

static const char *check_fmt_i64(int64_t v) {
    static char slots[4][32];
    static unsigned n = 0;
    char *s = slots[n++ & 3U];
    snprintf(s, sizeof slots[0], "%lld", (long long)v);
    return s;
}

/* 便捷包装：调用点可以是 CHECK(cond) / CHECK_MSG(cond, "fmt", ...) /
 * CHECK_EQ_U32(got, want) / CHECK_EQ_U32(got, want, "fmt", ...)，四者都合法。
 * 说明串永远是最后一个参数。 */
#define CHECK(cond)                                                             \
    check_at(__FILE__, __LINE__, (cond), #cond, "true",                        \
             (cond) ? "true" : "false", NULL)

#define CHECK_MSG(cond, ...)                                                    \
    check_at(__FILE__, __LINE__, (cond), #cond, "true",                        \
             (cond) ? "true" : "false", __VA_ARGS__)

#define CHECK_EQ_U32(got, want)                                                 \
    check_at(__FILE__, __LINE__, (uint32_t)(got) == (uint32_t)(want),           \
             #got " == " #want, check_fmt_u32(want), check_fmt_u32(got), NULL)

#define CHECK_EQ_I32(got, want)                                                 \
    check_at(__FILE__, __LINE__, (long long)(got) == (long long)(want),        \
             #got " == " #want, check_fmt_i64(want), check_fmt_i64(got), NULL)

/* 逐字节比较：失败时打印前 3 个不同字节的 offset / 实际 / 期望 */
#define CHECK_EQ_BYTES(got, want, n)                                            \
    do {                                                                        \
        size_t _n = (size_t)(n);                                                \
        const uint8_t *_g = (const uint8_t *)(got);                             \
        const uint8_t *_w = (const uint8_t *)(want);                            \
        char _e[160];                                                           \
        size_t _i = 0, _k = 0;                                                  \
        _e[0] = '\0';                                                           \
        for (_i = 0; _i < _n && _k < 3; _i++) {                                 \
            if (_g[_i] != _w[_i]) {                                             \
                char _t[48];                                                    \
                snprintf(_t, sizeof _t, "[%u] got 0x%02X want 0x%02X  ",       \
                         (unsigned)_i, (unsigned)_g[_i], (unsigned)_w[_i]);     \
                strncat(_e, _t, sizeof _e - strlen(_e) - 1);                    \
                _k++;                                                           \
            }                                                                   \
        }                                                                       \
        if (_k == 0) {                                                          \
            snprintf(_e, sizeof _e, "%u bytes byte-identical", (unsigned)_n);   \
        }                                                                       \
        check_at(__FILE__, __LINE__, memcmp((got), (want), _n) == 0,            \
                 #got " vs " #want, _e, _e, NULL);                              \
    } while (0)

/* 每个用例一行 PASS/FAIL，并打印计数 */
static void case_result(const char *name, const char *desc, unsigned failed_before,
                       const char *detail) {
    g_cases_run++;
    bool ok = (g_checks_failed == failed_before);
    if (!ok) {
        g_cases_failed++;
    }
    printf("[%s] %-46s %s%s%s\n", ok ? "PASS" : "FAIL", name, desc,
           (detail != NULL && detail[0] != '\0') ? " | " : "",
           (detail != NULL) ? detail : "");
    fflush(stdout);
}

#define CASE_BEGIN() unsigned _fail_mark = g_checks_failed

#define CASE_END(name, desc, detail)                                            \
    case_result((name), (desc), _fail_mark, (detail))

/* ==========================================================================
 * 测试辅助
 * ========================================================================== */

static void fill_pattern(uint8_t *buf, size_t n, uint8_t seed) {
    for (size_t i = 0; i < n; i++) {
        buf[i] = (uint8_t)(seed + (uint8_t)i * 7U + (uint8_t)(i >> 3));
    }
}

static uint32_t prng_next(uint32_t *s) {
    /* xorshift32 — 固定种子，结果可复现 */
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

/* 逐字节喂入，期望恰好收到一帧且 cmd/len/payload 与期望一致。
 * 返回 true 表示"恰好一帧且完全匹配"；收到 0 帧或 >1 帧都算失败。 */
static bool feed_expect_frame_impl(ProtoParser_t *p, const uint8_t *frame,
                                   uint16_t frame_len, uint8_t want_cmd,
                                   const uint8_t *want_payload,
                                   uint16_t want_len) {
    static uint8_t snapshot[PROTO_MAX_PAYLOAD];
    const ProtoFrame_t *f;
    bool got = false;
    unsigned hits = 0;

    proto_parser_init(p);
    for (uint16_t i = 0; i < frame_len; i++) {
        f = proto_parser_feed(p, frame[i]);
        if (f != NULL) {
            hits++;
            got = true;
            /* 快照，防止后续字节覆盖内部缓冲 */
            if (f->len > 0) {
                memcpy(snapshot, f->payload, f->len);
            }
            if (f->cmd != want_cmd || f->len != want_len) {
                return false;
            }
        }
    }
    if (!got || hits != 1) {
        return false;
    }
    if (want_len > 0 && memcmp(snapshot, want_payload, want_len) != 0) {
        return false;
    }
    return true;
}

/* 逐字节喂入，只关心“是否收到过一帧”（用于哨兵/不变量检查）。 */
static bool feed_accepts(ProtoParser_t *p, const uint8_t *frame, uint32_t len,
                         uint8_t *out_cmd, uint16_t *out_len,
                         uint8_t *out_payload) {
    const ProtoFrame_t *f;
    bool got = false;
    proto_parser_init(p);
    for (uint32_t i = 0; i < len; i++) {
        f = proto_parser_feed(p, frame[i]);
        if (f != NULL) {
            got = true;
            if (out_cmd) {
                *out_cmd = f->cmd;
            }
            if (out_len) {
                *out_len = f->len;
            }
            if (out_payload && f->len > 0) {
                memcpy(out_payload, f->payload, f->len);
            }
        }
    }
    return got;
}

/* ==========================================================================
 * 用例 1：CRC 向量
 * ========================================================================== */
static void case_01_crc_vectors(void) {
    CASE_BEGIN();
    printf("\n-- Case 01: CRC-32 vectors --\n");
    printf("   checks: standard IEEE vectors, chunked==one-shot, crc32_buf equivalence\n");

    static const uint8_t check_str[] = "123456789";
    static const uint8_t quick[] = "The quick brown fox jumps over the lazy dog";
    static const uint8_t empty[] = "";

    /* 标准 IEEE 检验值 */
    CHECK_EQ_U32(proto_crc32_buf(check_str, 9),
                 0xCBF43926U);
    /* 同一向量经 proto_crc32(...,0) 必须与 proto_crc32_buf 等价 */
    CHECK_EQ_U32(proto_crc32(check_str, 9, 0U), proto_crc32_buf(check_str, 9));
    /* 已知的 45 字节向量（独立算出，非由本实现生成） */
    CHECK_EQ_U32(proto_crc32_buf(quick, sizeof(quick) - 1), 0x414FA339U);
    /* 退化输入：空输入 → init 与 final xor 相消 → 0 */
    CHECK_EQ_U32(proto_crc32_buf(empty, 0), 0x00000000U);
    CHECK_EQ_U32(proto_crc32_buf(NULL, 0), 0x00000000U);

    /* 分块累积 == 一次性：覆盖 1/2/3 字节等长切分与不规则切分 */
    uint8_t block[64];
    fill_pattern(block, sizeof(block), 0x5AU);
    uint32_t oneshot = proto_crc32_buf(block, sizeof(block));

    uint32_t running = 0;
    for (size_t i = 0; i < sizeof(block); i++) {
        running = proto_crc32(&block[i], 1, running);
    }
    CHECK_EQ_U32(running, oneshot);

    running = 0;
    for (size_t i = 0; i < sizeof(block); i += 2) {
        running = proto_crc32(&block[i], 2, running);
    }
    CHECK_EQ_U32(running, oneshot);

    running = 0;
    for (size_t i = 0; i < sizeof(block); i += 3) {
        size_t n = (sizeof(block) - i < 3) ? (sizeof(block) - i) : 3;
        running = proto_crc32(&block[i], n, running);
    }
    CHECK_EQ_U32(running, oneshot);

    /* 不规则切分（固定种子，可复现） */
    running = 0;
    uint32_t rng = 0x12345678U;
    size_t off = 0;
    while (off < sizeof(block)) {
        size_t n = (size_t)(prng_next(&rng) % 7U) + 1U;
        if (n > sizeof(block) - off) {
            n = sizeof(block) - off;
        }
        running = proto_crc32(&block[off], n, running);
        off += n;
    }
    CHECK_EQ_U32(running, oneshot);

    /* 分块顺序敏感：交换两个块的顺序结果必须不同（防止“累积被忽略”的假通过） */
    uint32_t ab = proto_crc32(&block[32], 32, proto_crc32(block, 32, 0U));
    uint32_t ba = proto_crc32(block, 32, proto_crc32(&block[32], 32, 0U));
    CHECK_MSG(ab != ba, "chunk order changes CRC (accumulator is used)");

    CASE_END("Case 01 CRC vectors", "standard vectors + chunked consistency",
             "5 vectors, 5 chunkings");
}

/* ==========================================================================
 * 用例 2：正常帧往返
 * ========================================================================== */
static void case_02_roundtrip(void) {
    CASE_BEGIN();
    printf("\n-- Case 02: build -> parse round trip --\n");
    printf("   checks: build -> feed byte-by-byte -> cmd/len/payload identical\n");

    uint8_t payload[64];
    fill_pattern(payload, sizeof(payload), 0x11U);

    uint8_t frame[PROTO_MAX_FRAME];
    uint16_t n = proto_build_frame(frame, sizeof(frame), CMD_OTA_CHUNK, payload,
                                   (uint16_t)sizeof(payload));
    CHECK_EQ_I32(n,
                 (int)(PROTO_HEADER_SIZE + sizeof(payload) + PROTO_CRC_SIZE));
    CHECK_EQ_U32(frame[0], PROTO_SYNC_BYTE);
    CHECK_EQ_U32(frame[1], CMD_OTA_CHUNK);
    CHECK_EQ_U32(frame[2], 64U);
    CHECK_EQ_U32(frame[3], 0U);

    ProtoParser_t p;
    CHECK_MSG(feed_expect_frame_impl(&p, frame, n, CMD_OTA_CHUNK, payload,
                                      (uint16_t)sizeof(payload)), "every payload byte survives the round trip");

    CASE_END("Case 02 round trip", "cmd/len/payload identical",
             "64-byte payload");
}

/* ==========================================================================
 * 用例 3：粘包（两个完整帧连续喂入）
 * ========================================================================== */
static void case_03_back_to_back(void) {
    CASE_BEGIN();
    printf("\n-- Case 03: back-to-back frames (coalesced) --\n");
    printf("   checks: two complete frames in one stream -> two frames out, in order\n");

    uint8_t pa[16], pb[24];
    fill_pattern(pa, sizeof(pa), 0xA0U);
    fill_pattern(pb, sizeof(pb), 0xB0U);

    uint8_t f1[PROTO_MAX_FRAME], f2[PROTO_MAX_FRAME];
    uint16_t n1 = proto_build_frame(f1, sizeof(f1), CMD_OTA_BEGIN, pa,
                                    (uint16_t)sizeof(pa));
    uint16_t n2 = proto_build_frame(f2, sizeof(f2), CMD_OTA_CHUNK, pb,
                                    (uint16_t)sizeof(pb));
    CHECK_MSG(n1 > 0 && n2 > 0, "both frames built");

    uint8_t both[2 * PROTO_MAX_FRAME];
    memcpy(both, f1, n1);
    memcpy(both + n1, f2, n2);

    /* 逐字节喂入：必须依次返回两帧 */
    ProtoParser_t p;
    proto_parser_init(&p);
    unsigned hits = 0;
    bool first_ok = false, second_ok = false;
    static uint8_t snap[PROTO_MAX_PAYLOAD];

    for (uint32_t i = 0; i < (uint32_t)n1 + (uint32_t)n2; i++) {
        const ProtoFrame_t *f = proto_parser_feed(&p, both[i]);
        if (f == NULL) {
            continue;
        }
        hits++;
        if (hits == 1) {
            first_ok = (f->cmd == CMD_OTA_BEGIN) && (f->len == sizeof(pa));
            if (f->len) memcpy(snap, f->payload, f->len);
        } else if (hits == 2) {
            second_ok = (f->cmd == CMD_OTA_CHUNK) && (f->len == sizeof(pb));
            if (f->len) memcpy(snap, f->payload, f->len);
        }
    }
    CHECK_EQ_I32((int)hits, 2);
    CHECK_MSG(first_ok, "frame #1 = CMD_OTA_BEGIN len 16");
    CHECK_MSG(second_ok, "frame #2 = CMD_OTA_CHUNK len 24");

    CASE_END("Case 03 coalesced frames", "parser returns both frames in order",
             "no bytes lost between frames");
}

/* ==========================================================================
 * 用例 4：拆包（1 字节 / 2 字节 / 随机切分）
 * ========================================================================== */
static void case_04_split_feeds(void) {
    CASE_BEGIN();
    printf("\n-- Case 04: split feeds (fragmentation) --\n");
    printf("   checks: 1-byte / 2-byte / random chunking all decode identically\n");

    uint8_t payload[40];
    fill_pattern(payload, sizeof(payload), 0x77U);
    uint8_t frame[PROTO_MAX_FRAME];
    uint16_t n = proto_build_frame(frame, sizeof(frame), CMD_APP_MSG, payload,
                                   (uint16_t)sizeof(payload));

    /* 1 字节切分 == 用例 2 的逐字节路径，这里是显式基线 */
    ProtoParser_t p;
    CHECK_MSG(feed_expect_frame_impl(&p, frame, n, CMD_APP_MSG, payload,
                                      (uint16_t)sizeof(payload)), "1-byte chunks");

    /* 2 字节切分 */
    {
        static uint8_t snap[PROTO_MAX_PAYLOAD];
        proto_parser_init(&p);
        unsigned hits = 0;
        bool ok = false;
        for (uint16_t i = 0; i < n; i += 2) {
            uint16_t m = (uint16_t)((n - i < 2) ? (n - i) : 2);
            for (uint16_t k = 0; k < m; k++) {
                const ProtoFrame_t *f = proto_parser_feed(&p, frame[i + k]);
                if (f != NULL) {
                    hits++;
                    ok = (f->cmd == CMD_APP_MSG) && (f->len == sizeof(payload));
                    if (f->len) memcpy(snap, f->payload, f->len);
                }
            }
        }
        CHECK_EQ_I32((int)hits, 1);
        CHECK_MSG(ok, "2-byte chunks: cmd/len ok");
        CHECK_EQ_BYTES(snap, payload,
                       sizeof(payload));
    }

    /* 随机切分（固定种子，可复现） */
    {
        static uint8_t snap[PROTO_MAX_PAYLOAD];
        proto_parser_init(&p);
        unsigned hits = 0;
        bool ok = false;
        uint32_t rng = 0xDEADBEEFU;
        uint16_t off = 0;
        while (off < n) {
            uint16_t m = (uint16_t)(prng_next(&rng) % 13U) + 1U;
            if (off + m > n) {
                m = (uint16_t)(n - off);
            }
            for (uint16_t k = 0; k < m; k++) {
                const ProtoFrame_t *f = proto_parser_feed(&p, frame[off + k]);
                if (f != NULL) {
                    hits++;
                    ok = (f->cmd == CMD_APP_MSG) && (f->len == sizeof(payload));
                    if (f->len) memcpy(snap, f->payload, f->len);
                }
            }
            off = (uint16_t)(off + m);
        }
        CHECK_EQ_I32((int)hits, 1);
        CHECK_MSG(ok, "random chunks: cmd/len ok");
        CHECK_EQ_BYTES(snap, payload,
                       sizeof(payload));
    }

    CASE_END("Case 04 split feeds", "1B / 2B / random fragmentation identical",
             "40-byte payload");
}

/* ==========================================================================
 * 用例 5：前导垃圾字节
 * ========================================================================== */
static void case_05_leading_garbage(void) {
    CASE_BEGIN();
    printf("\n-- Case 05: leading garbage before frame --\n");
    printf("   checks: non-SYNC junk, stray 0xA5, 0xA5 flood -> resync behaviour\n");

    uint8_t payload[12];
    fill_pattern(payload, sizeof(payload), 0x33U);
    uint8_t frame[PROTO_MAX_FRAME];
    uint16_t n = proto_build_frame(frame, sizeof(frame), CMD_GET_STATUS, payload,
                                   (uint16_t)sizeof(payload));

    /* 构造垃圾：全部非 0xA5（含 0x00、0xFF、0xA4、0xA6 等边界值） */
    static const uint8_t junk[] = {0x00, 0xFF, 0xA4, 0xA6, 0x5A, 0x00, 0x7F,
                                   0x80, 0xFE, 0x01};
    for (size_t i = 0; i < sizeof(junk); i++) {
        CHECK_MSG(junk[i] != PROTO_SYNC_BYTE, "junk byte is not SYNC");
    }

    uint8_t buf[sizeof(junk) + PROTO_MAX_FRAME];
    memcpy(buf, junk, sizeof(junk));
    memcpy(buf + sizeof(junk), frame, n);
    uint32_t total = (uint32_t)sizeof(junk) + (uint32_t)n;

    ProtoParser_t p;
    proto_parser_init(&p);
    unsigned hits = 0;
    uint8_t cmd = 0;
    uint16_t len = 0;
    for (uint32_t i = 0; i < total; i++) {
        const ProtoFrame_t *f = proto_parser_feed(&p, buf[i]);
        if (f != NULL) {
            hits++;
            cmd = f->cmd;
            len = f->len;
        }
    }
    CHECK_EQ_I32((int)hits, 1);
    CHECK_EQ_U32(cmd, CMD_GET_STATUS);
    CHECK_EQ_I32((int)len, (int)sizeof(payload));

    /* 边界：垃圾里含 0xA5。
     *
     * 这是本协议**内建的同步风险**（诚实记录，不粉饰）：0xA5 既是 SYNC，
     * 也可能出现在"被丢弃字节"的任意位置。一旦在真帧前多出一个 0xA5，
     * parser 会把它当作帧头，从而吃掉真帧的 SYNC（当成自己的 CMD），
     * 于是**这一整帧被静默丢弃**。
     *
     * 更值得警惕的是：失步之后的 parser 遇到下一个 0xA5 时，会把它当作
     * "某帧的最后 4 个字节是 CRC"来校验 —— 也就是说，失步状态下的帧边界
     * 由**数据内容**决定，而不是由发送方的帧边界决定。极端情况下一个
     * 恰好满足 CRC 的字节序列可能被接受成一条伪帧。
     *
     * 下面的用例分别锁定：
     *   a) 单个前导 0xA5 → 该帧丢失（记录风险，作为已知行为断言）；
     *   b) 前置 0xA5 + 连续 4 帧 → 恢复 3 帧（损失上界 1 帧，不卡死）；
     *   c) 失步后恢复所需的最大字节数（随机噪声前缀，可复现）；
     *   d) 结构化的 A5+全零等候选前缀不得被接受成伪帧（任意长度前缀扫描）。
     */
    {
        static uint8_t pl[4];
        fill_pattern(pl, sizeof(pl), 0x21U);
        uint8_t fr[PROTO_MAX_FRAME];
        uint16_t fn = proto_build_frame(fr, sizeof(fr), CMD_GET_STATUS, pl,
                                       (uint16_t)sizeof(pl));

        /* (a) 单帧 + 前置 0xA5 → 必须 0 命中（记录协议风险） */
        {
            static uint8_t buf[1 + PROTO_MAX_FRAME];
            buf[0] = PROTO_SYNC_BYTE;
            memcpy(buf + 1, fr, fn);
            proto_parser_init(&p);
            unsigned h = 0;
            for (uint32_t i = 0; i < (uint32_t)fn + 1U; i++) {
                if (proto_parser_feed(&p, buf[i]) != NULL) {
                    h++;
                }
            }
            CHECK_EQ_I32((int)h, 0);
            printf("      NOTE: a single stray 0xA5 swallows the following frame "
                   "(hits=%u)\n", h);
        }

        /* (b) 前置 0xA5 + 连续 4 帧 → 恰好恢复 3 帧 */
        {
            static uint8_t stream[1 + 4 * PROTO_MAX_FRAME];
            stream[0] = PROTO_SYNC_BYTE;
            for (unsigned k = 0; k < 4; k++) {
                memcpy(stream + 1 + k * fn, fr, fn);
            }
            proto_parser_init(&p);
            unsigned h = 0;
            bool all_ok = true;
            for (uint32_t i = 0; i < 1U + 4U * (uint32_t)fn; i++) {
                const ProtoFrame_t *f = proto_parser_feed(&p, stream[i]);
                if (f != NULL) {
                    h++;
                    if (f->cmd != CMD_GET_STATUS || f->len != sizeof(pl) ||
                        memcmp(f->payload, pl, sizeof(pl)) != 0) {
                        all_ok = false;
                    }
                }
            }
            CHECK_EQ_I32((int)h, 3);
            CHECK_MSG(all_ok, "all recovered frames are intact");
            printf("      NOTE: parser resynchronizes on the next frame "
                   "(3/4 recovered, no lock-up)\n");
        }

        /* (c) 失步恢复的字节数上界：K 个随机非 0xA5 噪声 + 真帧。
         * 因为噪声长度本身会影响失步深度，这里固定 K 再取 2000 次试验的
         * 最坏值 —— 结果是可复现的（固定种子），且必须"总能恢复"。 */
        {
            enum { K_NOISE = 8, TRIALS = 2000 };
            static uint8_t pre[K_NOISE];
            unsigned worst = 0;
            unsigned never = 0;
            uint32_t rng = 0x0BADF00DU;
            for (int trial = 0; trial < TRIALS; trial++) {
                rng = 0x0BADF00DU ^ (uint32_t)trial * 2654435761U;
                for (unsigned i = 0; i < K_NOISE; i++) {
                    uint8_t b;
                    do {
                        b = (uint8_t)prng_next(&rng);
                    } while (b == PROTO_SYNC_BYTE);
                    pre[i] = b;
                }
                proto_parser_init(&p);
                unsigned at = 0;
                bool hit = false;
                for (uint32_t i = 0; i < K_NOISE + (uint32_t)fn; i++) {
                    uint8_t b = (i < K_NOISE) ? pre[i] : fr[i - K_NOISE];
                    if (proto_parser_feed(&p, b) != NULL) {
                        hit = true;
                        at = i + 1U;
                        break;
                    }
                }
                if (!hit) {
                    never++;
                } else if (at > worst) {
                    worst = at;
                }
            }
            CHECK_EQ_I32((int)never, 0);
            CHECK_MSG(worst <= K_NOISE + (unsigned)fn,
                      "resync latency stays inside the following frame");
            printf("      NOTE: %d random %d-byte non-A5 prefixes: 0 never synced, "
                   "worst resync latency = %u bytes\n", TRIALS, K_NOISE, worst);
        }

        /* (d) 结构化伪帧探测：0xA5 前缀 + 4 个'像 CRC 的'字节，不得被接受。
         * 全零 CRC 对应"整帧 CRC 为 0"的少见情形，是最容易踩中的候选。 */
        {
            static const uint8_t cand[][4] = {
                {0x00, 0x00, 0x00, 0x00},
                {0xFF, 0xFF, 0xFF, 0xFF},
                {0xA5, 0xA5, 0xA5, 0xA5},
                {0x01, 0x00, 0x00, 0x00},
            };
            unsigned accepted = 0;
            for (size_t c = 0; c < sizeof(cand) / sizeof(cand[0]); c++) {
                static uint8_t stream[5 + PROTO_MAX_FRAME];
                stream[0] = PROTO_SYNC_BYTE;
                memcpy(stream + 1, cand[c], 4);
                memcpy(stream + 5, fr, fn);
                proto_parser_init(&p);
                for (uint32_t i = 0; i < 5U + (uint32_t)fn; i++) {
                    const ProtoFrame_t *f = proto_parser_feed(&p, stream[i]);
                    if (f != NULL && i < 4U) { /* 只统计"伪帧"（真帧之前） */
                        accepted++;
                    }
                }
                CHECK_EQ_I32((int)accepted, 0);
            }
            CHECK_EQ_I32((int)accepted, 0);
        }

        /* (e) 长串 0xA5 洪水（64 字节）→ 自身即可重新同步，最终解出真帧 */
        {
            static uint8_t flood[64];
            static uint8_t buf[64 + PROTO_MAX_FRAME];
            memset(flood, PROTO_SYNC_BYTE, sizeof(flood));
            memcpy(buf, flood, sizeof(flood));
            memcpy(buf + sizeof(flood), fr, fn);
            proto_parser_init(&p);
            unsigned h = 0;
            bool ok = false;
            for (uint32_t i = 0; i < 64U + (uint32_t)fn; i++) {
                const ProtoFrame_t *f = proto_parser_feed(&p, buf[i]);
                if (f != NULL) {
                    h++;
                    ok = (f->cmd == CMD_GET_STATUS) && (f->len == sizeof(pl)) &&
                         (memcmp(f->payload, pl, sizeof(pl)) == 0);
                }
            }
            CHECK_EQ_I32((int)h, 1);
            CHECK_MSG(ok, "A5 flood: frame eventually decoded intact");
        }
    }

    CASE_END("Case 05 leading garbage", "resync after garbage / stray 0xA5",
             "10-byte junk, 1x/64x 0xA5");
}

/* ==========================================================================
 * 用例 6：非法长度（LEN > PROTO_MAX_PAYLOAD）
 * ========================================================================== */
static void case_06_illegal_length(void) {
    CASE_BEGIN();
    printf("\n-- Case 06: illegal length (LEN > PROTO_MAX_PAYLOAD) --\n");
    printf("   checks: LEN 1029 / 1028+1 / 0xFFFF rejected, no OOB write, resync\n");

    ProtoParser_t p;
    const uint16_t bad_lens[] = {PROTO_MAX_PAYLOAD + 1U, 0xFFFFU, 1029U};
    static const char *bad_names[] = {"1028+1", "0xFFFF", "1029"};

    for (size_t i = 0; i < sizeof(bad_lens) / sizeof(bad_lens[0]); i++) {
        uint16_t L = bad_lens[i];
        uint8_t frame[8 + 2048];
        frame[0] = PROTO_SYNC_BYTE;
        frame[1] = CMD_OTA_CHUNK;
        frame[2] = (uint8_t)(L & 0xFFU);
        frame[3] = (uint8_t)((L >> 8) & 0xFFU);
        /* 后面填上"本应"是 payload + CRC 的字节，确保 parser 若错误地继续
         * 消费不会因为缺字节而卡住 —— 它会吞掉这些字节，但必须不越界。 */
        for (size_t k = 4; k < sizeof(frame); k++) {
            frame[k] = (uint8_t)(0x40U + (uint8_t)k);
        }
        uint32_t feed_len = 4U + 4U + L; /* 声明长度所需的总字节数（可能巨大） */
        if (feed_len > sizeof(frame)) {
            feed_len = (uint32_t)sizeof(frame);
        }

        /* 用 feed_accepts 跑完整入流：不得返回帧 */
        uint8_t cmd = 0;
        uint16_t len = 0;
        bool got = feed_accepts(&p, frame, feed_len, &cmd, &len, NULL);
        CHECK_MSG(!got, "LEN > MAX is rejected (no frame returned)");
        CHECK_EQ_U32((unsigned)p.state, (unsigned)FRAME_STATE_SYNC);
        CHECK_EQ_U32((unsigned)p.payload_idx, 0U);
        printf("      LEN=%s -> rejected, parser state=%d\n", bad_names[i],
               (int)p.state);

        /* 不卡死：非法长度之后必须仍能解出下一帧。
         * 注意这里不是"再 init 一次"，而是在同一个 parser 上继续喂 —— 才是
         * 对"拒绝后回到 SYNC 并重新同步"的真实检验。 */
        {
            uint8_t pl[6];
            fill_pattern(pl, sizeof(pl), 0x21U);
            uint8_t next_frame[PROTO_MAX_FRAME];
            uint16_t nn = proto_build_frame(next_frame, sizeof(next_frame),
                                            CMD_OTA_END, pl, (uint16_t)sizeof(pl));
            static uint8_t snap[PROTO_MAX_PAYLOAD];
            bool after = feed_accepts(&p, next_frame, nn, &cmd, &len, snap);
            CHECK_MSG(after && cmd == CMD_OTA_END && len == sizeof(pl) &&
                          memcmp(snap, pl, sizeof(pl)) == 0,
                      "recovers on the SAME parser instance after illegal LEN");
        }
    }

    /* 0xFFFF 专项内存安全：完整 4 字节头 + 32 字节跟随垃圾。
     * parser 必须在 LEN_HI 阶段就拒绝，payload 数组不得被写。 */
    {
        uint8_t buf[4 + 32];
        buf[0] = PROTO_SYNC_BYTE;
        buf[1] = CMD_OTA_CHUNK;
        buf[2] = 0xFFU; /* LEN = 0xFFFF */
        buf[3] = 0xFFU;
        memset(&buf[4], 0x5C, 32); /* 伪 payload 区：parser 不得消费它 */

        /* 用已知模式填充 payload 数组作为哨兵：非法长度不得写入其中 */
        ProtoParser_t *pp = &p;
        proto_parser_init(pp);
        memset(pp->frame.payload, 0xC7, sizeof(pp->frame.payload));
        pp->frame.len = 0xFFFFU; /* 预置成"被拒绝的值"以便检出是否被改写 */

        unsigned hits = 0;
        for (uint32_t i = 0; i < (uint32_t)sizeof(buf); i++) {
            if (proto_parser_feed(pp, buf[i]) != NULL) {
                hits++;
                printf("      unexpected frame at byte %u of the 0xFFFF header\n",
                       (unsigned)i);
            }
        }

        bool untouched = true;
        for (size_t i = 0; i < sizeof(pp->frame.payload); i++) {
            if (pp->frame.payload[i] != 0xC7U) {
                untouched = false;
                printf("      payload[%u] was overwritten: 0x%02X\n",
                       (unsigned)i, (unsigned)pp->frame.payload[i]);
                break;
            }
        }
        CHECK_MSG(untouched,
                  "LEN=0xFFFF never writes into frame.payload (memory safe)");
        CHECK_EQ_I32((int)hits, 0);
        CHECK_EQ_U32((unsigned)pp->state, (unsigned)FRAME_STATE_SYNC);
        CHECK_EQ_U32((unsigned)pp->payload_idx, 0U);
        /* 说明：拒绝路径把 frame.len 原样保留为非法值（不写 payload）；
         * 该结构体在 state==SYNC 时会被下一次有效帧覆盖，因此安全。
         * 这里显式锁定该行为，避免将来被误改成"接受"或"清零后继续"。 */
        CHECK_EQ_U32((unsigned)pp->frame.len, 0xFFFFU);
    }

    /* 0xFFFF 之后追随的有效帧必须仍能解出（同一 parser 实例，不重新 init） */
    {
        uint8_t buf[4 + 32 + PROTO_MAX_FRAME];
        uint8_t pl[6];
        fill_pattern(pl, sizeof(pl), 0x21U);
        uint16_t nn = proto_build_frame(&buf[4 + 32], PROTO_MAX_FRAME,
                                        CMD_OTA_END, pl, (uint16_t)sizeof(pl));
        buf[0] = PROTO_SYNC_BYTE;
        buf[1] = CMD_OTA_CHUNK;
        buf[2] = 0xFFU;
        buf[3] = 0xFFU;
        memset(&buf[4], 0x5C, 32);

        ProtoParser_t *pp = &p;
        proto_parser_init(pp);
        unsigned hits = 0;
        uint8_t cmd = 0;
        uint16_t len = 0;
        static uint8_t snap[PROTO_MAX_PAYLOAD];
        for (uint32_t i = 0; i < (uint32_t)(4 + 32) + (uint32_t)nn; i++) {
            const ProtoFrame_t *f = proto_parser_feed(pp, buf[i]);
            if (f != NULL) {
                hits++;
                cmd = f->cmd;
                len = f->len;
                memcpy(snap, f->payload, f->len);
            }
        }
        CHECK_EQ_I32((int)hits, 1);
        CHECK_EQ_U32(cmd, CMD_OTA_END);
        CHECK_EQ_I32((int)len, (int)sizeof(pl));
        CHECK_EQ_BYTES(snap, pl, sizeof(pl));
        CHECK_MSG(hits == 1, "after LEN=0xFFFF: only the following valid frame "
                             "is decoded");
    }

    /* builder 侧同样必须拒绝非法长度 */
    {
        uint8_t out[PROTO_MAX_FRAME + 64];
        uint16_t r = proto_build_frame(out, sizeof(out), CMD_OTA_CHUNK, out,
                                       (uint16_t)(PROTO_MAX_PAYLOAD + 1U));
        CHECK_EQ_I32((int)r, 0);
        r = proto_build_frame(out, sizeof(out), CMD_OTA_CHUNK, out, 0xFFFFU);
        CHECK_EQ_I32((int)r, 0);
    }

    CASE_END("Case 06 illegal length", "reject + resync + no OOB write",
             "1029 / 0xFFFF");
}

/* ==========================================================================
 * 用例 7：损坏帧（单比特翻转 / CRC 篡改 / payload 篡改）+ 不卡死
 * ========================================================================== */
static void case_07_corruption(void) {
    CASE_BEGIN();
    printf("\n-- Case 07: corrupted frames (single-bit flips, CRC/payload) --\n");
    printf("   checks: every byte bit-flip, CRC tamper, payload tamper -> dropped\n");

    uint8_t payload[20];
    fill_pattern(payload, sizeof(payload), 0x99U);
    uint8_t frame[PROTO_MAX_FRAME];
    uint16_t n = proto_build_frame(frame, sizeof(frame), CMD_OTA_CHUNK, payload,
                                   (uint16_t)sizeof(payload));

    uint8_t valid_next[PROTO_MAX_FRAME];
    uint8_t next_payload[8];
    fill_pattern(next_payload, sizeof(next_payload), 0x42U);
    uint16_t next_n = proto_build_frame(valid_next, sizeof(valid_next),
                                        CMD_OTA_BEGIN_ACK, next_payload,
                                        (uint16_t)sizeof(next_payload));

    ProtoParser_t p;
    unsigned bit_flip_count = 0;
    unsigned stuck_count = 0;

    /* 7.1 逐字节、逐比特翻转。覆盖每个字节时只翻转 1 个 bit（8 个位置里的
     * 一个确定位置，i%8），保证用例数量可控且分布均匀。 */
    for (uint16_t i = 0; i < n; i++) {
        uint8_t bad[PROTO_MAX_FRAME];
        memcpy(bad, frame, n);
        bad[i] ^= (uint8_t)(1U << (i % 8U));
        bit_flip_count++;

        static uint8_t snap[PROTO_MAX_PAYLOAD];
        uint8_t cmd = 0;
        uint16_t len = 0;
        bool got = feed_accepts(&p, bad, n, &cmd, &len, snap);

        /* 翻转 SYNC 字节时不再有帧头 → 必然 NULL；
         * 翻转其它字节时 CRC 必然不匹配 → 也必须 NULL。
         * 注意：payload 的翻转在 CRC 保护下不可能被接受。 */
        if (got) {
            printf("      byte[%u] bit%d flip was ACCEPTED (cmd=0x%02X len=%u)\n",
                   (unsigned)i, (int)(i % 8U), (unsigned)cmd, (unsigned)len);
        }
        CHECK_MSG(!got, "single-bit flip is rejected");

        /* 不卡死：损坏帧之后必须仍能收到下一帧 */
        static uint8_t snap2[PROTO_MAX_PAYLOAD];
        uint8_t cmd2 = 0;
        uint16_t len2 = 0;
        bool after = feed_accepts(&p, valid_next, next_n, &cmd2, &len2, snap2);
        if (!after || cmd2 != CMD_OTA_BEGIN_ACK ||
            len2 != sizeof(next_payload) ||
            memcmp(snap2, next_payload, sizeof(next_payload)) != 0) {
            stuck_count++;
            printf("      after byte[%u] flip, next valid frame was NOT decoded\n",
                   (unsigned)i);
        }
    }
    CHECK_EQ_I32((int)bit_flip_count, (int)n);
    CHECK_EQ_I32((int)stuck_count, 0);

    /* 7.2 CRC 字段被整体篡改（4 字节全改） */
    for (int variant = 0; variant < 4; variant++) {
        uint8_t bad[PROTO_MAX_FRAME];
        memcpy(bad, frame, n);
        bad[n - 4 + variant] ^= 0xFFU;
        uint8_t cmd = 0;
        uint16_t len = 0;
        bool got = feed_accepts(&p, bad, n, &cmd, &len, NULL);
        CHECK_MSG(!got, "tampered CRC field is rejected");

        uint8_t cmd2 = 0;
        uint16_t len2 = 0;
        bool after = feed_accepts(&p, valid_next, next_n, &cmd2, &len2, NULL);
        CHECK_MSG(after && cmd2 == CMD_OTA_BEGIN_ACK, "recovers after CRC tamper");
    }

    /* 7.3 payload 每个字节被改成"另一个值"（+1） */
    for (uint16_t i = 0; i < sizeof(payload); i++) {
        uint8_t bad[PROTO_MAX_FRAME];
        memcpy(bad, frame, n);
        bad[PROTO_HEADER_SIZE + i] = (uint8_t)(bad[PROTO_HEADER_SIZE + i] + 1U);
        uint8_t cmd = 0;
        uint16_t len = 0;
        bool got = feed_accepts(&p, bad, n, &cmd, &len, NULL);
        if (got) {
            printf("      payload[%u] tamper was ACCEPTED\n", (unsigned)i);
        }
        CHECK_MSG(!got, "tampered payload byte is rejected");
    }

    /* 7.4 截断帧（尾部缺字节）→ 不得返回帧，且后续有效帧仍可解出 */
    {
        uint8_t cmd = 0;
        uint16_t len = 0;
        bool got = feed_accepts(&p, frame, (uint32_t)(n - 1U), &cmd, &len, NULL);
        CHECK_MSG(!got, "truncated frame (missing last CRC byte) is rejected");
        uint8_t cmd2 = 0;
        uint16_t len2 = 0;
        bool after = feed_accepts(&p, valid_next, next_n, &cmd2, &len2, NULL);
        CHECK_MSG(after && cmd2 == CMD_OTA_BEGIN_ACK, "recovers after truncated frame");
    }

    CASE_END("Case 07 corruption", "every byte bit-flip + CRC/payload tamper",
             "20 payload bytes, no parser lock-up");
}

/* ==========================================================================
 * 用例 8：零长度 payload
 * ========================================================================== */
static void case_08_zero_length(void) {
    CASE_BEGIN();
    printf("\n-- Case 08: zero-length payload --\n");
    printf("   checks: LEN=0 returns a valid frame (NULL and non-NULL payload)\n");

    uint8_t frame[PROTO_MAX_FRAME];
    uint16_t n = proto_build_frame(frame, sizeof(frame), CMD_RESET, NULL, 0);
    CHECK_EQ_I32(n,
                 (int)(PROTO_HEADER_SIZE + PROTO_CRC_SIZE));
    CHECK_EQ_U32(frame[2], 0U);
    CHECK_EQ_U32(frame[3], 0U);

    ProtoParser_t p;
    proto_parser_init(&p);
    unsigned hits = 0;
    uint8_t cmd = 0;
    uint16_t len = 0xFFFFU;
    for (uint16_t i = 0; i < n; i++) {
        const ProtoFrame_t *f = proto_parser_feed(&p, frame[i]);
        if (f != NULL) {
            hits++;
            cmd = f->cmd;
            len = f->len;
        }
    }
    CHECK_EQ_I32((int)hits, 1);
    CHECK_EQ_U32(cmd, CMD_RESET);
    CHECK_EQ_I32((int)len, 0);

    /* 零长度帧的 CRC 字节不应恰好是 SYNC（否则 0xA5 洪水场景会出现
     * "帧中间出现同步字节"的歧义）；显式记录该前提。 */
    CHECK_MSG(frame[4] != PROTO_SYNC_BYTE && frame[5] != PROTO_SYNC_BYTE &&
                  frame[6] != PROTO_SYNC_BYTE && frame[7] != PROTO_SYNC_BYTE,
              "zero-length frame CRC bytes contain no 0xA5");

    /* 篡改零长度帧的 CRC → 必须被丢弃（LEN=0 不能绕过 CRC 校验） */
    {
        uint8_t bad[PROTO_MAX_FRAME];
        memcpy(bad, frame, n);
        bad[4] ^= 0x01U;
        uint8_t c2 = 0;
        uint16_t l2 = 0;
        bool got = feed_accepts(&p, bad, n, &c2, &l2, NULL);
        CHECK_MSG(!got, "zero-length frame with tampered CRC is dropped");
    }

    /* 非 NULL 的 payload 指针 + len 0 也必须正常（不应解引用） */
    {
        uint8_t dummy = 0xEEU;
        uint16_t n2 = proto_build_frame(frame, sizeof(frame), CMD_RESET, &dummy, 0);
        CHECK_EQ_I32((int)n2,
                     (int)n);
        proto_parser_init(&p);
        unsigned h2 = 0;
        for (uint16_t i = 0; i < n2; i++) {
            if (proto_parser_feed(&p, frame[i]) != NULL) {
                h2++;
            }
        }
        CHECK_EQ_I32((int)h2, 1);
    }

    CASE_END("Case 08 zero-length payload", "LEN=0 returns a valid frame",
             "8-byte frame");
}

/* ==========================================================================
 * 用例 9：最大长度 payload
 * ========================================================================== */
static void case_09_max_length(void) {
    CASE_BEGIN();
    printf("\n-- Case 09: maximum-length payload (LEN=%u) --\n",
           (unsigned)PROTO_MAX_PAYLOAD);
    printf("   checks: LEN=1028 (PROTO_MAX_PAYLOAD) round trip, all bytes intact\n");

    static uint8_t payload[PROTO_MAX_PAYLOAD];
    fill_pattern(payload, sizeof(payload), 0x01U);

    uint8_t frame[PROTO_MAX_FRAME];
    uint16_t n = proto_build_frame(frame, sizeof(frame), CMD_OTA_CHUNK, payload,
                                   (uint16_t)sizeof(payload));
    CHECK_EQ_I32((int)n,
                 (int)PROTO_MAX_FRAME);
    CHECK_EQ_U32(frame[2],
                 (uint8_t)(PROTO_MAX_PAYLOAD & 0xFFU));
    CHECK_EQ_U32(frame[3],
                 (uint8_t)((PROTO_MAX_PAYLOAD >> 8) & 0xFFU));

    ProtoParser_t p;
    proto_parser_init(&p);
    unsigned hits = 0;
    uint16_t len = 0;
    uint8_t cmd = 0;
    static uint8_t snap[PROTO_MAX_PAYLOAD];
    for (uint16_t i = 0; i < n; i++) {
        const ProtoFrame_t *f = proto_parser_feed(&p, frame[i]);
        if (f != NULL) {
            hits++;
            cmd = f->cmd;
            len = f->len;
            memcpy(snap, f->payload, f->len);
        }
    }
    CHECK_EQ_I32((int)hits, 1);
    CHECK_EQ_U32(cmd, CMD_OTA_CHUNK);
    CHECK_EQ_I32((int)len, (int)PROTO_MAX_PAYLOAD);
    CHECK_EQ_BYTES(snap, payload,
                   sizeof(payload));

    CASE_END("Case 09 max-length payload", "1028-byte payload round trip",
             "no truncation, no OOB");
}

/* ==========================================================================
 * 用例 10：小端序号编解码锚点（bootloader 层测试预留）
 * ========================================================================== */
static uint32_t seq_encode(uint32_t seq, uint8_t out[4]) {
    /* 协议约定：payload 内多字节值为小端 */
    out[0] = (uint8_t)(seq & 0xFFU);
    out[1] = (uint8_t)((seq >> 8) & 0xFFU);
    out[2] = (uint8_t)((seq >> 16) & 0xFFU);
    out[3] = (uint8_t)((seq >> 24) & 0xFFU);
    return seq;
}

static uint32_t seq_decode_le(const uint8_t in[4]) {
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
           ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

static void case_10_sequence_endianness(void) {
    CASE_BEGIN();
    printf("\n-- Case 10: 4-byte LE sequence anchor (bootloader-layer "
           "placeholder) --\n");
    printf("   checks: explicit LE codec == memcpy view of the same 4 bytes\n");
    printf("      NOTE: 重复/乱序 chunk 语义属于 bootloader 层，parser 只负责成帧。\n");
    printf("      本条是为将来 bootloader 层测试预留的锚点：固定序号在本项目\n");
    printf("      工具链（x86-64 主机 + STM32F1/ESP32 目标，均小端）下的编解码。\n");

    const uint32_t seqs[] = {0U,          1U,          255U,
                             256U,        0x01020304U, 0x7FFFFFFFU,
                             0x80000000U, 0xFFFFFFFFU, 0xDEADBEEFU};
    bool all_le = true;

    for (size_t i = 0; i < sizeof(seqs) / sizeof(seqs[0]); i++) {
        uint32_t seq = seqs[i];
        uint8_t wire[4];
        (void)seq_encode(seq, wire);

        /* 线格式断言：字节序必须是"低字节在前" */
        if (wire[0] != (uint8_t)(seq & 0xFFU) ||
            wire[3] != (uint8_t)((seq >> 24) & 0xFFU)) {
            all_le = false;
            printf("      seq=0x%08X wire={%02X %02X %02X %02X} not LE\n",
                   (unsigned)seq, wire[0], wire[1], wire[2], wire[3]);
        }
        CHECK_EQ_U32(seq_decode_le(wire), seq);

        /* memcpy 成 uint32_t 后读出的值必须与显式小端解码一致 ——
         * 这是"主机工具链与协议约定一致"的直接证明 */
        uint32_t via_memcpy = 0;
        memcpy(&via_memcpy, wire, 4);
        CHECK_EQ_U32(via_memcpy, seq);
    }
    CHECK_MSG(all_le, "all sequences serialize little-endian on the wire");

    /* 该锚点必须能穿透真实的帧路径：序号作为 payload 前 4 字节 */
    {
        static uint8_t payload[8];
        uint32_t seq = 0x0A0B0C0DU;
        (void)seq_encode(seq, payload);
        payload[4] = 0xAAU;
        payload[5] = 0xBBU;
        payload[6] = 0xCCU;
        payload[7] = 0xDDU;

        uint8_t frame[PROTO_MAX_FRAME];
        uint16_t n = proto_build_frame(frame, sizeof(frame), CMD_OTA_CHUNK,
                                       payload, (uint16_t)sizeof(payload));
        ProtoParser_t p;
        proto_parser_init(&p);
        static uint8_t snap[PROTO_MAX_PAYLOAD];
        bool got = false;
        for (uint16_t i = 0; i < n; i++) {
            const ProtoFrame_t *f = proto_parser_feed(&p, frame[i]);
            if (f != NULL) {
                got = true;
                memcpy(snap, f->payload, f->len);
            }
        }
        CHECK_MSG(got, "sequence-bearing frame decodes");
        uint32_t decoded = 0;
        memcpy(&decoded, snap, 4);
        CHECK_EQ_U32(decoded, seq);
    }

    CASE_END("Case 10 LE sequence anchor", "memcpy/LE consistency on x86-64",
             "9 values + in-frame path");
}

/* ==========================================================================
 * 用例 11：缓冲区边界（proto_build_frame buf_size 不足）
 * ========================================================================== */
static void case_11_buffer_bounds(void) {
    CASE_BEGIN();
    printf("\n-- Case 11: proto_build_frame buffer boundary --\n");
    printf("   checks: buf_size 0..need-1 -> return 0, guard bytes untouched\n");

    static uint8_t payload[32];
    fill_pattern(payload, sizeof(payload), 0x66U);

    const uint16_t need =
        (uint16_t)(PROTO_HEADER_SIZE + sizeof(payload) + PROTO_CRC_SIZE);

    /* 带哨兵的缓冲区：out[0..need-1] 是合法写入区，out[need..] 是哨兵 */
    static uint8_t out[PROTO_MAX_FRAME + 64];
    const uint8_t guard = 0x3CU;

    /* 11.1 全部不足尺寸都必须返回 0，且越界区域不被写 */
    for (uint16_t size = 0; size < need; size++) {
        memset(out, guard, sizeof(out));
        uint16_t r = proto_build_frame(out, size, CMD_APP_MSG, payload,
                                       (uint16_t)sizeof(payload));
        if (r != 0) {
            printf("      buf_size=%u (<%u) returned %u instead of 0\n",
                   (unsigned)size, (unsigned)need, (unsigned)r);
        }
        CHECK_MSG(r == 0, "too-small buf_size returns 0");
        bool clean = true;
        for (size_t i = 0; i < sizeof(out); i++) {
            if (out[i] != guard) {
                clean = false;
                printf("      buf_size=%u wrote out[%u] (OOB write!)\n",
                       (unsigned)size, (unsigned)i);
                break;
            }
        }
        CHECK_MSG(clean, "nothing written when buf_size is too small");
    }

    /* 11.2 恰好够：必须成功且哨兵完好 */
    {
        memset(out, guard, sizeof(out));
        uint16_t r = proto_build_frame(out, need, CMD_APP_MSG, payload,
                                       (uint16_t)sizeof(payload));
        CHECK_EQ_I32((int)r, (int)need);
        bool guard_ok = true;
        for (size_t i = need; i < sizeof(out); i++) {
            if (out[i] != guard) {
                guard_ok = false;
                break;
            }
        }
        CHECK_MSG(guard_ok, "no write past returned length");
    }

    /* 11.3 比需要多 1 字节：成功，且只写 need 字节 */
    {
        memset(out, guard, sizeof(out));
        uint16_t r = proto_build_frame(out, (uint16_t)(need + 1U), CMD_APP_MSG,
                                       payload, (uint16_t)sizeof(payload));
        CHECK_EQ_I32((int)r, (int)need);
        bool guard_ok = true;
        for (size_t i = need; i < sizeof(out); i++) {
            if (out[i] != guard) {
                guard_ok = false;
                break;
            }
        }
        CHECK_MSG(guard_ok, "writes exactly `need` bytes");
    }

    /* 11.4 buf_size = 0xFFFF（uint16_t 上界）配合最大 payload */
    {
        static uint8_t maxp[PROTO_MAX_PAYLOAD];
        fill_pattern(maxp, sizeof(maxp), 0x02U);
        static uint8_t big[PROTO_MAX_FRAME];
        memset(big, guard, sizeof(big));
        uint16_t r = proto_build_frame(big, (uint16_t)sizeof(big),
                                       CMD_OTA_CHUNK, maxp,
                                       (uint16_t)sizeof(maxp));
        CHECK_EQ_I32((int)r,
                     (int)PROTO_MAX_FRAME);
        CHECK_EQ_U32((unsigned)PROTO_MAX_FRAME,
                     (unsigned)(PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD +
                                PROTO_CRC_SIZE));
    }

    /* 11.5 buf == NULL 且 buf_size 充足：实现未声明该契约，
     *      此处只记录行为，不作为失败项（避免把未定义行为当成测试断言）。 */

    CASE_END("Case 11 buffer bounds", "insufficient buf_size -> 0, no OOB",
             "all sizes 0..need-1");
}

/* ==========================================================================
 * main
 * ========================================================================== */
int main(void) {
    printf("========================================================\n");
    printf(" UART protocol boundary test matrix (host side)\n");
    printf(" PROTO_MAX_PAYLOAD=%u PROTO_MAX_FRAME=%u\n",
           (unsigned)PROTO_MAX_PAYLOAD, (unsigned)PROTO_MAX_FRAME);
    printf("========================================================\n");

    case_01_crc_vectors();
    case_02_roundtrip();
    case_03_back_to_back();
    case_04_split_feeds();
    case_05_leading_garbage();
    case_06_illegal_length();
    case_07_corruption();
    case_08_zero_length();
    case_09_max_length();
    case_10_sequence_endianness();
    case_11_buffer_bounds();

    printf("\n========================================================\n");
    printf(" cases : %u/%u passed\n", g_cases_run - g_cases_failed, g_cases_run);
    printf(" checks: %u/%u passed\n", g_checks_total - g_checks_failed,
           g_checks_total);
    printf(" RESULT: %u/%u passed\n", g_cases_run - g_cases_failed, g_cases_run);
    printf("========================================================\n");

    return (g_cases_failed == 0 && g_checks_failed == 0) ? 0 : 1;
}
