/**
 * @file    protocol.h
 * @brief   Shared UART protocol definitions for STM32 ↔ ESP32 communication.
 *
 * Used identically by: bootloader, application, ESP32, and PC test tools.
 *
 * Frame format (big-endian on the wire):
 *   [SYNC 0xA5] [CMD 1B] [LEN_L 1B] [LEN_H 1B] [PAYLOAD LEN bytes] [CRC32 4B LE]
 *
 * Max frame: 4 + 1024 + 4 = 1032 bytes.
 */

#ifndef SHARED_PROTOCOL_H
#define SHARED_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * Constants
 *---------------------------------------------------------------------------*/

#define PROTO_SYNC_BYTE         0xA5U
#define PROTO_MAX_PAYLOAD       1028U /* OTA chunk: sequence (4) + page data (1024) */
#define PROTO_HEADER_SIZE       4U    /* SYNC + CMD + LEN(2) */
#define PROTO_CRC_SIZE          4U    /* CRC32 LE */
#define PROTO_MAX_FRAME         (PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD + PROTO_CRC_SIZE)

/* Flash layout (shared so bootloader and app agree) */
/* #ifndef guards: STM32 HAL already defines FLASH_BASE/FLASH_PAGE_SIZE */
#ifndef FLASH_BASE
#define FLASH_BASE              0x08000000U
#endif
#ifndef FLASH_SIZE
#define FLASH_SIZE              0x00010000U   /* 64KB */
#endif
#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE         1024U         /* 1KB per page on F103C8 */
#endif

#define BOOTLOADER_BASE         0x08000000U
#define BOOTLOADER_SIZE         0x00002000U   /* 8KB, pages 0-7 */
#define BOOTLOADER_PAGES        8U

#define APP_BASE                0x08002000U
#define APP_SIZE                0x0000D800U   /* 54KB, pages 8-61 */
#define APP_PAGES               54U

#define CONFIG_BASE             0x0800F800U   /* page 62 */
#define CONFIG_SIZE             0x00000800U   /* 2KB, pages 62-63 */
#define CONFIG_PAGE             62U

/* The config region holds two identical copies, one per page, written
 * alternately (ping-pong). This is what makes a config update atomic: a single
 * flash page can only be erased as a whole, so two copies inside one page would
 * both vanish during the erase. With one copy per page, the page that is not
 * being written always still holds the previous valid config, and
 * BootConfig_t.config_seq decides which copy is newer. */
#define CONFIG_PAGE_A           62U
#define CONFIG_PAGE_B           63U
#define CONFIG_RECORD_SIZE      64U           /* padded record per page */

/* OTA window: bootloader waits this long (ms) for OTA_BEGIN after reset */
#define OTA_WINDOW_MS           200U

/* Window used when the bootloader has decided the host must re-send firmware
 * (partial/invalid image, or an application that never confirmed its boot). */
#define OTA_RECOVERY_WINDOW_MS  10000U

/* Consecutive unconfirmed boots before the bootloader declares the image bad
 * and opens the long recovery window. */
#define BOOT_ATTEMPT_LIMIT      3U

/* Per-chunk ACK timeout (ms) — ESP32 side.
 * Sized for the original 9600-baud bring-up link (~1.1s per 1 KiB chunk);
 * generous headroom at the current 115200 baud, safe to tighten later. */
#define OTA_CHUNK_TIMEOUT_MS    2000U
/* Max retries per chunk */
#define OTA_MAX_RETRIES         3U

/*---------------------------------------------------------------------------
 * Command IDs
 *---------------------------------------------------------------------------*/

/* Master → Slave (ESP32 → STM32) */
#define CMD_OTA_BEGIN           0x10U
#define CMD_OTA_CHUNK           0x11U
#define CMD_OTA_END             0x12U
#define CMD_OTA_ABORT           0x13U
#define CMD_OTA_AVAILABLE       0x14U  /* "update ready, please reboot" */
#define CMD_APP_MSG             0x20U  /* passthrough data to application */
#define CMD_GET_STATUS          0x30U
#define CMD_RESET               0x31U
#define CMD_GET_SENSOR_SNAPSHOT 0x32U  /* request latest app sensor state */
#define CMD_DIAG_SNAPSHOT       0x33U  /* request cached runtime diagnostics */
#define CMD_RECOVERY_CHECK      0x34U  /* "who is running, and do you need firmware?" */
#define CMD_OTA_COMMITTED       0x35U  /* app: "version N booted and confirmed" */

/* Slave → Master (STM32 → ESP32) */
#define CMD_OTA_BEGIN_ACK       0x81U
#define CMD_CHUNK_ACK           0x82U
#define CMD_NAK                 0x83U
#define CMD_OTA_RESULT          0x84U
#define CMD_STATUS_RSP          0x85U
#define CMD_OTA_READY           0x86U  /* app saved OTA request and will reboot */
#define CMD_SENSOR_SNAPSHOT_RSP 0x87U
#define CMD_DIAG_SNAPSHOT_RSP   0x88U
#define CMD_RECOVERY_RSP        0x89U  /* RECOVERY_STATUS_* payload (4 bytes LE) */

/* DiagSnapshot_t.stack_high_water[] index order (fixed across all builds) */
#define DIAG_TASK_COUNT         4U
#define DIAG_TASK_IDX_COMM      0U
#define DIAG_TASK_IDX_CONTROL   1U
#define DIAG_TASK_IDX_APP       2U
#define DIAG_TASK_IDX_MONITOR   3U

/* SensorSnapshot_t.flags */
#define SENSOR_FLAG_ENV_VALID     (1U << 0)
#define SENSOR_FLAG_LIGHT_VALID   (1U << 1)
#define SENSOR_FLAG_PIR_READY     (1U << 2)
#define SENSOR_FLAG_PIR_WARMED_UP (1U << 3)
#define SENSOR_FLAG_MOTION        (1U << 4)
#define SENSOR_FLAG_RELAY1_ON     (1U << 5)
#define SENSOR_FLAG_RELAY2_ON     (1U << 6)
#define SENSOR_FLAG_AUTO_MODE     (1U << 7)
#define SENSOR_FLAG_BUZZER_ON     (1U << 8)
#define SENSOR_FLAG_UI_CHINESE    (1U << 9)

/*---------------------------------------------------------------------------
 * Error codes (in NAK / RSP_ERROR payloads)
 *---------------------------------------------------------------------------*/

#define ERR_NONE                0x00000000U
#define ERR_FRAME_CRC           0x00000001U
#define ERR_SEQ_MISMATCH        0x00000002U
#define ERR_FLASH_ERASE         0x00000003U
#define ERR_FLASH_PROGRAM       0x00000004U
#define ERR_FLASH_VERIFY        0x00000005U
#define ERR_IMAGE_CRC           0x00000006U
#define ERR_SIZE_TOO_LARGE      0x00000007U
#define ERR_UNKNOWN_CMD         0x00000008U
#define ERR_TIMEOUT             0x00000009U
#define ERR_BUSY                0x0000000AU

/*---------------------------------------------------------------------------
 * OTA result codes (in OTA_RESULT payload)
 *---------------------------------------------------------------------------*/

#define OTA_RESULT_OK           0x00000000U
#define OTA_RESULT_FAIL         0x00000001U

/*---------------------------------------------------------------------------
 * Boot modes
 *---------------------------------------------------------------------------*/

#define BOOT_MODE_APP           0x00000000U
#define BOOT_MODE_OTA           0x00000001U

/*---------------------------------------------------------------------------
 * Update status
 *---------------------------------------------------------------------------*/

#define UPDATE_STATUS_NONE      0x00000000U
#define UPDATE_STATUS_OK        0x00000001U
#define UPDATE_STATUS_FAILED    0x00000002U
#define UPDATE_STATUS_IN_PROGRESS 0x00000003U

/*---------------------------------------------------------------------------
 * Recovery status — the bootloader's own reason for opening the OTA window.
 *
 * Reported in CMD_STATUS_RSP so the host can tell "all good, this was just the
 * passive boot window" apart from "this device needs firmware re-sent".
 * ---------------------------------------------------------------------------*/

#define RECOVERY_STATUS_NORMAL           0x00000000U /* app valid + boot confirmed */
#define RECOVERY_STATUS_OTA_PENDING      0x00000001U /* app asked for an update */
#define RECOVERY_STATUS_IMAGE_INVALID    0x00000002U /* image missing or CRC mismatch */
#define RECOVERY_STATUS_BOOT_UNCONFIRMED 0x00000003U /* app never proved it runs */
#define RECOVERY_STATUS_NO_APP           0x00000004U /* nothing bootable in flash */

/*---------------------------------------------------------------------------
 * Config struct magic
 *---------------------------------------------------------------------------*/

#define BOOT_CONFIG_MAGIC       0x424F4F54U  /* 'BOOT' */

/*---------------------------------------------------------------------------
 * Data structures
 *---------------------------------------------------------------------------*/

#pragma pack(push, 1)

/**
 * @brief Boot configuration, stored as two ping-pong copies in the config pages.
 *
 * Written only on OTA transitions and boot confirmation. Must be byte-identical
 * across bootloader and application compiles — hence the static assertion on
 * its size further down.
 *
 * `config_seq` is the ping-pong tiebreaker: `ota_config_write()` writes to the
 * page that does *not* hold the newest valid copy, with `config_seq`
 * incremented. A reader picks the valid copy with the highest sequence, so a
 * power loss during a write leaves the other copy intact and authoritative.
 *
 * `boot_confirmed` is the durable half of the boot-verification contract:
 *   ota_config_mark_valid(): set 0 — a freshly committed image has not run yet
 *   application:             set 1 once it is demonstrably healthy
 * The live half is a counter in the RTC backup registers, incremented by the
 * bootloader on every boot and zeroed by the application when it confirms. It
 * lives in the backup domain so a crash loop costs no flash erase cycles, and
 * is lost on power-on — which is correct, since a power cycle is a fresh start.
 * After BOOT_ATTEMPT_LIMIT consecutive unconfirmed boots the bootloader
 * declares the image bad and opens the long recovery window.
 */
typedef struct {
    uint32_t magic;             /* BOOT_CONFIG_MAGIC */
    uint32_t boot_mode;         /* BOOT_MODE_APP or BOOT_MODE_OTA */
    uint32_t fw_version;        /* currently installed firmware version */
    uint32_t pending_version;   /* version being delivered via OTA */
    uint32_t image_size;        /* size of current firmware image in bytes */
    uint32_t image_crc32;       /* CRC-32 of current firmware image */
    uint32_t update_status;     /* UPDATE_STATUS_* */
    uint32_t config_seq;        /* ping-pong sequence; higher wins */
    uint32_t boot_confirmed;    /* 1 after the app proved it can run */
    uint32_t boot_attempts;     /* consecutive unconfirmed boots */
    uint32_t reserved[3];       /* future use */
    uint32_t cfg_crc32;         /* CRC-32 of all preceding fields */
} BootConfig_t;

/**
 * @brief Fixed-point application state returned to the ESP32.
 *
 * Multi-byte values use the native little-endian representation shared by
 * the current STM32F1 and ESP32 targets. Validity and actuator state live in
 * flags so the payload stays compact and does not require software floats.
 */
typedef struct {
    uint32_t uptime_ms;
    int16_t  temperature_centi_c;
    uint16_t humidity_centi_percent;
    uint32_t pressure_pa;
    uint16_t light_lux;
    uint16_t flags;
    uint8_t  led_brightness;    /* raw WS2812B channel value, 0..255 */
    uint8_t  led_percent;       /* 0..100 relative to configured max */
} SensorSnapshot_t;

/**
 * @brief Cached runtime observability state returned to the ESP32.
 *
 * Sampled by vMonitorTask (every DIAG_SNAPSHOT_UPDATE_MS) and copied out on
 * CMD_DIAG_SNAPSHOT, so answering a request never walks the task lists.
 * Stack figures are FreeRTOS "high water mark" values: the smallest number of
 * *words* that ever remained free on that task's stack, so a smaller number
 * means less headroom. 0xFFFF means "never sampled / task not created".
 *
 * Little-endian like SensorSnapshot_t; no floats.
 */
typedef struct {
    uint32_t uptime_ms;                 /* since scheduler start */
    uint32_t free_heap_bytes;           /* xPortGetFreeHeapSize() */
    uint32_t min_ever_free_heap_bytes;  /* xPortGetMinimumEverFreeHeapSize() */
    uint16_t stack_high_water[DIAG_TASK_COUNT]; /* words left, DIAG_TASK_IDX_* */
    uint8_t  task_count;                /* entries valid in stack_high_water */
    uint8_t  sample_seq;                /* +1 per sample; detects a stale cache */
} DiagSnapshot_t;

/* Statically verify sizes */
#if defined(__cplusplus)
static_assert(sizeof(BootConfig_t) == 56, "BootConfig_t size mismatch");
static_assert(sizeof(SensorSnapshot_t) == 18,
              "SensorSnapshot_t size mismatch");
static_assert(sizeof(DiagSnapshot_t) == 22, "DiagSnapshot_t size mismatch");
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(BootConfig_t) == 56, "BootConfig_t size mismatch");
_Static_assert(sizeof(SensorSnapshot_t) == 18,
               "SensorSnapshot_t size mismatch");
_Static_assert(sizeof(DiagSnapshot_t) == 22,
               "DiagSnapshot_t size mismatch");
#endif

#pragma pack(pop)

/*---------------------------------------------------------------------------
 * CRC-32
 *---------------------------------------------------------------------------*/

/**
 * @brief Compute CRC-32 (Ethernet polynomial 0x04C11DB7).
 *
 * init=0xFFFFFFFF, final xor=0xFFFFFFFF, reflected in/out.
 * This implementation is shared across STM32, ESP32, and PC tools.
 *
 * @param data   Input buffer.
 * @param len    Length in bytes.
 * @param crc    Running finalized CRC (use 0 for the first block, then feed
 *                each returned value into the next call).
 * @return       Standard IEEE CRC-32 (polynomial 0xEDB88320) for all data
 *                processed so far.
 */
uint32_t proto_crc32(const uint8_t *data, size_t len, uint32_t crc);

/**
 * @brief One-shot CRC-32 over a buffer.
 */
static inline uint32_t proto_crc32_buf(const uint8_t *data, size_t len) {
    return proto_crc32(data, len, 0U);
}

/*---------------------------------------------------------------------------
 * Frame parser (re-entrant, no dynamic allocation)
 *---------------------------------------------------------------------------*/

/**
 * @brief Frame parser states.
 */
typedef enum {
    FRAME_STATE_SYNC = 0,
    FRAME_STATE_CMD,
    FRAME_STATE_LEN_LO,
    FRAME_STATE_LEN_HI,
    FRAME_STATE_DATA,
    FRAME_STATE_CRC0,
    FRAME_STATE_CRC1,
    FRAME_STATE_CRC2,
    FRAME_STATE_CRC3
} FrameState_t;

/**
 * @brief Decoded frame passed to the caller.
 */
typedef struct {
    uint8_t  cmd;                       /* command byte */
    uint16_t len;                       /* payload length */
    uint8_t  payload[PROTO_MAX_PAYLOAD];/* payload data */
} ProtoFrame_t;

/**
 * @brief Frame parser context (opaque; caller allocates as static/global).
 */
typedef struct {
    FrameState_t state;
    uint16_t     payload_idx;
    uint32_t     rx_crc;
    uint32_t     calc_crc;
    ProtoFrame_t frame;
} ProtoParser_t;

/**
 * @brief Initialize (or reset) a parser instance.
 */
void proto_parser_init(ProtoParser_t *p);

/**
 * @brief Feed one byte into the parser.
 *
 * @param p    Parser instance.
 * @param byte Incoming byte.
 * @return     Pointer to completed frame on success, NULL otherwise.
 *             The pointer is valid until the next proto_parser_feed() call.
 */
const ProtoFrame_t *proto_parser_feed(ProtoParser_t *p, uint8_t byte);

/*---------------------------------------------------------------------------
 * Frame building (sender side)
 *---------------------------------------------------------------------------*/

/**
 * @brief Build a protocol frame into a caller-provided buffer.
 *
 * @param buf       Output buffer (must be at least PROTO_MAX_FRAME bytes).
 * @param buf_size  Size of output buffer.
 * @param cmd       Command byte.
 * @param payload   Payload data (may be NULL if len == 0).
 * @param len       Payload length.
 * @return          Number of bytes written to buf, or 0 on error.
 */
uint16_t proto_build_frame(uint8_t *buf, uint16_t buf_size,
                           uint8_t cmd, const uint8_t *payload, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* SHARED_PROTOCOL_H */
