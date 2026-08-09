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
#define PROTO_MAX_PAYLOAD       1024U
#define PROTO_HEADER_SIZE       4U    /* SYNC + CMD + LEN(2) */
#define PROTO_CRC_SIZE          4U    /* CRC32 LE */
#define PROTO_MAX_FRAME         (PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD + PROTO_CRC_SIZE)

/* Flash layout (shared so bootloader and app agree) */
#define FLASH_BASE              0x08000000U
#define FLASH_SIZE              0x00010000U   /* 64KB */
#define FLASH_PAGE_SIZE         1024U         /* 1KB per page on F103C8 */

#define BOOTLOADER_BASE         0x08000000U
#define BOOTLOADER_SIZE         0x00002000U   /* 8KB, pages 0-7 */
#define BOOTLOADER_PAGES        8U

#define APP_BASE                0x08002000U
#define APP_SIZE                0x0000D800U   /* 54KB, pages 8-61 */
#define APP_PAGES               54U

#define CONFIG_BASE             0x0800F800U   /* page 62 */
#define CONFIG_SIZE             0x00000800U   /* 2KB, pages 62-63 */
#define CONFIG_PAGE             62U

/* OTA window: bootloader waits this long (ms) for OTA_BEGIN after reset */
#define OTA_WINDOW_MS           200U

/* Per-chunk ACK timeout (ms) — ESP32 side */
#define OTA_CHUNK_TIMEOUT_MS    500U
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
#define CMD_SENSOR_DATA         0x21U  /* STM32 → ESP32: JSON telemetry */
#define CMD_CONTROL_CMD         0x22U  /* ESP32 → STM32: JSON control */
#define CMD_GET_STATUS          0x30U
#define CMD_RESET               0x31U

/* Slave → Master (STM32 → ESP32) */
#define CMD_OTA_BEGIN_ACK       0x81U
#define CMD_CHUNK_ACK           0x82U
#define CMD_NAK                 0x83U
#define CMD_OTA_RESULT          0x84U
#define CMD_STATUS_RSP          0x85U
#define CMD_CONTROL_ACK         0x86U  /* STM32 → ESP32: control confirm */

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
 * Config struct magic
 *---------------------------------------------------------------------------*/

#define BOOT_CONFIG_MAGIC       0x424F4F54U  /* 'BOOT' */

/*---------------------------------------------------------------------------
 * Data structures
 *---------------------------------------------------------------------------*/

#pragma pack(push, 1)

/**
 * @brief Configuration stored in the last two flash pages.
 *
 * Written only on OTA transitions. Must be byte-identical across
 * bootloader and application compiles.
 */
typedef struct {
    uint32_t magic;             /* BOOT_CONFIG_MAGIC */
    uint32_t boot_mode;         /* BOOT_MODE_APP or BOOT_MODE_OTA */
    uint32_t fw_version;        /* currently installed firmware version */
    uint32_t pending_version;   /* version being delivered via OTA */
    uint32_t image_size;        /* size of current firmware image in bytes */
    uint32_t image_crc32;       /* CRC-32 of current firmware image */
    uint32_t update_status;     /* UPDATE_STATUS_* */
    uint32_t reserved[4];       /* future use */
    uint32_t cfg_crc32;         /* CRC-32 of all preceding fields */
} BootConfig_t;

/* Statically verify sizes */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(BootConfig_t) == 56, "BootConfig_t size mismatch");
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
 * @param crc    Running CRC (use 0xFFFFFFFF for first block).
 * @return       Updated CRC value.
 */
uint32_t proto_crc32(const uint8_t *data, size_t len, uint32_t crc);

/**
 * @brief One-shot CRC-32 over a buffer.
 */
static inline uint32_t proto_crc32_buf(const uint8_t *data, size_t len) {
    return proto_crc32(data, len, 0xFFFFFFFFU) ^ 0xFFFFFFFFU;
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
