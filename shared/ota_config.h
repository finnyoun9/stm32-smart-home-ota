/**
 * @file    ota_config.h
 * @brief   Flash-based OTA configuration read/write utilities.
 *
 * Shared by bootloader and application. The config lives in the last two flash
 * pages (62 and 63). It is stored as two identical copies, one per page,
 * written alternately — see ota_config.c for why a single page cannot hold two
 * atomic copies. On STM32F103 flash writes are halfword-at-a-time and a page
 * must be erased (to 0xFF) before it can be programmed.
 */

#ifndef SHARED_OTA_CONFIG_H
#define SHARED_OTA_CONFIG_H

#include "protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------
 * API
 *---------------------------------------------------------------------------*/

/**
 * @brief Read the newest valid boot configuration.
 *
 * Reads both copies, validates magic + internal CRC on each, and returns the
 * one with the higher config_seq. A copy that fails validation is ignored, so
 * a power loss during a write cannot lose the previous configuration.
 *
 * @param cfg  Pointer to caller-allocated BootConfig_t.
 * @return     true if at least one valid copy exists.
 */
bool ota_config_read(BootConfig_t *cfg);

/**
 * @brief Read state and write the next config into the alternate page.
 *
 * Never erases the page holding the newest valid copy, so the write is
 * atomic with respect to power loss. Blocks during erase (~40ms/page on F103).
 * Call from a task, not an ISR.
 *
 * On success `cfg` receives the sequence number that was written.
 *
 * @param cfg  Config to write; magic and cfg_crc32 are set by this call.
 * @return     true on success.
 */
bool ota_config_write(BootConfig_t *cfg);

/**
 * @brief Prepare a BootConfig_t for writing: set magic and compute CRC.
 */
void ota_config_prepare(BootConfig_t *cfg);

/**
 * @brief Mark OTA request: sets boot_mode=BOOT_MODE_OTA and writes.
 *
 * Application calls this before NVIC_SystemReset().
 *
 * @param pending_version  Firmware version expected from OTA.
 * @param image_size       Expected image size in bytes (0 = unknown).
 * @return                 true on success.
 */
bool ota_config_request_update(uint32_t pending_version, uint32_t image_size);

/**
 * @brief Mark the current firmware as valid (bootloader calls on OTA success).
 *
 * @param version     New firmware version.
 * @param image_size  Image size in bytes (>0 enables boot-time CRC checking).
 * @param image_crc   CRC-32 of the written image.
 * @return            true on success.
 */
bool ota_config_mark_valid(uint32_t version, uint32_t image_size, uint32_t image_crc);

/**
 * @brief Mark the update as failed (bootloader calls on error).
 */
bool ota_config_mark_failed(void);

/*---------------------------------------------------------------------------
 * Boot verification (cross-reset handshake)
 *
 * bootloader: record_boot_attempt() on every boot, clear_attempts() once the
 *             application is known good or a fresh image was committed.
 * application: confirm_boot() once it is demonstrably healthy.
 * ---------------------------------------------------------------------------*/

/**
 * @brief Note that this boot has started and report how many consecutive boots
 *        have failed to confirm so far.
 *
 * Persisted in the RTC backup registers rather than flash, so it survives a
 * watchdog reset without costing flash erase cycles; the durable count lives in
 * BootConfig_t.boot_attempts and is only written when the count crosses a
 * threshold or is reset.
 *
 * @param out_attempts  Receives the attempt count for this boot (1-based).
 * @return              true if out_attempts was filled.
 */
bool ota_config_record_boot_attempt(uint32_t *out_attempts);

/**
 * @brief Reset the consecutive failed-boot counter.
 *
 * Called by the bootloader (after a successful commit or on a confirmed boot)
 * and by the application (via ota_config_confirm_boot()).
 */
bool ota_config_clear_attempts(void);

/**
 * @brief Application-side boot confirmation.
 *
 * Sets boot_confirmed=1 and clears the failed-boot counter. Until this runs,
 * the bootloader treats the next reset as evidence the image is bad.
 */
bool ota_config_confirm_boot(void);

#ifdef __cplusplus
}
#endif

#endif /* SHARED_OTA_CONFIG_H */
