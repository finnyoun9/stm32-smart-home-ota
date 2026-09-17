/**
 * @file    ota_config.c
 * @brief   Flash-based OTA configuration, stored as two ping-pong copies.
 *
 * Why two pages instead of two copies in one page
 * ----------------------------------------------
 * STM32F103 flash is erased a whole page at a time. Two copies inside one page
 * would both be destroyed by the erase that precedes a write, so a power loss
 * between "erase" and "program" would lose the configuration entirely. With one
 * copy per page, `ota_config_write()` always erases the page that is *not*
 * holding the newest valid copy; the other page keeps a complete, valid config
 * at every instant.
 *
 * `BootConfig_t.config_seq` orders the two copies. A reader validates each copy
 * independently (magic + internal CRC) and takes the valid one with the higher
 * sequence number, so:
 *   - interrupted write  -> new copy invalid, old copy still wins
 *   - both valid         -> the newer one wins
 *
 * Boot verification counter
 * -------------------------
 * The counter of consecutive unconfirmed boots is mirrored in the RTC backup
 * registers (`BKP_DR1`) so a watchdog or HardFault reset can increment it
 * without an erase cycle. The durable value in `BootConfig_t.boot_attempts` is
 * only written when it increases, which bounds flash wear.
 *
 * NOTE: Flash programming code must execute from RAM on STM32F1 (single bank).
 */

/* STM32 HAL first so FLASH_BASE/FLASH_PAGE_SIZE are defined before
 * protocol.h's #ifndef guards are evaluated (include-order independent). */
#include "stm32f1xx_hal.h"
#include "ota_config.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Internal helpers — placed in RAM (.ramfunc) for F1 single-bank safety
 *---------------------------------------------------------------------------*/

/**
 * @brief Erase a single 1KB flash page.
 *
 * Must run from RAM because the CPU stalls when fetching from flash during a
 * flash erase/program operation on single-bank F1 devices.
 */
__attribute__((section(".ramfunc"), noinline))
static uint32_t flash_erase_page(uint32_t page_addr) {
    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase   = FLASH_TYPEERASE_PAGES,
        .PageAddress = page_addr,
        .NbPages     = 1
    };
    uint32_t page_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK) {
        return page_error;
    }
    return 0;
}

/**
 * @brief Program a halfword to flash. Runs from RAM.
 */
__attribute__((section(".ramfunc"), noinline))
static HAL_StatusTypeDef flash_program_halfword(uint32_t addr, uint16_t data) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, data);
}

/*---------------------------------------------------------------------------
 * Copy addressing
 *---------------------------------------------------------------------------*/

static uint32_t copy_address(uint32_t page) {
    return FLASH_BASE + (page * FLASH_PAGE_SIZE);
}

/**
 * @brief Read and validate one copy.
 * @return true if the copy at `page` is present and self-consistent.
 */
static bool read_copy(uint32_t page, BootConfig_t *cfg) {
    memcpy(cfg, (const void *)copy_address(page), sizeof(BootConfig_t));

    if (cfg->magic != BOOT_CONFIG_MAGIC) {
        return false;
    }

    uint32_t computed = proto_crc32_buf((const uint8_t *)cfg,
                                        sizeof(BootConfig_t) - sizeof(uint32_t));
    return computed == cfg->cfg_crc32;
}

/**
 * @brief Decide which page the next write should go to.
 *
 * Writes alternate, but the tiebreaker is "never overwrite the newest valid
 * copy". If only one copy is valid, or neither is, the other page is used.
 *
 * @param seq_out  Receives the sequence number the next write should carry.
 */
static uint32_t next_write_page(uint32_t *seq_out) {
    BootConfig_t copy_a;
    BootConfig_t copy_b;
    bool a_valid = read_copy(CONFIG_PAGE_A, &copy_a);
    bool b_valid = read_copy(CONFIG_PAGE_B, &copy_b);

    if (a_valid && b_valid) {
        if (copy_a.config_seq >= copy_b.config_seq) {
            *seq_out = copy_a.config_seq + 1U;
            return copy_address(CONFIG_PAGE_B);
        }
        *seq_out = copy_b.config_seq + 1U;
        return copy_address(CONFIG_PAGE_A);
    }

    if (a_valid) {
        *seq_out = copy_a.config_seq + 1U;
        return copy_address(CONFIG_PAGE_B);
    }

    if (b_valid) {
        *seq_out = copy_b.config_seq + 1U;
        return copy_address(CONFIG_PAGE_A);
    }

    /* No valid copy: start fresh. */
    *seq_out = 1U;
    return copy_address(CONFIG_PAGE_A);
}

/*---------------------------------------------------------------------------
 * Public API — read / write
 *---------------------------------------------------------------------------*/

bool ota_config_read(BootConfig_t *cfg) {
    if (cfg == NULL) {
        return false;
    }

    BootConfig_t copy_a;
    BootConfig_t copy_b;
    bool a_valid = read_copy(CONFIG_PAGE_A, &copy_a);
    bool b_valid = read_copy(CONFIG_PAGE_B, &copy_b);

    if (a_valid && b_valid) {
        *cfg = (copy_a.config_seq >= copy_b.config_seq) ? copy_a : copy_b;
        return true;
    }
    if (a_valid) {
        *cfg = copy_a;
        return true;
    }
    if (b_valid) {
        *cfg = copy_b;
        return true;
    }
    return false;
}

void ota_config_prepare(BootConfig_t *cfg) {
    if (cfg == NULL) {
        return;
    }
    cfg->magic = BOOT_CONFIG_MAGIC;
    cfg->cfg_crc32 = proto_crc32_buf((const uint8_t *)cfg,
                                     sizeof(BootConfig_t) - sizeof(uint32_t));
}

bool ota_config_write(BootConfig_t *cfg) {
    if (cfg == NULL || cfg->magic != BOOT_CONFIG_MAGIC) {
        return false;
    }

    uint32_t seq = 0;
    uint32_t addr = next_write_page(&seq);
    cfg->config_seq = seq;
    ota_config_prepare(cfg);

    HAL_FLASH_Unlock();

    if (flash_erase_page(addr) != 0) {
        HAL_FLASH_Lock();
        return false;
    }

    const uint16_t *src = (const uint16_t *)cfg;
    size_t halfwords = sizeof(BootConfig_t) / 2;
    uint32_t dst = addr;

    for (size_t i = 0; i < halfwords; i++) {
        if (flash_program_halfword(dst, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        dst += 2;
    }

    HAL_FLASH_Lock();

    /* Verify by re-reading through the normal read path: this also proves the
     * new copy is the one a subsequent ota_config_read() would select. */
    BootConfig_t verify;
    if (!ota_config_read(&verify)) {
        return false;
    }
    return memcmp(cfg, &verify, sizeof(BootConfig_t)) == 0;
}

/*---------------------------------------------------------------------------
 * Public API — OTA transitions
 *---------------------------------------------------------------------------*/

bool ota_config_request_update(uint32_t pending_version, uint32_t image_size) {
    BootConfig_t cfg;

    /* Read existing config (preserve fw_version on first boot) */
    if (!ota_config_read(&cfg)) {
        memset(&cfg, 0, sizeof(cfg));
    }

    cfg.boot_mode       = BOOT_MODE_OTA;
    cfg.pending_version = pending_version;
    cfg.image_size      = image_size;
    cfg.update_status   = UPDATE_STATUS_IN_PROGRESS;
    cfg.boot_confirmed  = 0U;

    return ota_config_write(&cfg);
}

bool ota_config_mark_valid(uint32_t version, uint32_t image_size, uint32_t image_crc) {
    BootConfig_t cfg;

    if (!ota_config_read(&cfg)) {
        memset(&cfg, 0, sizeof(cfg));
    }

    cfg.boot_mode     = BOOT_MODE_APP;
    cfg.fw_version    = version;
    cfg.image_size    = image_size;
    cfg.image_crc32   = image_crc;
    cfg.update_status = UPDATE_STATUS_OK;
    /* The newly committed image has not run yet; it must confirm its own boot
     * before the bootloader will treat it as good. */
    cfg.boot_confirmed = 0U;

    return ota_config_write(&cfg);
}

bool ota_config_mark_failed(void) {
    BootConfig_t cfg;

    if (!ota_config_read(&cfg)) {
        return false;
    }

    cfg.boot_mode     = BOOT_MODE_APP;
    cfg.update_status = UPDATE_STATUS_FAILED;

    return ota_config_write(&cfg);
}

/*---------------------------------------------------------------------------
 * Boot verification
 *
 * BKP_DR1 holds the live attempt counter. It is written with a magic tag in
 * BKP_DR2 so a fresh power-on (backup domain lost) is distinguishable from a
 * reset (backup domain retained by VBAT/standby).
 *---------------------------------------------------------------------------*/

#define BOOT_ATTEMPTS_REG   BKP->DR1
#define BOOT_ATTEMPTS_MAGIC 0xB007U
#define BOOT_MAGIC_REG      BKP->DR2

static void backup_domain_init(void) {
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
}

bool ota_config_record_boot_attempt(uint32_t *out_attempts) {
    backup_domain_init();

    uint32_t attempts;

    if (BOOT_MAGIC_REG != BOOT_ATTEMPTS_MAGIC) {
        /* Backup domain was lost (cold power-on, not a reset): start counting
         * again. A power cycle is a fresh start, so losing the count is right. */
        attempts = 0U;
        BOOT_MAGIC_REG = BOOT_ATTEMPTS_MAGIC;
    } else {
        attempts = BOOT_ATTEMPTS_REG;
    }

    if (attempts < 0xFFFFU) {
        attempts++;
    }
    BOOT_ATTEMPTS_REG = attempts;

    if (out_attempts != NULL) {
        *out_attempts = attempts;
    }

    /* Deliberately no flash write here. The live count lives in the backup
     * register, which survives watchdog/HardFault/NVIC_SystemReset() resets —
     * exactly the resets a crash loop produces — so counting costs zero erase
     * cycles. Only the application's confirmation (ota_config_confirm_boot)
     * writes the durable copy, which bounds flash wear to OTA transitions plus
     * one write per successful, confirmed boot. */
    return true;
}

bool ota_config_clear_attempts(void) {
    backup_domain_init();
    BOOT_ATTEMPTS_REG = 0U;
    BOOT_MAGIC_REG = BOOT_ATTEMPTS_MAGIC;

    BootConfig_t cfg;
    if (ota_config_read(&cfg) && cfg.boot_attempts != 0U) {
        cfg.boot_attempts = 0U;
        return ota_config_write(&cfg);
    }
    return true;
}

bool ota_config_confirm_boot(void) {
    BootConfig_t cfg;

    if (!ota_config_read(&cfg)) {
        return false;
    }

    backup_domain_init();
    BOOT_ATTEMPTS_REG = 0U;
    BOOT_MAGIC_REG = BOOT_ATTEMPTS_MAGIC;

    if (cfg.boot_confirmed != 0U && cfg.boot_attempts == 0U) {
        /* Already confirmed and nothing to clear: avoid a pointless erase. */
        return true;
    }

    cfg.boot_confirmed = 1U;
    cfg.boot_attempts  = 0U;

    return ota_config_write(&cfg);
}
