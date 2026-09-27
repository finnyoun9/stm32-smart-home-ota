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
 *
 * ===========================================================================
 * 中文导读：这个文件只解决一个问题 ——「复位/掉电后，状态还在吗」
 * ===========================================================================
 * 全文件 345 行，逻辑其实就三段：
 *
 *   ① 怎么擦、怎么写（flash_erase_page / flash_program_halfword）
 *      —— 平台细节，两个函数都必须放 RAM 执行
 *
 *   ② 两份配置选哪份、写哪份（next_write_page / read_copy / read / write）
 *      —— ★安全关键★ 掉电不丢配置全靠这一段
 *
 *   ③ 连续启动失败怎么计数（BKP 备份寄存器那一段）
 *      —— "这个固件能不能跑"的判定
 *
 * 读这个文件最容易卡住的地方是第 ②段：为什么要两份、为什么要两页、
 * 为什么"序号大的赢"。建议先读 next_write_page()，它把这套规则写全了。
 *
 * 本文件只在 STM32 上编译（依赖 HAL 的 Flash 接口），ESP32 和 PC 都编不了。
 * 所以主机测试 tools/ota_config_logic_test.c 是把第 ②段的"选择规则"重写了
 * 一遍来测的 —— 那是模型测试，不是本文件的代码测试。
 * ===========================================================================
 */

/* STM32 HAL first so FLASH_BASE/FLASH_PAGE_SIZE are defined before
 * protocol.h's #ifndef guards are evaluated (include-order independent). */
/* 中文：★这个 include 顺序是有意为之，不能调换★
 * protocol.h 里有 `#ifndef FLASH_BASE` 保护（防止和 HAL 重复定义）。
 * 如果先 include protocol.h，它就会自己定义 FLASH_BASE 等宏；之后 HAL 再
 * 定义时值可能不一致。先让 HAL 定义，protocol.h 的保护就自动跳过 ——
 * 这样头文件谁先谁后都安全（"include-order independent"）。 */
#include "stm32f1xx_hal.h"
#include "ota_config.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Internal helpers — placed in RAM (.ramfunc) for F1 single-bank safety
 *---------------------------------------------------------------------------*/
/* 中文：★为什么这两个函数要放 RAM 执行（面试高频）★
 *
 * STM32F103 只有**一个 Flash Bank**，代码和数据在同一块 Flash 里。
 * 当 Flash 控制器正在擦除/写入时，CPU 从 Flash 取指令会被硬件挡停
 * （总线仲裁优先给擦写操作）。如果擦 Flash 的代码本身也在 Flash 里，
 * 那就是"我正在擦我自己脚下的路"—— CPU 会卡死。
 *
 * 解法：把这两个函数编译到 RAM 里执行。
 *   `__attribute__((section(".ramfunc")))` 把函数放进 .ramfunc 段；
 *   链接脚本里 .ramfunc 的 VMA 在 RAM、LMA 在 Flash（> RAM AT > FLASH），
 *   启动时由 ramfunc_init() 把机器码从 Flash 拷到 RAM。
 *   `noinline` 防止编译器把它内联回调用方（内联了就又回到 Flash 了）。
 *
 * 反过来说：如果芯片有双 Bank 或支持从 RAM 擦另一 Bank（F4/F7 部分型号），
 * 就不需要这个技巧。这里是 F103 的硬约束。 */

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
        .NbPages     = 1              /* 中文：一次擦 1 页 = 1KB，与分页粒度一致 */
    };
    uint32_t page_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK) {
        return page_error;            /* 中文：非 0 = 出错，返回出错的页地址 */
    }
    return 0;                         /* 中文：0 = 成功。所以调用方判 "!= 0" 即失败 */
}

/**
 * @brief Program a halfword to flash. Runs from RAM.
 */
__attribute__((section(".ramfunc"), noinline))
static HAL_StatusTypeDef flash_program_halfword(uint32_t addr, uint16_t data) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, data);
}
/* 中文：★为什么是"半字"（2 字节）而不是字节★
 * STM32F1 的 Flash 编程接口最小单位就是 16 位。想写 1 个字节？做不到，
 * 必须凑成 2 字节一起写。这就是 write() 里要把 BootConfig_t 强转成
 * uint16_t* 并按 2 字节步进的原因。
 * 附带结论：BootConfig_t 的字节数必须是偶数（56 ✓），否则最后一次写会
 * 越界。这也是协议头里那条 56 字节静态断言的意义之一。 */

/*---------------------------------------------------------------------------
 * Copy addressing
 *---------------------------------------------------------------------------*/
/* 中文：所谓"两份配置"就是把同一个 BootConfig_t 结构体，分别放在 62 页和
 * 63 页的开头。页号 → 地址的换算就下面这一行。 */

static uint32_t copy_address(uint32_t page) {
    return FLASH_BASE + (page * FLASH_PAGE_SIZE);
}

/**
 * @brief Read and validate one copy.
 * @return true if the copy at `page` is present and self-consistent.
 */
static bool read_copy(uint32_t page, BootConfig_t *cfg) {
    /* 中文：直接把 Flash 当普通内存读（Flash 在地址空间里是可读的），
     * 不需要任何"读 Flash"的特殊 API。 */
    memcpy(cfg, (const void *)copy_address(page), sizeof(BootConfig_t));

    if (cfg->magic != BOOT_CONFIG_MAGIC) {
        return false;   /* 中文：魔数不对 = 这页根本没写过配置（擦除后是 0xFF） */
    }

    /* 中文：★CRC 只覆盖前 13 个字段，也就是 sizeof - 最后一个 4 字节★
     * 因为最后一个字段 cfg_crc32 就是校验值本身，不能把自己算进去。
     * 这两个"减法"（sizeof 减 4）在读出和写入时都必须一致，错一个字节
     * 就会导致所有配置永远校验失败。 */
    uint32_t computed = proto_crc32_buf((const uint8_t *)cfg,
                                        sizeof(BootConfig_t) - sizeof(uint32_t));
    return computed == cfg->cfg_crc32;
}
/* 中文：★这个函数返回 false 不是"出错"，而是"这份副本不可用"★
 * 两种正常情况都会返回 false：
 *   · 那一页从来没写过（magic 是 0xFF）
 *   · 那一页正在写的过程中掉电了（magic 对但 CRC 不过）
 * 调用方据此决定用另一份。这正是"掉电不会丢配置"的实现方式：
 * 坏的那份被无视，好的那份继续用。 */

/**
 * @brief Decide which page the next write should go to.
 *
 * Writes alternate, but the tiebreaker is "never overwrite the newest valid
 * copy". If only one copy is valid, or neither is, the other page is used.
 *
 * @param seq_out  Receives the sequence number the next write should carry.
 */
static uint32_t next_write_page(uint32_t *seq_out) {
    /* 中文：★这是整套掉电安全机制的核心，就这几行★
     *
     * 四种情况，穷举如下（A/B 指两份副本是否有效）：
     *
     *   A有效 B有效 → 谁序号大谁是最新；写"另一页"，序号 = 最新的 +1
     *   A有效 B无效 → 写 B 页，序号 = A 的 +1        （B 是坏的那份，可以擦）
     *   A无效 B有效 → 写 A 页，序号 = B 的 +1
     *   A无效 B无效 → 全新设备，写 A 页，序号从 1 开始
     *
     * 一句话概括：**永远擦"不是最新有效那份"所在的那一页**。
     * 因此被擦的那一页即使擦到一半掉电，另一页仍然是完整的旧配置，
     * 设备下次上电照常能启动。
     *
     * 为什么"序号 +1"很重要：如果两份都有效但序号相同（理论上不该发生），
     * 读取时用 `>=` 比较会让 A 胜出 —— 这是确定的规则，不是随机行为。 */
    BootConfig_t copy_a;
    BootConfig_t copy_b;
    bool a_valid = read_copy(CONFIG_PAGE_A, &copy_a);
    bool b_valid = read_copy(CONFIG_PAGE_B, &copy_b);

    if (a_valid && b_valid) {
        if (copy_a.config_seq >= copy_b.config_seq) {
            *seq_out = copy_a.config_seq + 1U;
            return copy_address(CONFIG_PAGE_B);   /* 中文：A 更新 → 写 B */
        }
        *seq_out = copy_b.config_seq + 1U;
        return copy_address(CONFIG_PAGE_A);       /* 中文：B 更新 → 写 A */
    }

    if (a_valid) {
        *seq_out = copy_a.config_seq + 1U;
        return copy_address(CONFIG_PAGE_B);       /* 中文：B 坏了 → 安全，擦它 */
    }

    if (b_valid) {
        *seq_out = copy_b.config_seq + 1U;
        return copy_address(CONFIG_PAGE_A);
    }

    /* No valid copy: start fresh. */
    *seq_out = 1U;
    return copy_address(CONFIG_PAGE_A);
}
/* 中文：这个函数不写任何东西，只做"决策"。真正的擦写发生在
 * ota_config_write() 里。把决策和动作分开，让这段最容易出错的规则可以被
 * 单独理解和测试（tools/ota_config_logic_test.c 测的就是这段规则）。 */

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

    /* 中文：★选择规则：两份都有效就比序号，用 >= 让 A 在并列时胜出★
     * 并列其实不该出现（next_write_page 每次都 +1），但代码里必须有确定
     * 的行为，不能"看运气"。 */
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
    return false;   /* 中文：两份都不可用 —— 全新设备，或配置区被擦坏了 */
}

void ota_config_prepare(BootConfig_t *cfg) {
    if (cfg == NULL) {
        return;
    }
    cfg->magic = BOOT_CONFIG_MAGIC;
    /* 中文：算 CRC 时同样只覆盖前 13 个字段（减掉 cfg_crc32 自己）。
     * 顺序很重要：必须先填完所有字段、再算 CRC —— 所以 write() 里
     * 先设置 config_seq 再调 prepare()，不能反过来。 */
    cfg->cfg_crc32 = proto_crc32_buf((const uint8_t *)cfg,
                                     sizeof(BootConfig_t) - sizeof(uint32_t));
}

bool ota_config_write(BootConfig_t *cfg) {
    if (cfg == NULL || cfg->magic != BOOT_CONFIG_MAGIC) {
        return false;   /* 中文：magic 没填说明调用方没走 prepare()，直接拒绝 */
    }

    uint32_t seq = 0;
    uint32_t addr = next_write_page(&seq);   /* 中文：先决策写哪一页、序号是多少 */
    cfg->config_seq = seq;                   /* 中文：★必须在 prepare() 之前★
                                              * 因为 CRC 要覆盖 config_seq */
    ota_config_prepare(cfg);                 /* 中文：填 magic + 算 CRC */

    HAL_FLASH_Unlock();                      /* 中文：解锁 Flash 控制器才能擦写 */

    /* 中文：★★ 到这里为止，配置还没被破坏。下面这一行才是危险时刻 ★★
     * 擦除掉的永远是"另一页"，所以即使这一行之后立刻掉电，
     * 另一页的旧配置仍然完好，设备照常启动。 */
    if (flash_erase_page(addr) != 0) {
        HAL_FLASH_Lock();
        return false;
    }

    /* 中文：把结构体当 uint16_t 数组，按 2 字节步进写 —— 因为 F1 的 Flash
     * 编程最小单位是半字。dst 每次 +2 字节。 */
    const uint16_t *src = (const uint16_t *)cfg;
    size_t halfwords = sizeof(BootConfig_t) / 2;   /* 中文：56/2 = 28 个半字 */
    uint32_t dst = addr;

    for (size_t i = 0; i < halfwords; i++) {
        if (flash_program_halfword(dst, src[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;   /* 中文：写到一半失败 → 这一页是坏的，
                             * 但读取时会因为 CRC 不过而自动回退到旧的那页 */
        }
        dst += 2;
    }

    HAL_FLASH_Lock();

    /* Verify by re-reading through the normal read path: this also proves the
     * new copy is the one a subsequent ota_config_read() would select. */
    /* 中文：★校验方式很讲究：不是"回读刚才写的地址比字节"，而是走正常的
     * ota_config_read() 再比一次★ 好处是顺带证明了"下次读取真的会选中
     * 我这份新的"。如果只比字节，可能出现"写对了但选不中"（比如序号算错）
     * 这种更隐蔽的问题，那样就查不出来。
     * 这是"用真实路径验证"而不是"用旁路验证"的典型例子。 */
    BootConfig_t verify;
    if (!ota_config_read(&verify)) {
        return false;
    }
    return memcmp(cfg, &verify, sizeof(BootConfig_t)) == 0;
}

/*---------------------------------------------------------------------------
 * Public API — OTA transitions
 *---------------------------------------------------------------------------*/
/* 中文：★这三个函数的共同套路：读 → 改 → 写★
 * 不是简单地把结构体整体覆盖，因为每次只改几个字段，其它字段（比如
 * fw_version、image_crc32）必须保留。所以都是先 read 出现有配置，
 * 改掉要改的字段，再整体写回另一页。 */

bool ota_config_request_update(uint32_t pending_version, uint32_t image_size) {
    BootConfig_t cfg;

    /* Read existing config (preserve fw_version on first boot) */
    /* 中文：读不出来的话就用一块全 0 的配置起步。全 0 的 magic 不等于
     * BOOT_CONFIG_MAGIC，所以这份"空配置"不会被误读成有效配置。 */
    if (!ota_config_read(&cfg)) {
        memset(&cfg, 0, sizeof(cfg));
    }

    cfg.boot_mode       = BOOT_MODE_OTA;      /* 中文：★告诉 bootloader"我要升级"★ */
    cfg.pending_version = pending_version;
    cfg.image_size      = image_size;         /* 中文：★期望大小，传输开始前就落盘★ */
    cfg.update_status   = UPDATE_STATUS_IN_PROGRESS;
    cfg.boot_confirmed  = 0U;                 /* 中文：新固件还没跑过，先不算数 */

    return ota_config_write(&cfg);
}
/* 中文：这个函数由**应用**调用，紧跟着就是 NVIC_SystemReset()。
 * 它是应用唯一一次触碰 Flash —— 之后所有 Flash 操作都在 bootloader 里。
 * 这样划分的好处：应用正常运行期间完全不用管 Flash 的擦写时序和
 * .ramfunc 约束，出问题的面小很多。 */

bool ota_config_mark_valid(uint32_t version, uint32_t image_size, uint32_t image_crc) {
    BootConfig_t cfg;

    if (!ota_config_read(&cfg)) {
        memset(&cfg, 0, sizeof(cfg));
    }

    cfg.boot_mode     = BOOT_MODE_APP;        /* 中文：正常启动模式，不再是 OTA */
    cfg.fw_version    = version;              /* 中文：VERSION 命令回的就是这个 */
    cfg.image_size    = image_size;           /* 中文：★留给下次上电复核用★ */
    cfg.image_crc32   = image_crc;            /* 中文：★留给下次上电复核用★ */
    cfg.update_status = UPDATE_STATUS_OK;
    /* The newly committed image has not run yet; it must confirm its own boot
     * before the bootloader will treat it as good. */
    cfg.boot_confirmed = 0U;
    /* 中文：★最后这一行是回滚机制的关键★
     * 刚写完的镜像"字节正确"但"还没证明能跑"。所以 boot_confirmed 置 0，
     * 如果它启动就崩，计数会涨到 3，bootloader 就判定它坏了。
     * 只有应用自己调 confirm_boot() 才把这一位置 1。 */

    return ota_config_write(&cfg);
}

bool ota_config_mark_failed(void) {
    BootConfig_t cfg;

    if (!ota_config_read(&cfg)) {
        return false;   /* 中文：连旧配置都没有，没什么可标记的 */
    }

    cfg.boot_mode     = BOOT_MODE_APP;
    cfg.update_status = UPDATE_STATUS_FAILED;   /* 中文：留个"上次失败了"的痕迹 */

    return ota_config_write(&cfg);
}

/*---------------------------------------------------------------------------
 * Boot verification
 *
 * BKP_DR1 holds the live attempt counter. It is written with a magic tag in
 * BKP_DR2 so a fresh power-on (backup domain lost) is distinguishable from a
 * reset (backup domain retained by VBAT/standby).
 *---------------------------------------------------------------------------*/
/* 中文：★BKP 备份寄存器是什么，为什么用它★
 *
 * BKP 是 STM32 内部一小块由"备份域"供电的寄存器（BKP->DR1..DR10，每个 16 位）。
 * 备份域只要 VBAT 或主电在就不会丢，所以：
 *   · 看门狗复位   → 计数保留
 *   · HardFault 复位 → 计数保留
 *   · NVIC_SystemReset() → 计数保留
 *   · 彻底断电     → 计数丢失  ← 这正是我们想要的
 *
 * 为什么不用 Flash 存实时计数：崩溃循环会反复复位，每复位一次写一次 Flash，
 * 配置页大约 1 万次擦写就报废了 —— 而崩溃循环恰恰是最容易触发这条路径的
 * 场景。用 BKP 计数是零 Flash 磨损的。掉电丢失也没关系：掉电本来就是
 * "全新开始"，不该继承上一次的失败计数。
 *
 * DR2 存一个魔数（0xB007，故意写成"BOOT"样子的 leet），用来区分
 * "刚上电（备份域被清空）"和"只是复位（备份域还在）"。
 * 如果没有这个魔数，掉电后 DR1 里可能残留随机值，计数就乱了。 */

#define BOOT_ATTEMPTS_REG   BKP->DR1
#define BOOT_ATTEMPTS_MAGIC 0xB007U
#define BOOT_MAGIC_REG      BKP->DR2

static void backup_domain_init(void) {
    /* 中文：备份域默认是写保护的（防止误改 RTC/BKP）。要用必须先
     * 开 PWR 和 BKP 时钟，再调 HAL_PWR_EnableBkUpAccess() 解锁。 */
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
        BOOT_MAGIC_REG = BOOT_ATTEMPTS_MAGIC;   /* 中文：打上"我初始化过"的标记 */
    } else {
        attempts = BOOT_ATTEMPTS_REG;           /* 中文：复位，接着上次的数 */
    }

    /* 中文：封顶在 0xFFFF，防止无限自增溢出回绕（16 位寄存器）。 */
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
    /* 中文：★这段注释值得背下来，面试常问"你怎么保护 Flash 寿命"★
     * 一句话：实时计数只在 BKP（零磨损），Flash 只在"确认成功"时写一次。
     * 于是 Flash 的擦写次数 = 每次 OTA + 每次成功启动一次，
     * 崩 100 次也只多写 0 次。 */
    return true;
}

bool ota_config_clear_attempts(void) {
    backup_domain_init();
    BOOT_ATTEMPTS_REG = 0U;
    BOOT_MAGIC_REG = BOOT_ATTEMPTS_MAGIC;

    /* 中文：BKP 清零了，但 Flash 里的持久值只在确实非 0 时才写 ——
     * 避免每次上电都白擦一次 Flash。这种"先判断再写"的写法在嵌入式里
     * 是常规操作，因为写 Flash 是有代价的（磨损 + 40ms 阻塞）。 */
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
        return true;    /* 中文：★快速返回，不写 Flash★
                         * 正常启动的第二次之后都会走这条路，
                         * 于是"正常运行"对 Flash 寿命零消耗。 */
    }

    cfg.boot_confirmed = 1U;   /* 中文：★"我活着"的持久化标记★ */
    cfg.boot_attempts  = 0U;

    return ota_config_write(&cfg);
}
/* 中文：★调用它的地方：application/Core/Src/main.c 的 vControlTask★
 * 启动后先 vTaskDelay(3 秒)，再调这个函数。
 * 如果固件在初始化阶段就崩，永远走不到这里 → 计数继续涨 → 3 次后
 * bootloader 判定坏镜像 → 打开 10 秒恢复窗口等 ESP32 重推固件。
 * 整个"自动回滚"链条的起点就是这一行。 */
