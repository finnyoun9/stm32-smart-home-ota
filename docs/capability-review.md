# 第一性原理能力审查：这个项目对嵌入式求职到底值多少

> 审查基线：2026-09-17 仓库现状（代码 + README/CHANGELOG 的证据边界）。
> 目的：回答“哪些能力是招聘方真正在筛的、哪些只是看起来热闹、哪些该补深度而不是补功能”。
> 本文是决策文档，不是完成度声明。所有“已实现/未实现”判断以 README 的“已真机验证/规划中”表格为准。

---

## 一、五句话结论

```text
First Principle: 招聘方买的不是“你接了 12 个模块”，而是“把一块只有 64KB Flash / 20KB SRAM 的裸
                 MCU 交给你，出问题你能否从原理定位、并把固件安全地推上去还不砖”。这个项目的
                 不可替代资产是 Bootloader + 自定义协议 + 真实硬件证据链，不是传感器数量。

Non-negotiables: (1) 每一条“已验证”都要有可复现证据；(2) 只能在 8KB/54KB/2KB + 20KB SRAM 预算内；
                 (3) 任何升级路径都不能把设备变成砖；(4) 面试要能讲到寄存器和 linker 层。

Assumptions to Drop: “堆功能 = 提升逼格”。F103 上再挂 MPU6050/VL53L0X/DHT11/MQ-2/HC-SR04/舵机/雾化片，
                     边际说服力接近 0，还会稀释深度。传感器数量是 demo 味最重的信号。

Smallest Sufficient Path: 不再加外设。把 OTA 从“能升级”做成“坏了能恢复”，把 RAM/中断/DMA/启动流程
                           从“能用”做成“能讲”，再补可重复的测试与 CI。三件事都做完，项目从 demo 变成
                           “小型量产固件”。

Escalation Signal: F407 最小系统板若在简历投递时仍未上电验证，两个项目的“硬件链路”叙事同时失效——
                   这比本项目缺任何功能都严重。
```

---

## 二、这个项目真正的不可替代资产（别丢）

按“别人抄不走 / 没有你就没有”排序：

| 资产 | 为什么值钱 | 当前证据 |
|---|---|---|
| 从零写的 Bootloader + VTOR 重定位 + `.ramfunc` | 绝大多数应届简历只有 HAL 点灯和传感器。Bootloader 是“敢不敢让你碰量产固件”的分水岭 | `bootloader/Core/Src/main.c`，8KB/54KB/2KB 分区，`STM32F103C8TX_BOOT.ld` |
| 单 Bank Flash 约束下的 RAM 执行 | 能讲清“为什么擦写时 CPU 不能取指”，是理解 F1 架构的硬证据 | `__attribute__((section(".ramfunc")))` + 显式 `.data` 拷贝（startup 不管 `.ramfunc`） |
| 自定义帧协议 + 序号 ACK + 超时重试 | 跨设备协议设计 + 异常恢复，比“调通一个现成库”高一个层级 | `shared/protocol.{h,c}`，LEN 越界在 `FRAME_STATE_LEN_HI` 就拒绝 |
| DIN→DOUT 的证据边界排障 | 展示的是方法论（只信接收端证据），不是运气 | `docs/captures/ws2812/evidence/*.vcd` + README“最大难点” |
| 20KB SRAM 下的 5 任务预算管理 | 资源受限下的量化决策，PM/面试官都认 | `application/FreeRTOSConfig.h`（14KB heap）、17,900B RAM 实测 |
| 手机直连 SoftAP 完成 OTA | 无 PC 的端到端闭环，演示脚本闭环 | `tools/ota_regression.py` 自动化回归 |
| 三目标同源协议（STM32 boot / STM32 app / ESP32） | 真正的多节点系统，不是单板练习 | `shared/` + `esp32-comm-bridge/` |

**结论：这套东西的“逼格”已经够一个中级岗的地板了。真正的问题是深度没被证明，广度被过度证明。**

---

## 三、按“招聘方真正在筛的能力类别”分档

### A 档 · 必须有（嵌入式岗的硬门槛）

| 能力类别 | 本项目状态 | 缺口 / 需补 |
|---|---|---|
| 裸机 + HAL 混合编程 | ✅ 强 | — |
| Linker script / 启动流程 / 向量表 | ⚠️ 用了但未系统化 | 能讲清 `.ramfunc` 的 LMA/VMA、`_siramfunc` 拷贝、VTOR、MSP 重设；建议在 `docs/` 落一页启动流程图 |
| 中断与优先级（NVIC、FreeRTOS 优先级阈值） | ⚠️ 用了 | `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 只用来设 USART1；EXIT/编码器/I2C 未做优先级审计。补一张“中断优先级分配表 + 为什么” |
| Flash 生命周期与原子性 | ⚠️ 有硬伤（见第五节） | OTA 掉电会砖；配置区单副本无备份 |
| 资源预算（RAM/Flash/栈/heap） | ⚠️ 测过但未持续 | `configGENERATE_RUN_TIME_STATS=0`、无 HighWaterMark 常态化输出；补 monitor 任务打印最小剩余栈/heap |
| 协议设计 + 错误恢复 | ✅ 强 | 缺主机侧协议单测矩阵（CRC 向量/粘包/非法长度/重复乱序） |

### B 档 · 应该熟悉（区分“会写”和“懂系统”）

| 能力类别 | 本项目状态 | 缺口 / 需补 |
|---|---|---|
| DMA + UART IDLE 收包 | ❌ 明确没有（README 已声明） | **最高性价比的补强点**：当前 RXNE 单字节中断在 115200 下中断频率约 11.5k/s，全部进 ISR 再进 StreamBuffer。改成 DMA 循环 + IDLE 中断给 StreamBuffer，是 F1 上“数据手册级”的功能。这一项在面试里几乎必问 |
| 低功耗 / Tickless Idle | ⚠️ 有 `lowpower-probe` 但未纳入主线 | F103 + 15 颗 WS2812 + PIR 场景下，讲清 sleep/stop 模式取舍、IWDG 与 tickless 的冲突，比再加传感器值钱 |
| 可观测性（trace / profiling） | ❌ 没有 | 上 SEGGER SystemView 或 [RTEdbg](https://www.beningo.com/rtedbg-open-source-data-logging-and-tracing-for-embedded-systems/) 抓一次任务切换、阻塞时长、优先级反转，截图进 `docs/`。这是“高级感”最强的单点证据 |
| 故障注入 / 半实物测试 | ⚠️ 有人工回归 | `tools/ota_regression.py` 是好的开始；补“传输中断/CRC 错/重复 chunk/ESP32 复位/掉电”自动化用例表 |
| 测试与 CI | ⚠️ 有最小 smoke test | CI 已建三目标编译 + `protocol_smoke_test`；建议引入主机侧单元测试（如 Unity/CMock 风格）跑 parser 状态机的边界矩阵 |
| 版本与兼容性管理 | ❌ 没有 | 协议无 version 字段，固件镜像无版本协商/回滚保护。补 `PROTO_VERSION` + `OTP`/trailer |

### C 档 · 加分但要有理由

| 能力类别 | 判断 |
|---|---|
| 多节点网络（ESP-NOW / BLE GATT / LoRa） | F103 侧没有无线，无线全在 ESP32。**不建议**现在加，除非岗位明确要无线协议栈 |
| 云平台对接（MQTT + TLS） | MQTT 已写但未实机验证，且用的是公网 EMQX 沙盒。若要做，必须补 TLS + 证书校验，否则只是“连上了” |
| RTOS 高级特性（事件组、消息缓冲、任务通知、静态分配） | 已用事件组/StreamBuffer/静态 Stream Buffer。可深挖“为什么用 StreamBuffer 而不是 Queue” |
| 安全启动 / 固件签名 | 空。对 MCU 岗是加分不是门槛；但对 IoT/汽车电子岗是门槛，见第六节 |

### D 档 · 会减分

| 项 | 原因 |
|---|---|
| 继续挂 MPU6050 / VL53L0X / DHT11 / MQ-2 / HC-SR04 / SG90 / 雾化片 | 全是成熟模块 + 现成库。面试官只会问“这不就是调库吗”，并顺势怀疑 Bootloader 是不是也是抄的 |
| README 里大量“规划中”设备出现在引脚表主体 | 视觉上像功能清单，稀释了 Bootloader 的分量 |
| 文档/代码比例失衡 | 当前 docs + README + CHANGELOG ≈ 8k+ 行中文 prose vs ≈ 8.4k 行 C。面试官会怀疑“代码是谁写的” |
| AI 辅助段落写得太诚实但没给“你的判断点” | 现在的写法已经比大多数人好；但要补一句“哪些架构决策是我定的、AI 提议被我否掉的例子”，否则容易被读成“AI 写的项目” |

### E 档 · 明确不做

WebSocket、原生 App、Node-RED、加湿器联动、更多传感器。`docs/resume-roadmap.md` 的 P2 已经写了同样的判断，本次审查强化它：**这些只增加演示面，不增加说服力。**

---

## 四、必须能“讲到寄存器/链接器层”的清单

面试官区分“做过”和“懂”的方式，就是往下追问一层。以下每条都应当在仓库里有一页文档或一段代码注释支撑：

1. **为什么擦写 Flash 时 CPU 会停？**（单 Bank、取指总线仲裁、`.ramfunc` 的 LMA 在 Flash/VMA 在 RAM、谁把这个段拷进 RAM）
2. **为什么跳转前要 `__set_MSP` + `SCB->VTOR`？** PRIMASK 跨函数跳转不恢复、PendSV/SysTick 归属。
3. **USART1 中断优先级数值为什么设成 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`？** 数值越小优先级越高，以及 FreeRTOS 的 `FromISR` 红线。
4. **为什么是 StreamBuffer 而不是 Queue/环形数组？** 触发阈值、单生产者单消费者、静态分配、与 DMA 的耦合方式。
5. **IWDG 为什么寄存器直写而不走 HAL？** LSI 的 ±50% 漂移、跨 `NVIC_SystemReset()` 不清零、与 tickless idle 的冲突。
6. **1KB 页 = 1 chunk 的耦合是优点还是负债？** `addr = APP_BASE + seq*1024` 简单，但把 Flash 几何写死在协议语义里；换 F4（扇区不均）会怎样。
7. **CRC-32 逐块复用的数学前提**（当前实现先 `^0xFFFFFFFF` 再 `^0xFFFFFFFF`，是“可继续累积”的正确写法；要能证明为什么）。
8. **AHT20/BMP280 的固定点换算与误差预算**（当前无 FPU，为什么用 centi 单位）。

---

## 五、代码层面的真实风险（审查中确认，不是猜测）

这些不是“逼格”问题，是会被深挖的实锤。按严重度排序：

1. **OTA 掉电即砖（已知，README 已披露）。** 第一个 chunk 就擦除 app 第 0 页（含向量表），中断后 `app_is_valid()` 失败，设备停在 maintenance。**这是当前最大的可信度缺口**——因为它是唯一一个“本来可以做得更严谨、但没做”的点，而它恰好落在项目最核心的能力上。
2. **配置区单副本无原子性兜底。** `ota_config_write()` 先擦除 `CONFIG_BASE` 再写；写入途中掉电 → magic/CRC 失效 → `ota_config_read()` 返回 false → 版本号与状态丢失（应用本身仍能启动）。配置区给了 2KB 却只用 1 页，属于“空间已付钱、冗余没买”。
3. **协议实现存在镜像副本。** `shared/protocol.c` 与 `esp32-comm-bridge/src/protocol.cpp` 目前仅有注释差异（本次审查已 diff 确认），但 `esp32-comm-bridge/platformio.ini` 已经把这个同步风险写进注释。逻辑一旦漂移，就是最难查的一类 bug。这是一处“同一职责两个 owner”。
4. **`app_is_valid()` 只校验 SP 与 reset vector。** 没有 image CRC 复核、没有“升级后首次启动确认”的概念，因此无法做无 A/B 的最小回滚（见第六节）。
5. **MQTT 走公网明文 `mqtt://broker.emqx.io:1883` 且未实机验证。** 要么补 TLS + 去掉沙盒，要么在 README 里降到“实验分支”，不要与已验证能力并列。
6. **RAM 余量仅约 2KB（17,900/20,480）。** 任何新功能都会先撞 RAM 而不是 Flash。这本身就是很值得讲的一条工程约束，但前提是先把 HighWaterMark/heap 统计常态化。

---

## 六、提升逼格：借鉴成熟开源项目（有具体落点，不是“去读源码”）

原则：**借设计，不借代码**。借来的东西要能在面试里讲清“它解决什么问题、我为什么不能照抄、我改成了什么”。

### 6.1 OTA 从“能升级”做成“能恢复”（最高优先级）

- **[MCUboot](https://docs.mcuboot.com/design.html)** — 读它的 *image trailer / swap status / magic* 设计：
  - `swap-using-scratch`（用一小块 scratch 区做 A/B 交换）与 `direct-xip`（双槽各持可执行镜像）
  - `image_ok` / `BOOT_MAX_ALIGN` / 掉电可恢复的 swap 状态记录
  - 结论落到本项目：54KB app 里挪出约 1KB 做 **golden/backup 第 0 页**，先写第 0 页到备份，最后写向量页 → 掉电后仍能回到旧固件。这是 F103 上完全可实现的“穷人版 A/B”，比现在“掉电即砖”高一个量级。
- **[wolfBoot](https://www.wolfssl.com/wolfboot-on-stm32h5-enhancing-secure-boot-with-trustzone-m/)** — 看它在小 MCU 上如何用**极小的 bootloader** 做分区布局与镜像分区表（partition table 与 bootloader 解耦）。落点：把当前硬编码的 `APP_BASE/APP_SIZE` 换成一张分区表结构体，Bootloader 与 ESP32 共用。
- **STM32 官方 IAP 应用笔记 [AN3965](https://www.st.com/resource/en/application_note/dm00036049-stm32f40x-stm32f41x-in-application-programming-using-the-usart-stmicroelectronics.pdf) / AN4657** — 用于对照“官方推荐的 IAP 做法”，并在面试里说明你为什么不采用它（它不解决回滚，只解决搬运）。
- **升级确认（rollback protection）概念**：Zephyr/MCUboot 的 `test` vs `confirm` 镜像状态（[参考](https://docs.zephyrproject.org/latest/zephyr.pdf)）——落点：新固件启动后必须由应用写一次“我活着”标记，否则下次启动回滚。**这一条不需要 A/B 也能部分实现。**

### 6.2 协议层（低成本高回报）

**已完成的实测证据（2026-09-17）**：`tools/protocol_boundary_test.c` 现在覆盖 11 个用例 / 260 条断言，
全部通过（粘包、拆包、前导垃圾、非法长度含 0xFFFF 不越界写、逐字节位翻转、零长度、最大长度、缓冲区边界）。

测试过程中量化出一个**协议内建同步风险**，值得写进面试追问稿：

| 现象 | 实测 |
| --- | --- |
| 帧前出现单个游散 `0xA5` | 它被当成帧头，吃掉真帧的 SYNC → **整帧静默丢弃**（hits=0） |
| 失步后 parser 的行为 | 把下一个 `0xA5` 当作"某帧末 4 字节 CRC"来校验，即**帧边界由数据内容而非发送方成帧决定** |
| 2000 组随机 8 字节非 `0xA5` 前缀 | 0 次永久失步，最坏恢复延迟 20 字节 |
| 结构化伪帧探测（`A5+00000000`/`A5+FFFFFFFF`/`A5+A5A5A5A5`/`A5+01000000`） | 均未被误接受 |

结论：当前靠 ACK + 超时重传兜底是可接受的，但这是一个**能讲清楚、且知道如何根治**的已知边界（根治方案见下）。

- **COBS/COBS-R 或 SLIP 风格组帧**（[原始论文](https://dl.acm.org/doi/pdf/10.1145/263105.263168)）：`0xA5` 同步 + 长度 + CRC 的边界由数据内容决定，噪声下存在"假同步 + 长度域损坏"的窗口期。COBS 能给出**无歧义边界**，且不需要转义。落点：作为 v2 协议提案，用上面这套已建好的边界矩阵做 A/B 对比（现状 vs COBS 的坏帧恢复延迟）。
- **MAVLink 的教训**：schema 化消息 + 版本协商 + 心跳超时。落点：给现有帧加 `PROTO_VERSION` + `CMD_HELLO/HELLO_ACK` 握手，写清前后兼容策略。
- **参考实现**：`cobs` 社区实现均可对照测试向量。

### 6.3 RTOS 与可观测性

- **[SEGGER SystemView](https://github.com/SEGGERMicro/SystemView)** 或 **[RTEdbg](https://www.beningo.com/rtedbg-open-source-data-logging-and-tracing-for-embedded-systems/)**：抓一次任务切换/阻塞/优先级反转，导出截图。落点：解决“优先级反转是否真的在你系统里发生”这个只能靠数据回答的问题。
- **FreeRTOS 官方 `vTaskGetRunTimeStats` / `uxTaskGetStackHighWaterMark`**：当前 `configGENERATE_RUN_TIME_STATS=0`、HighWaterMark 未常态化。落点：monitor 任务周期性输出，回填 README 的 RAM 表。
- **Tickless Idle + IWDG 冲突**：这是低功耗与看门狗结合的经典陷阱，`docs/low-power-eco-demo-plan.md` 已有素材，但没进主线。

### 6.4 测试与工程化

- **主机侧单元测试**：把 `shared/protocol.c` 的 parser 状态机拿到 PC 上跑边界矩阵（非法长度、粘包、拆包、位翻转、重复/乱序 chunk）。当前 `tools/protocol_smoke_test.c` 是起点，不是矩阵。
- **故障注入表**：对齐 `docs/ota-regression-template.md`，把“中断/CRC 错/重复/ESP32 复位/掉电”变成可重复执行的脚本用例，输出结果表。
- **CI 增量**：已经很不错（三目标编译 + smoke test）。补 (a) 主机侧单测，(b) 每次构建产出 size diff（Flash/RAM 变化趋势），(c) 镜像签名/CRC 一致性校验。

### 6.5 安全（加分项，视岗位决定）

- 固件签名与验签：MCUboot 的 `imgtool` 签名流程 + ECDSA-P256 在小 MCU 上的验签成本。F103 无硬件加速，验签耗时会成为很好的面试话题（“为什么我选择 CRC 而非签名 / 签名要付多少 ms”）。
- 关键前提：**只有在 OTA 传输链路不受信时签名才有意义**。当前 UART 短链路 + ESP32 同板，属于“可解释地不做”，写清楚比硬做更好。

---

## 七、优先级建议（替代 roadmap 的 P2）

| 优先级 | 任务 | 产出/验收 | 预估 |
|---|---|---|---|
| **P0** | OTA 掉电可恢复（golden 第 0 页 或 最小 A/B） | 掉电注入测试通过：任一时刻断电后设备仍能启动旧固件 | 2–4 天 |
| **P0** | 配置区双副本 + 序列号轮换（利用已付钱的 2KB） | 配置写入掉电后能回退到上一份有效配置 | 0.5–1 天 |
| **P0** | DMA + UART IDLE 收包替换 RXNE 单字节中断 | 中断次数下降数据 + 波形/日志证据，写进 README | 1–2 天 |
| **P1** | 消灭协议镜像副本（单一 owner：构建脚本生成或符号链接） | diff 归零，CI 里加一致性检查 | 0.5 天 |
| **P1** | 协议主机侧边界测试矩阵 + CI 接入 | 覆盖非法长度/粘包/拆包/位翻转/乱序，结果表 | 1–2 天 |
| **P1** | FreeRTOS 运行时可观测性（SystemView/RTEdbg + HighWaterMark 常态化） | trace 截图 + RAM 余量真实数据 | 1–2 天 |
| **P1** | 中断优先级分配表 + 启动流程/`.ramfunc` 说明文档 | `docs/` 新增两页，面试可直接投屏 | 0.5–1 天 |
| **P2** | 协议 v2 提案（版本协商，或 COBS 组帧对比） | 设计文档 + 对比数据，不一定要上机 | 2–3 天 |
| **P2** | MQTT 实机验证 + TLS，或降级为实验分支 | 明确证据边界 | 1–2 天 |
| **P3** | 固件签名可行性分析（算清 F103 验签耗时） | 分析文档，不做实现 | 0.5 天 |

**明确不做**：新增 MPU6050/VL53L0X/DHT11/MQ-2/HC-SR04/SG90/雾化片；WebSocket；原生 App；Node-RED。

---

## 八、F407 项目的联动风险（本次审查的放大器）

`docs/resume-roadmap.md` 把两个项目定位成“固件深度 + 硬件链路”互补。这个组合是对的，但有一个致命前提：**F407 板必须真正上电 Bring-up**。

- 若 F407 仍是“PCB 已回板、未焊接验证”，那么本项目承担全部说服力，而它恰好缺“可靠性/可恢复性”这层深度 → 整体叙事停在“做得挺多”。
- 若 F407 完成限流上电 + SWD + 时钟 + GPIO/UART/ADC 验证，即使有飞线，本项目只需补上第 6.1 节的回滚能力，组合立刻进入“能独立负责一块板从硬件到固件”的层级。

**因此时间被压缩时的顺序应当是：F407 Bring-up ≥ 本项目 OTA 可恢复 > 其它一切。**

---

## 九、一句话收口

这个项目已经越过了“初级 demo”的门槛，它的定位问题是**广度自证过度、深度自证不足**：面试官能从 Bootloader 看出你不是抄教程，但也会从“12 个传感器 + 规划中 5 个设备”看出你在用功能量掩盖深度不确定。把 OTA 做成可恢复、把 UART 做成 DMA+IDLE、把 RTOS 做成可观测、把协议做成可测，这四件事做完，简历上就不需要再靠“外设数量”这个词撑场面了。

---

## 十、重启点与并行结构（2026-09-17 决策）

两条线**并行但独立仓库/独立里程碑**，避免互相阻塞：

### 线 A：F407VET6 最小系统板（独立模块，优先级最高）

必须单独推的理由：难度不在代码，在**一次性不可逆的物理动作**（焊接、上电）和**不可远程回滚的失败模式**（焊错 = 板子报废或芯片锁死）。所以它的推进方式与 OTA 线完全不同：阶段化、每阶段有硬停条件、每阶段留证据。

模块边界（详细 checklist 另开文档，不放在本审查里）：

- `docs/design-review.md` — 焊接前原理图复核（VDD/VSS 全组、VDDA/VSSA、VREF+、VBAT、VCAP1/2、BOOT0、NRST、HSE 负载电容、LDO 稳定电容）
- `docs/bringup-log.md` — 分阶段上电记录（LDO-only → MCU → HSI 最小程序 → HSE 168MHz → GPIO/UART/ADC），每阶段含限流值、实测电流、电压、纹波、结论
- `docs/v1-issues.md` — v1 问题清单与 v2 修改建议（飞线也要如实记录，这比“一次点亮”更值钱）
- `hardware/` — 原理图 PDF、Gerber、BOM、ERC/DRC 结果、实物照片

### 线 B：本仓库 OTA 深化（P0 → P1 顺序）

本仓库接下来的 P0/P1 清单见第七节。**注意：不要因为 F407 排期而把 OTA 线降到“维护状态”**——它是简历上唯一证明“固件系统集成深度”的项目，两者同时可用才构成组合叙事。

两条线的共同验收红线不变：**没有实机证据的能力不进简历。**
