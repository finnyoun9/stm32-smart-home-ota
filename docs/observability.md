# Runtime Observability — 运行时余量诊断

> 一句话：把「还剩多少 RAM、每个任务还剩多少栈」从文档里的静态快照，变成可以从 ESP32（或 PC 串口）随时查询的**运行时数据**。

关联代码：

| 位置 | 作用 |
| --- | --- |
| `shared/protocol.h` | `CMD_DIAG_SNAPSHOT` / `CMD_DIAG_SNAPSHOT_RSP` / `DiagSnapshot_t` |
| `application/Core/Src/main.c` | `vMonitorTask` 每 5s 采样 → `g_diag_snapshot`；`cmd_handler_dispatch()` 回包 |
| `application/FreeRTOSConfig.h` | `INCLUDE_uxTaskGetStackHighWaterMark=1`、中断优先级红线 |

---

## 1. 数据流

```
vMonitorTask (1s 循环)
  ├─ 每 5s (DIAG_SNAPSHOT_UPDATE_MS)
  │    ├─ uxTaskGetStackHighWaterMark(handle)  ×4 任务   ← 句柄在 xTaskCreate 时保存
  │    ├─ xPortGetFreeHeapSize()
  │    ├─ xPortGetMinimumEverFreeHeapSize()
  │    └─ taskENTER_CRITICAL() → g_diag_snapshot = sample (22 B 结构体赋值)
  └─ 每 1s iwdg_refresh()

ESP32 ──CMD_DIAG_SNAPSHOT(0x33)──► vCommTask → cmd_handler_dispatch()
      ◄─CMD_DIAG_SNAPSHOT_RSP(0x88)── 直接把 g_diag_snapshot 拷出来发走
```

两个设计点：

* **采样与应答解耦**：命令处理路径只做一次 22 B 结构体拷贝，不遍历 FreeRTOS 任务列表、不调用 `uxTaskGetSystemState()`（那个 API 需要一个 `TaskStatus_t[]`，本项目 4 个任务就要 100 B 以上的临时数组，在 88%+ RAM 占用下不可接受）。
* **看门狗节奏不能跟着改**：IWDG 超时约 4s，`vMonitorTask` 的 `vTaskDelay` 保持 **1s**，只把诊断采样节流到 5s。若把整个循环周期改成 5s，看门狗会先复位。

### 内存预算（硬约束：新增 static ≤ 256 B）

| 对象 | 大小 | 说明 |
| --- | --- | --- |
| `g_diag_task_handles[4]` | 16 B | 4 × `TaskHandle_t`(4 B) |
| `g_diag_snapshot` | 24 B | 22 B 结构体 + 2 B 对齐填充 |
| **合计** | **40 B** | 无堆分配、无第三方库、无新增缓冲区 |

实测构建增量：**RAM +40 B，Flash +272 B**（对比 HEAD 基线，见 §6）。

---

## 2. 命令定义

| 方向 | 名称 | ID | payload |
| --- | --- | --- | --- |
| ESP32 → STM32 | `CMD_DIAG_SNAPSHOT` | `0x33` | 空（0 字节） |
| STM32 → ESP32 | `CMD_DIAG_SNAPSHOT_RSP` | `0x88` | 22 字节，见 §3 |

ID 冲突检查（写入前核对过 `shared/protocol.h`）：主→从已用 `0x10..0x14, 0x20, 0x30..0x32`，`0x33` 空闲；从→主已用 `0x81..0x87`，`0x88` 空闲。

### 帧格式

沿用全项目统一的帧格式（大端写长度，CRC-32 小端）：

```
[0xA5] [CMD 1B] [LEN_L 1B] [LEN_H 1B] [PAYLOAD LEN B] [CRC32 4B LE]
```

请求帧（4 字节头 + 0 字节 payload + 4 字节 CRC = 8 字节），CRC 覆盖 `A5 33 00 00`：

```
A5 33 00 00 E2 6D DC 7D
```

---

## 3. `CMD_DIAG_SNAPSHOT_RSP` payload 字段表

`DiagSnapshot_t`，`#pragma pack(1)`，小端（与 `SensorSnapshot_t` 同约定），**全部整数/定点，无浮点**。共 22 字节。

| 偏移 | 长度 | 类型 | 字段 | 单位 | 含义 |
| --- | --- | --- | --- | --- | --- |
| 0 | 4 | `uint32` | `uptime_ms` | ms | 采样时刻，自调度器启动（`xTaskGetTickCount() * portTICK_PERIOD_MS`） |
| 4 | 4 | `uint32` | `free_heap_bytes` | B | `xPortGetFreeHeapSize()`：**当前**剩余 FreeRTOS heap |
| 8 | 4 | `uint32` | `min_ever_free_heap_bytes` | B | `xPortGetMinimumEverFreeHeapSize()`：启动以来的**历史最小**剩余 heap |
| 12 | 2 | `uint16` | `stack_high_water[0]` | word | Comm 任务历史最小剩余栈 |
| 14 | 2 | `uint16` | `stack_high_water[1]` | word | Control 任务历史最小剩余栈 |
| 16 | 2 | `uint16` | `stack_high_water[2]` | word | App 任务历史最小剩余栈 |
| 18 | 2 | `uint16` | `stack_high_water[3]` | word | Monitor 任务历史最小剩余栈 |
| 20 | 1 | `uint8` | `task_count` | 个 | 有效条目数（当前恒为 4） |
| 21 | 1 | `uint8` | `sample_seq` | — | 采样序号，每次采样 +1 并自然回绕；用于区分「缓存是新的」还是「vMonitorTask 没在跑」 |

### 解读规则

* **`stack_high_water` 是"水位线"，越小越危险**：它等于该任务自创建以来剩余栈的最小值（单位 **word**，Cortex-M3 上 1 word = 4 B）。典型 768 word 的栈如果报 100，意思是"最深的一次只用到 668 word，还剩 100 word = 400 B"。
* **换算成字节**：`bytes_free = stack_high_water * 4`。经验红线：`< 32 word (128 B)` 视为告警，需要加栈或减局部变量。
* **`0xFFFF`**：该槽位对应的任务句柄为空（任务没创建成功），不是"水位线 65535"。
* **`free_heap_bytes` vs `min_ever_free_heap_bytes`**：两者差距就是"已经被峰值吃掉、再也不会还回来的余量"。
* **`sample_seq` 不变**：说明 `vMonitorTask` 卡住或没被调度（例如更高优先级任务饿死它），这本身就是健康告警——注意此时 heap/水位线数字都是**过期值**。
* `task_count` 目前恒为 4；解析方应按它决定读多少个水位线条目，为将来加任务留出兼容空间。

---

## 4. 手动发起一次请求

### 方法 A：PC 串口直连（推荐，无需改 ESP32）

USB-TTL 接 STM32 的 PA9/PA10，115200 8N1，用仓库自带的 `tools/ota_sender.py` 里的帧构造/解析函数（同一套 CRC-32 与字节序）：

```python
# python3 diag_probe.py /dev/tty.usbserial-XXXX
import struct, sys
import serial
sys.path.insert(0, 'tools')
from ota_sender import build_frame, parse_frame

CMD_DIAG_SNAPSHOT     = 0x33
CMD_DIAG_SNAPSHOT_RSP = 0x88

with serial.Serial(sys.argv[1], 115200, timeout=0.1) as ser:
    ser.reset_input_buffer()
    ser.write(build_frame(CMD_DIAG_SNAPSHOT, b''))
    rsp = parse_frame(ser, 1000)
    if rsp is None or rsp[0] != CMD_DIAG_SNAPSHOT_RSP:
        print('no diag response:', rsp); raise SystemExit(1)
    p = rsp[1]
    uptime, free_heap, min_heap = struct.unpack_from('<III', p, 0)
    hw = struct.unpack_from('<4H', p, 12)
    count, seq = struct.unpack_from('<BB', p, 20)
    print(f'uptime={uptime} ms  free_heap={free_heap} B  min_ever={min_heap} B  seq={seq}')
    for name, w in zip(('Comm', 'Control', 'App', 'Monitor'), hw[:count]):
        print(f'  {name:<8} high-water {w:5d} word = {w*4:5d} B free')
```

> 注意：STM32 的 USART1 平常由 ESP32 占用。PC 直连时需先让 ESP32 不驱动这条链路（拔掉/释放其 TX），否则总线争用会打乱帧。

### 方法 B：手工回放原始字节

串口工具里直接发这 8 个字节（等价于方法 A 的请求）：

```
A5 33 00 00 E2 6D DC 7D
```

响应示例（**构造的示例，用于对照解码格式，不是实机采集**）：

```
A5 88 16 00 | 40 E2 01 00 | 39 30 00 00 | 94 26 00 00 | 64 00 78 00 50 00 C8 00 | 04 07 | BB BA 51 54
  sync cmd len   uptime=123456  free=12345   min_ever=9876  hw=(100,120,80,200) w  n=4 seq=7   CRC(LE)
```

即：Comm 剩 100 word(400 B)、Control 剩 120 word(480 B)、App 剩 80 word(320 B)、Monitor 剩 200 word(800 B)，且第 7 次采样时 heap 还剩 12,345 B、历史最低到过 9,876 B。

### 方法 C：从 ESP32 侧转发

ESP32 桥 `esp32-comm-bridge/src/main.cpp` 目前只轮询 `CMD_GET_SENSOR_SNAPSHOT`（web dashboard 用）。要在网页上看到诊断数据，需要仿照那段逻辑加一次 `CMD_DIAG_SNAPSHOT` 轮询，并把 22 B payload 解析进 JSON —— **本次未做**，属后续工作。

---

## 5. 中断优先级分配表

### 前提：分组与位宽

| 项 | 值 | 来源 |
| --- | --- | --- |
| Cortex-M3 NVIC 优先级位 | 4 位（16 级，数值越大优先级越低） | `FreeRTOSConfig.h` `configPRIO_BITS = 4` |
| 优先级分组 | `NVIC_PRIORITYGROUP_4` = **4 位抢占 / 0 位子优先级** | `HAL_Init()` → `HAL_NVIC_SetPriorityGrouping()`（STM32Cube HAL） |
| 推论 | 子优先级位宽为 0，`HAL_NVIC_SetPriority(irq, p, sub)` 的 `sub` 无实际作用（代码里一律写 0） | 同上 |
| HAL SysTick 时基优先级 | `TICK_INT_PRIORITY = 0x0F`（`HAL_InitTick()` 先设一次） | Cube HAL `stm32f1xx_hal_conf.h` |
| 之后 | SysTick 被 FreeRTOS 端口层改写成 15（见下表） | `port.c` `xPortStartScheduler()` |

### 红线：`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`

```
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 0x05      (FreeRTOSConfig.h)
configMAX_SYSCALL_INTERRUPT_PRIORITY        = 0x05 << 4 = 0x50   （写入 BASEPRI 的值）
configKERNEL_INTERRUPT_PRIORITY             = 0x0F << 4 = 0xF0   （SysTick/PendSV 用）
```

**规则：任何调用 `...FromISR()` 类 FreeRTOS API 的中断，其 NVIC 优先级数值必须 ≥ 5（即逻辑优先级不高于 5）；数值 0–4 的中断不允许调用任何 FreeRTOS API。**

原因：FreeRTOS 进出临界区时写 `BASEPRI = 0x50`，只屏蔽优先级数值 ≥ 5 的中断。优先级 0–4 的中断能穿透这个屏障，在 kernel 正在改就绪链表/队列时打断它，导致数据结构损坏——现象通常是随机 HardFault，而不是可复现的错误码。`port.c` 的 `vPortValidateInterruptPriority()`（经 `portASSERT_IF_INTERRUPT_PRIORITY_INVALID()`，在 `configASSERT` 编译进来时生效）会在 `FromISR` 调用与 `portYIELD_FROM_ISR` 时做运行时校验；本项目的 `configASSERT` 是 `taskDISABLE_INTERRUPTS() + while(1)`，所以违规的表现是**直接挂死**，不会打印。

### 分配表（按当前工作区代码逐条核对）

| 中断源 | 向量 / IRQ 号 | 抢占 | 子 | 是否调用 FreeRTOS FromISR API | 依据 |
| --- | --- | --- | --- | --- | --- |
| SysTick | `SysTick_IRQn` (-1) | **15** | 0 | 是（`xPortSysTickHandler`，tick 调度） | `port.c` `xPortStartScheduler()` 用 `configKERNEL_INTERRUPT_PRIORITY`(0x0F) 写 SHPR3；必须最低，否则 tick 会抢占内核临界区 |
| PendSV | `PendSV_IRQn` (-2) | **15** | 0 | 是（`xPortPendSVHandler`，上下文切换） | 同上（写 SHPR2/3）；必须是全局最低优先级 |
| SVCall | `SVC_IRQn` (-5) | 未显式配置，使用默认值（SHPR2 复位值 0） | 0 | 仅启动首个任务时进 `vPortSVCHandler`，不调用 `FromISR` 队列/事件 API | `main.c` 只做 `SVC_Handler → vPortSVCHandler` 别名；端口层不配置 SVC 优先级，且调度器启动后不再触发 |
| USART1（RX 空闲/错误） | `USART1_IRQn` (37) | **5** | 0 | **是**：`uart_comm_drain_rx()` → `xStreamBufferSendFromISR()` | `uart_comm.c` `uart_comm_init()` 显式传 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`，即**正好压在红线允许的最高优先级上**——这样 BASEPRI 能屏蔽它，同时它又能最快响应收包 |
| EXTI9_5（PA6/PA7 编码器 A/B 相） | `EXTI9_5_IRQn` (23) | **6** | 0 | 否：只更新 `volatile` 计数/状态，任务侧用 `__disable_irq()` 读 | `rotary_encoder.c` `rotary_encoder_init()` 显式 `6,0`；取 6（≥5）留出与红线 1 级余量，将来在 ISR 里加 `FromISR` 调用也安全 |
| DMA1 Channel 5（USART1_RX） | `DMA1_Channel5_IRQn` (15) | 未显式配置，且 **NVIC 未 EnableIRQ** | — | 否 | `uart_comm.c` 只配 DMA 循环接收（`HAL_UART_Receive_DMA`），HAL 会置 CCR 的 TCIE/TEIE，但没有 `HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn)`，工程里也没有 `DMA1_Channel5_IRQHandler`；收包结束靠 USART IDLE 中断 |
| I2C1（OLED / BH1750 / AHT20 / BMP280） | — | 未显式配置，无中断 | — | 否 | `env_i2c.c` 用阻塞式 `HAL_I2C_Master_Receive(..., timeout)` 轮询，未开 IT/DMA |
| SPI2（ST7789 屏） | — | 未显式配置，无中断 | — | 否 | `tft_st7789.c` 用阻塞式 `HAL_SPI_Transmit(..., 100)`，未开 IT/DMA |
| TIMx（蜂鸣器 / WS2812B） | — | 无中断 | — | 否 | 蜂鸣器是普通 GPIO 电平，WS2812B 是 PB5 位翻转（`ws2812b.c`），都没用定时器中断 |
| GPIO 输入（PIR PB0 / 确认键 PA1 / 返回键 PA4 / 编码器按键） | — | 未显式配置，无中断 | — | 否 | 全部轮询 `HAL_GPIO_ReadPin()` + 软件去抖，没有 EXTI |
| IWDG | — | 无中断（超时直接复位） | — | 否 | `main.c` `iwdg_init()` 纯寄存器配置，约 4.0s 超时 |

补充说明（面试常被追问的两点）：

1. **优先级 0–4 目前完全空闲**：表里最高的用户中断是 USART1 的 5。也就是说，如果将来要加一个"必须在 5µs 内响应、且不调用 FreeRTOS API"的中断（例如 WS2812B 的 DMA 完成），0–4 是留给它的。
2. **最长的临界区不在中断里，而在 `ws2812b.c`**：`ws_send_white()` 用 `__disable_irq()` 关掉全局中断约 **0.5 ms**（15 颗 LED × 24 bit 的位翻转 + 复位脉冲，见该文件注释"a full 15-LED frame occupies about 0.5 ms"）。这段时间里 USART1 的 IDLE 中断最多迟到 0.5 ms——115200 波特下 0.5 ms 只收到约 6 字节，DMA 环形缓冲足够吸收，所以不会丢包；但它是本系统最大的调度抖动源，比任何 NVIC 优先级设置都更影响实时性。

---

## 6. 为什么 HighWaterMark + 最小剩余 heap 比静态 size 报告更有价值

**静态 size 报告回答的是"我预留了多少"，运行时水位线回答的是"实际最深用了多少"。** 两者的差才是真正可用的余量（margin）：

```
栈余量 = app_tasks.h 里预留的 STACK_* (word) − uxTaskGetStackHighWaterMark (word)
堆余量 = configTOTAL_HEAP_SIZE           − xPortGetMinimumEverFreeHeapSize
```

具体价值：

1. **一次最坏情况就会被永久记住**。`uxTaskGetStackHighWaterMark()` 依赖内核在创建任务时把栈填成已知的填充字（`0xA5A5A5A5`），并在任务切换时扫描"未被触碰"的字。它返回的是**历史最小值**，不会因为"你恰好这次采到的是浅调用"而漏掉那次深调用。heap 的 `xPortGetMinimumEverFreeHeapSize()` 同理。当前值（`xPortGetFreeHeapSize()`）会骗人：OTA 状态机、TLS/HTTP（ESP32 侧）、字库绘制这些峰值过去之后，当前值立刻回到"看起来很宽裕"。
2. **区分"编译期猜测"和"运行事实"**。`STACK_APP_TASK = 768` 只是人的估计；`-O2` 下寄存器分配、最深分支（`ui_status_build` + 字库 + `ws2812b` 路径）、以及标准库函数的局部变量都会改变真实用量。水位线是编译产物在真机上跑出来的结果。
3. **水位线给出的是余量，不是占用**，所以能直接支撑决策：余量 ≥ 一半 → 可以调小栈换 RAM（本项目 RAM 很紧，这是真实的收益）；余量 < 32 word → 加栈或减少局部变量。只看"预留了多少"既可能导致过度预留（浪费本就紧张的 20 KB SRAM），也可能在真正栈溢出前毫无预警。
4. **它比溢出钩子更早**。工程开了 `configCHECK_FOR_STACK_OVERFLOW = 2`，但方法 2 只能在**溢出已经发生**之后（切换时发现 canary 被破坏）触发 `vApplicationStackOverflowHook()`，而该 hook 里是 `__disable_irq() + while(1)`——设备已经宕机。水位线是**溢出前的预警**，而且方法 2 本身无法发现"已经用到只剩 8 字节"这种濒临溢出的状态。
5. **让余量变成可观测的时间序列**。每 5s 一次采样 + 一条查询命令，就能把"上电 5 分钟后的余量"接进 ESP32 / 网页：RAM 从 88% 涨到 97% 这类只有运行时才暴露的变化，才会被人发现。静态文档里的数字只在写文档那一刻是真的。

> 证据等级：栈/堆 API 语义为 FreeRTOS 内核实现事实（`tasks.c` / `heap_4.c`）；本节的余量公式为静态分析结论；**实机水位线数值待上电实测**——请勿引用未经采集的具体数字。

---

## 7. 验证记录

```
$ pio run -e app
RAM:   [==========]  97.3% (used 19936 bytes from 20480 bytes)
Flash: [======    ]  60.9% (used 39888 bytes from 65536 bytes)
========================= [SUCCESS] =========================
```

⚠️ 上表是**当前工作区**的构建结果，其中包含同时进行的另一项改动（`uart_comm.c` 由逐字节 RXNE 中断迁移到 DMA1_CH5 + IDLE，缓冲区增加约 1.9 KB RAM）——所以 19,936 B 不能全部算在可观测性特性头上。

只看本特性的增量（在 HEAD 基线副本上单独应用本特性、`pio run -e app -t clean && pio run -e app`，0 warning）：

| 项 | 基线（HEAD） | 加可观测性 | 增量 |
| --- | --- | --- | --- |
| RAM | 18,032 B (88.0%) | 18,072 B (88.2%) | **+40 B** |
| Flash | 38,656 B (59.0%) | 38,928 B (59.4%) | **+272 B** |
| 编译警告 | 0 | 0（`-Wall -Wextra -Werror`） | — |

增量与 §1 的预算表一致（16 B 句柄 + 24 B 缓存）。

## 8. 边界与后续

* 请求 payload 会被忽略（与 `CMD_GET_SENSOR_SNAPSHOT` 的处理方式一致）；未知 payload 长度不会 NAK。
* 缓存是"最近一次 5s 采样"，不是即时值；需要即时值请连续两次请求并比较 `sample_seq`。
* `DiagSnapshot_t` 只增不改：将来加字段请追加在尾部，并同步更新 `sizeof` 静态断言与本文档字段表。
* 未做：ESP32 侧轮询与 dashboard 展示；`configGENERATE_RUN_TIME_STATS`（CPU 占用率）仍然关闭——它会引入一个额外的高频定时器中断，需要单独评估其 RAM/中断预算。
