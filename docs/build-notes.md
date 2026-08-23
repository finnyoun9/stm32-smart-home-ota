# Build Notes — 2026-08-11

> 三端固件均能通过 PlatformIO 构建；OTA、本地环境终端、继电器、蜂鸣器和 WS2812B 自动调光均已完成实机验证。

## 三端编译结果

| 固件 | 工具链 | RAM | Flash | 产物 |
|------|--------|-----|-------|------|
| Bootloader | `pio run -e bluepill` | 11.0% (2260B) | 10.7% (7040B) | `.pio/build/bluepill/firmware.bin` |
| Application | `pio run -e app` | 87.4% (17904B) | 55.4% (36320B) | `.pio/build/app/firmware.bin` |
| ESP32 Bridge | `pio run -d esp32-comm-bridge` | 19.9% (65184B) | 74.9% (1374661B / 1835008B) | `esp32-comm-bridge/.pio/build/esp32dev/firmware.bin` |

- Bootloader 产物约 6.9KB，满足 **8KB 硬约束**
- Application 仍在 54KB 应用区内；RAM 仅余 2,580B，后续增加任务或大缓冲区前必须检查 heap/stack 余量
- ESP32 开 Bluedroid 后固件 ~1.5MB，factory 分区必须 ≥ 1.5MB

## 硬件 Bring-up（2026-08-09）

- ESP32 CH340 串口为 `COM4`，STM32 通过 ST-Link SWD 烧录；两端写后校验均已通过。
- 链路接线：`GPIO17 → PA10`、`GPIO16 ← PA9`、GND 直连；STM32 使用 USART1，双方暂定 9600 baud。
- Windows Bluetooth Classic SPP 建立后，`COM6` 可发送 `STATUS` 与 `VERSION`；连续 10 次 `VERSION` 均返回 `FW Version: 0`。
- 使用 `tools/bridge_ota.py COM6 .pio/build/app/firmware.bin --version 1` 推送 15,956 字节镜像（CRC-32 `0x3B274D7E`），ESP32 返回 `STATUS: OTA complete!`。
- OTA 完成并重启后，`VERSION` 返回 `FW Version: 1`，证明暂存、Bootloader 切换、Flash 擦写、CRC 校验和应用回跳均已跑通。

### ST-Link 与 IWDG（2026-08-21）

- Application 的 IWDG 使用约 4 秒超时，且在 `NVIC_SystemReset()` 后持续运行；SWD 烧录时 CPU 被 halt，IWDG 仍会计时并在完整 Application 写入过程中复位目标，OpenOCD 表现为 `external reset detected` 或 Flash algorithm abort。
- `tools/stlink_upload.py` 会在烧录前经 SWD 将 IWDG 临时延长至约 26 秒，再执行写入和 verify。不要把该报错先判定为 NRST 短路或板卡损坏。
- 如果 SWD 无法在 IWDG 超时前 halt，给 Blue Pill 完整断电再上电后立即烧录；刷完后的 Application 会自行恢复约 4 秒看门狗配置。

## Application 编译要点

原来 `application/` 从未编译过（缺 FreeRTOS 内核、HAL、时钟配置），本次补全：

1. **Vendor FreeRTOS**：从 `framework-stm32cubef1/Middlewares/Third_Party/FreeRTOS/Source` 复制到 `application/lib/FreeRTOS/`（内核 + `GCC/ARM_CM3` 移植 + `heap_4`），根 `platformio.ini` 加 `[env:app]`
2. **64MHz 时钟**：实物板已确认 8MHz HSE；`main.c` 与 bootloader 均使用 HSE→PLL×8。初次 OTA 闭环使用 9600 baud，当前 Application、Bootloader 和 ESP32 已统一为 USART1 115200 baud
3. **FreeRTOS 必踩的坑**：
   - `configSUPPORT_STATIC_ALLOCATION=1` 时必须实现 `vApplicationGetIdleTaskMemory` / `vApplicationGetTimerTaskMemory`
   - 启动文件向量表用 `SVC_Handler`/`PendSV_Handler`，FreeRTOS 移植用 `vPortSVCHandler`/`xPortPendSVHandler` → 加薄包装函数
   - `SysTick_Handler` 里要调 `xPortSysTickHandler()`
4. **heap 8KB→12KB**：5 个动态任务栈合计 ~10KB，8KB heap 在调度器启动时直接触发 malloc-failed hook 挂死
5. **链接脚本**：`._user_heap_stack` 保留区从 8K 降到 1K（`heap_4` 用自己的静态数组，8K 保留区是纯浪费，会把 RAM 顶爆）

## STM32 本地环境终端（2026-08-10）

- I2C1 使用 `PB6/PB7 @ 100kHz`，实机共挂 SSD1306 `0x3C`、BH1750 `0x23`、AHT20 `0x38` 和 BMP280 `0x76/0x77`。
- AHT20 与 BMP280 位于同一块组合模块；当前代码因此把两者放在 I2C1，而不是目标架构里预留的 I2C2。
- AHT20 检查 CRC-8；BMP280 读取 24 字节工厂校准参数并使用整数补偿，显示单位为 °C、`% RH` 和 hPa。
- EC11 A/B 接 PA6/PA7，使用双边沿 EXTI + Gray-code 状态表，完整四状态回到卡点后才输出一次 ±1；独立确认按键接 PA1。
- HC-SR501 OUT 接 PB0，页面在上电后先显示 30 秒 `WARMUP`，随后显示 `DETECTED`/`CLEAR`。
- OLED、旋转选择、确认/返回、光照、人体感应、温湿度和气压均已完成面包板验证。
- 固件打开 `-Wall -Wextra -Werror`；本次构建无 warning。STM32F103 无 FPU，显示换算继续采用整数/定点运算。
- OLED 现在固定显示温湿度、气压、光照、灯带亮度、PIR、灯带电源和固件版本；TFT 独立显示五项卡片菜单与详情页。页面缓存只重画当前页相关数据，未使用全屏帧缓冲。
- 新增 PA4 低电平返回键（内部上拉，按键另一端接 GND）；PA1 继续作为确认键，在灯带详情页直接切换 Relay 2 / NO2。

## 继电器与蜂鸣器（2026-08-11）

- 两路继电器从与 TFT 冲突的 PB12/PB13 改到 `PA2/PA3`；有源蜂鸣器接 `PB1`。三路均为低电平触发，初始化先输出高电平，避免上电误动作。
- 继电器模块控制侧接 DP100 5V、IN1/IN2 和公共 GND；负载侧 `COM/NO/NC` 是无源开关触点，`COM` 不会自己产生电压。需要把被控电源送入 `COM`，再由 `NO` 输出到默认断开的负载。
- 蓝牙命令支持 `RELAY1/RELAY2 ON|OFF`、`RELAY`、`AUTO ON|OFF`、`MANUAL` 和 `BUZZER ON|OFF`。继电器吸合声、触点导通、共地和蜂鸣器动作均已实机确认。
- 当前通道映射：**Relay 1 / PA2 暂未使用；Relay 2 / PA3 / NO2 = 灯带 VCC**。加湿器已移除，自动湿度联动强制关闭；网页通过 `{"light":true|false}` 控制 Relay 2，灯带亮度仍由 BH1750 自动映射。
- 2026-08-12 当前版本已烧录：STM32 ST-Link Verify OK，ESP32 各分区 Hash 校验通过。被动回读为 `RELAY1: OFF, RELAY2: OFF, AUTO: OFF`；新版页面已包含 `light → RELAY2` 和 `NO2` 标识。按用户要求，ESP32 更新后未再次主动切换继电器。
- 继电器初始化已提前到传感器/显示屏之前，减少复位窗口里的输入瞬态。这是防误动作措施，不是对之前持续咔咔响的根因认定；当时雾化驱动板未接雾化片上电后损坏，损坏负载/供电异常才是现场背景。

## ESP32 编译要点（ESP-IDF 6.0.1）

### 1. API 迁移（IDF 5→6 变了）

| 旧 API | IDF 6 |
|--------|-------|
| `esp_spp_init(ESP_SPP_MODE_CB)` | `esp_spp_enhanced_init(&cfg)`，`cfg.mode = ESP_SPP_MODE_CB` |
| `esp_bt_dev_set_device_name()` | `esp_bt_gap_set_device_name()`（`esp_gap_bt_api.h`） |
| `esp_wifi_is_connected()` | `esp_wifi_sta_get_ap_info() == ESP_OK` |

### 2. 蓝牙组件必须用 sdkconfig 开

`-D CONFIG_BT_SPP_ENABLED=1` 这类编译宏对 ESP-IDF **无效**（组件选择靠 Kconfig）。必须加 `sdkconfig.defaults`：

```
CONFIG_BT_ENABLED=y
CONFIG_BT_BLUEDROID_ENABLED=y
CONFIG_BT_CLASSIC_ENABLED=y
CONFIG_BT_SPP_ENABLED=y
CONFIG_BT_CONTROLLER_ENABLED=y
CONFIG_BT_BLE_ENABLED=n
CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY=y  # Classic-only SPP；释放 BLE RAM
```

### 3. Flash 实测是 4MB

2026-08-12 在下载模式执行 `esptool flash_id`，读取 JEDEC ID `c4:6016`，工具确认 **4MB**。此前 2MB 配置是误判，现已修正：
- `platformio.ini`：`board_upload.flash_size = 4MB`
- `sdkconfig.defaults`：`CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y`
- `partitions.csv`：factory 保持 `0x1C0000`（1.75MB），storage(SPIFFS) 扩为 `0x230000`（2.19MB），总边界到 `0x400000`

### 4. C/C++ 混编 flag 冲突 → protocol.cpp

PlatformIO 的 ESP-IDF 主组件里 `.c` 和 `.cpp` 并存时，会把 C 专用 flag（`-Wno-old-style-declaration`）和 C++ 专用 flag（`-fuse-cxa-atexit`）互相泄漏，双双报错。解法：把共享协议源码复制为 `src/protocol.cpp`（按 C++ 编译），并让 `shared/protocol.c` 兼容 C++：
- `proto_parser_feed` 的 `FRAME_STATE_CRC3` case 体加大括号（否则 C++ 报 `jump to case label`）
- `shared/protocol.c` 与 `esp32-comm-bridge/src/protocol.cpp` **字节一致**，改动时保持同步

### 5. ESP-IDF 忽略 build_src_filter

PlatformIO 的 ESP-IDF 框架用 CMake 管理源文件（`src/CMakeLists.txt` 里 `FILE(GLOB_RECURSE ...)`），`build_src_filter` 无效。加源文件就放进 `src/` 目录。

### 6. 国内网络需要代理

PlatformIO 官方包镜像（contabostorage / github release）在国内直连会被 SSL 掐断，导致装工具链卡住无限重试。编译前设代理：

```powershell
$env:HTTPS_PROXY='http://127.0.0.1:7897'   # 本机代理端口
pio run -d esp32-comm-bridge
```

2026-08-12 灯带三项控制和 TFT 精简中文包加入后，Application Flash 增加约 3.1KB，仍有约 28KB 余量；中文只收录菜单实际字形，没有引入完整 CJK 字库或 framebuffer。

## 蓝牙协议（实现现状）

服务名 `STM32-OTA-Bridge`。命令：`STATUS` / `VERSION` / `OTA <url>` / `FW <ver>,<size>,<crc32>` / `DATA <offset>,<base64>` / `VERIFY` / `SEND` / `WIFI <ssid>,<pass>` / `RESET`。

蓝牙推固件的正确流程（`FW` + Base64 `DATA` + `VERIFY` + `SEND`）：
1. PC 发 `FW <version>,<size>,<crc32hex>` —— 声明精确大小、版本和标准 IEEE CRC-32，ESP32 清空旧暂存区
2. PC 把固件拆成不超过 128 字节的块，发送 `DATA <offset>,<base64>`；每块等待 `DATA: ACK <next_offset>`，超时可安全重发
3. 全部分块完成后发送 `VERIFY`；ESP32 同时检查接收过程 CRC 和 SPIFFS 文件回读 CRC
4. 等 ESP32 返回 `FW: staged ...` 后发送 `SEND`
5. ESP32 向应用发 `CMD_OTA_AVAILABLE`，等待 `CMD_OTA_READY` 确认配置页已写好，再开始 Bootloader OTA

> 早期直接传二进制的方案受 SPP 字节流分包和终端路径影响，现改用可打印 Base64 分块。`tools/bridge_ota.py` 会自动完成编码、ACK 重试、`VERIFY` 和 `SEND`。

### 7. CRC 回归测试不能只用经典向量

`shared/protocol.c` 和 ESP32 的 `protocol.cpp` 曾各有 4 个 CRC 查表常量抄错，但 `123456789 → 0xCBF43926` 仍然通过，因为该输入没有访问错误表项。现已：

- 修正两份表中的 4 个常量，并用脚本核对 256/256 表项
- 在 `tools/protocol_smoke_test.c` 增加 `0x00..0xFF → 0x29058C73` 全字节向量
- 用真实 15,956 字节固件的 `0x3B274D7E` 完成 ESP32 暂存和 STM32 Bootloader 双端 CRC 验证

## 手机 Web OTA（2026-08-09）

- ESP32 使用 `WIFI_MODE_APSTA`，常驻 SoftAP：SSID `STM32-OTA-Bridge`，默认密码 `stm32ota`，入口 `http://192.168.4.1`
- `web_ota_page.h` 存在 ESP32 Application 的只读段，不占 SPIFFS；SPIFFS 可完整用于 54KB STM32 固件暂存
- API：`GET /api/status`、`POST /api/upload?version=<N>`、`POST /api/start`
- 上传采用固定 1KB 缓冲区流式写文件；检查大小、CRC、初始 SP、Thumb Reset Vector 和 Application 地址范围
- Web/蓝牙传输共用 FreeRTOS Mutex；真正的 STM32 OTA 在独立 8KB 栈任务中执行，HTTP 状态查询不会被 115200-baud 写入过程阻塞
- 实机结果：iPhone 上传 15,956 字节 `fw_v2.bin`，页面完成升级；COM6 查询得到 `FW Version: 2`

## Web 实时仪表盘（2026-08-11）

- `vAppTask` 每 200ms 把当前传感器和执行器状态复制到 18B `SensorSnapshot_t`；`vCommTask` 收到 `CMD_GET_SENSOR_SNAPSHOT (0x32)` 后返回 `CMD_SENSOR_SNAPSHOT_RSP (0x87)`。
- ESP32 新增 `sensor_poll` 任务，每 1 秒查询一次 STM32 并写入 Mutex 保护的静态缓存；`GET /api/sensors` 只读取缓存，不让浏览器请求直接占用 UART。
- 传感器轮询、蓝牙 `VERSION`/继电器命令和 OTA 共用 UART 事务锁。OTA 持锁期间轮询跳过；缓存超过 5 秒未更新时页面显示数据超时。
- 页面改成首页/控制/系统三页：首页实时显示温湿度、气压、光照、PIR、继电器、蜂鸣器和灯带亮度；系统页保留原 Web OTA；控制页已接入灯带电源和蜂鸣器写接口。
- 协议烟测加入 18B 快照帧 round-trip，用 CRC 帧构建与解析验证结构体字节不变。三目标编译无 warning；新版 STM32 Application 已由 ST-Link 写入并 Verify OK，ESP32 固件烧录后 Flash hash 校验通过。ESP32 接入 2.4GHz Wi-Fi 后，`GET /api/sensors` 连续返回真实数据，`online=true`、`age_ms<1000`，板载页面按秒刷新通过实机回归。
- 完整协议字段、REST JSON 和断线检查步骤见 [web-realtime-dashboard.md](web-realtime-dashboard.md)。

## 下一步

- [x] 实机烧录 Bootloader / Application / ESP32，并验证 Bluetooth SPP 到 STM32 的双向命令链路
- [x] 用已构建的 `app/firmware.bin` 验证端到端 OTA（暂存、触发重启、分块、CRC、回跳）
- [x] 用 iPhone 直连 ESP32 SoftAP，验证无需 PC 发包工具的 Web OTA
- [x] 完成第一批本地外设：SSD1306、BH1750、AHT20、BMP280、HC-SR501、EC11
- [x] 完成 PA2/PA3 两路继电器、PB1 蜂鸣器和 PB5 WS2812B 自动调光实机闭环
- [x] 最终 Application 的 Bluetooth/Web OTA 回归均通过；构建、版本回读与记录已由 `tools/ota_regression.py` 自动化，镜像数据和实机结果见 `docs/ota-regression-final.md`
- [x] 定义 STM32→ESP32 传感器快照协议并提供 `/api/sensors` 实时状态页
- [ ] 增加 `/ws` 推送与 Web 写控制接口
- [ ] Pi5 继续使用现有 Mosquitto，接入 Node-RED + FlowFuse Dashboard 2.0，做可远程访问的实时曲线与状态卡片
- [ ] 更换损坏的雾化驱动板；接回前先确认必须带雾化片/水位负载上电的要求，再重新设计加湿器通道

## GMT020-02 双屏显示（2026-08-10）

- 新增 ST7789V 240×320 SPI2 驱动：`PB13=SCL`、`PB15=SDA(MOSI)`、`PB12=CS`、`PB14=DC`、`PA8=RST`。
- SSD1306 继续保留在 I2C1；`ui_display` 把原 16×4 OLED 页面同步绘制到两块屏幕。
- TFT 使用竖屏、RGB565、16MHz SPI，以 12×24 字符居中绘制完整 16 列界面，没有申请 153,600 字节的整屏 framebuffer。
- Application 构建通过且无 warning：RAM `17,772 / 20,480 bytes (86.8%)`，Flash `28,996 / 65,536 bytes (44.2%)`。

## WS2812B 光照联动（2026-08-11）

- 从 `D:\Project\ws2812b-stm32` 复用经过流水灯实机验证的 15 灯驱动；PA7 已被 EC11 占用，因此把 GPIO bit-bang 数据线改到 `PB5`。灯带固定白光，只由 BH1750 调节亮度。
- 供电为 DP100 5V 经 1N4001 降到约 4.2V，灯带和 STM32 共地；`PB5 → DIN`。入口建议串 220~470Ω 数据电阻并并联 470~1000µF 电容，但本次无响应并不是没串数据电阻造成的。
- BH1750 每 200ms 更新目标亮度：`≤5 lux → 160/255`、`≥1000 lux → 1/255`，中间反向线性映射；每次最多变化 16 级，2 级死区过滤小幅传感器抖动。WS2812B 会保持锁存状态，所以只有亮度变化时才发帧，减少约 0.5ms 关中断窗口对 UART/FreeRTOS 的影响。
- 15 颗灯全白、亮度 160 时按每颗满白 60mA 的保守上限估算约 0.57A；DP100 限流建议设在 0.8~1.0A，并留意串联 1N4001 的温升。
- 当时灯带闭环版本的 Application 构建占用为 RAM `17,780 / 20,480 bytes (86.8%)`、Flash `30,892 / 65,536 bytes (47.1%)`；加入 Web 实时快照后的当前值见本文顶部构建表。

### 这次排障为什么重要

刚开始把问题归到亮度算法，因为 BH1750 数值正常，但把白光固定到 `1/255` 后灯带还是保持高亮。逻辑分析仪移到 `PB5/DIN` 后，旧 SPI 方案能看到 3 帧：每帧 360 bit、约 449.7µs，周期约 51.0ms，`T0H≈0.50µs`、`T1H≈0.75µs`，并且能解码出 45 字节 `0x01`。单看 MCU 输出，这个结果很容易被判断为“驱动没问题”。

真正的分界点是把探头移到第一颗灯珠的 `DOUT`。SPI 输入存在时，DOUT 没有有效转发，说明第一颗灯珠实际上没有接收；换成 64MHz GPIO bit-bang 后，灯带立刻进入接近熄灭的 `1/255` 状态，DOUT 又捕获到 336 bit，也就是后 14 颗的 42 字节 `0x01`。这才证明 `DIN → 第一颗灯珠 → DOUT → 后续灯珠` 全链路成立。

结论不是“逻辑分析仪波形对就够了”，而是单线级联器件要在两个位置验证：

1. `DIN` 验证 MCU 是否真的把数据送到了灯带入口；
2. 第一颗 `DOUT` 验证器件是否接受并转发了数据。

保留的对照证据：[`spi-din-not-accepted.vcd`](captures/ws2812/evidence/spi-din-not-accepted.vcd) 是旧 SPI 方案的 DIN 输入，[`bitbang-dout-confirmed.vcd`](captures/ws2812/evidence/bitbang-dout-confirmed.vcd) 是最终 bit-bang 方案在第一、第二颗之间的 DOUT 输出。其余 UART 和 WS2812B 中间抓包统一收在 [`docs/captures/`](captures/README.md)。

### PB0 Bootloader 冲突修复

- 自定义 Bootloader 原先把 `PB0` 作为软件强制维护输入，但 HC-SR501 也使用 `PB0`，且其空闲输出为低电平，导致每次上电都停在 Bootloader。
- 已移除 Bootloader 对 `PB0` 的读取，`PB0` 现专用于 PIR；OTA 入口保留配置标志和 200ms UART 窗口。
- 救援方式仍然完整：Blue Pill 独立的硬件 `BOOT0` 跳帽可进入 STM32 ROM Bootloader，ST-Link/SWD 也可直接重刷。
