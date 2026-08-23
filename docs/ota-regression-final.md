# Final Application OTA Regression Record

> 自动生成/更新：`py -3.11 tools/ota_regression.py ...`
> 生成时间：2026-08-23T14:13:42+08:00

## 固件基线

| 项目 | 值 |
|---|---|
| Git commit | `6987f309645a70a2bb63a73b271aa35acf07fb7b` |
| 工作树 | dirty（含本次未提交改动） |
| Application image | `.pio/build/app/firmware.bin` |
| 镜像大小 | **39268 bytes** / 55296 bytes (71.0%) |
| IEEE CRC-32 | **`0x671CBE6C`** |
| Initial SP | `0x20005000` |
| Reset vector | `0x08009C61` |
| RAM | 18032 / 20480 bytes (88.0%) |
| Flash | 38916 / 65536 bytes (59.4%) |
| Protocol smoke test | PASS |

镜像已通过与 ESP32 相同的入口校验：非空、≤54 KiB、SP 位于 STM32F103 20 KiB RAM、Thumb Reset Vector 位于 `0x08002000..0x0800F800`。

## 已核对的当前 OTA 流程

1. Bluetooth：`FW <version,size,crc>` → 带 offset ACK 的 Base64 `DATA` → `VERIFY` 回读 size/CRC → `SEND`。
2. Web：`POST /api/upload?version=N` 流式写 SPIFFS → 校验 size/CRC/SP/Reset Vector → `POST /api/start`。
3. 公共下半程：ESP32 发 `CMD_OTA_AVAILABLE` → Application 写 Flash 配置并回 `CMD_OTA_READY` → reset → Bootloader 收 `OTA_BEGIN/CHUNK/END`、擦写并校验整镜像 CRC → 标记有效并回跳 Application。
4. 最终判定：入口报告 complete 还不够；脚本会再发 `VERSION`，只有返回目标版本才记 PASS。Web 未传 `--verify-port` 时只记 PARTIAL。

## 回归结果

| 路径 | 目标版本 | 自动结果 |
|---|---:|---|
| Bluetooth SPP → ESP32 SPIFFS → STM32 | 3 | PASS (2026-08-23T14:19:52+08:00)<br>OTA complete; VERSION=3 |
| Web HTTP → ESP32 SPIFFS → STM32 | 4 | PASS (2026-08-23T14:20:59+08:00)<br>HTTP OTA complete; ESP32 size=39268, CRC=0x671CBE6C; VERSION=4; sensors online; relay1=false, relay2=false, buzzer=false |

## 实机验收记录

- [ ] OTA 前确认 Relay 1、Relay 2、buzzer 均处于安全状态，雾化驱动板保持断开。
- [x] Bluetooth 回归完成后，Application 正常回跳，`VERSION` = `3`。
- [x] Web 回归完成后，Application 正常回跳，`VERSION` = `4`。
- [x] OLED/TFT 正常刷新，无 HardFault、无反复复位。（用户实机确认，2026-08-23）
- [x] `/api/sensors` 返回 `online=true`，数据持续刷新。
- [ ] Relay 2 / 灯带、buzzer 的控制状态符合现场预期；不为测试主动接通未知负载。
- [ ] 若失败，保存 ESP32 USB log、页面/脚本输出、供电电压和复现步骤。

## 实机执行命令（完成后可复现）

```powershell
# 1. 蓝牙：Windows 已配对 STM32-OTA-Bridge，确认实际 outgoing COM 口后执行
py -3.11 tools/ota_regression.py bluetooth --port COM6

# 2. Web：电脑连接 ESP32 SoftAP 后执行；--verify-port 会在 OTA 后自动核对版本
py -3.11 tools/ota_regression.py web --base-url http://192.168.4.1 --verify-port COM6
```

如果 Web 回归坚持用 iPhone 页面上传，选择下面这份同一镜像，并填写版本 `4`：

`build/ota-regression/application-39268B-671CBE6C.bin`

页面显示完成后，再运行：

```powershell
py -3.11 tools/ota_regression.py verify-version --port COM6 --expected 4 --transport web
```
