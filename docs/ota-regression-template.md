# Application OTA Regression Template

复制本模板用于手工补充记录。推荐直接运行 `py -3.11 tools/ota_regression.py prepare`，脚本会构建镜像、校验 Cortex-M 向量、计算大小/CRC，并生成已填值的 `docs/ota-regression-final.md`。

## 固件基线

| 项目 | 值 |
|---|---|
| 日期 / 操作者 | |
| Git commit / 工作树状态 | |
| Application image | |
| 镜像大小 / 54 KiB | |
| IEEE CRC-32 | |
| Initial SP / Reset vector | |
| RAM / Flash | |
| Protocol smoke test | PASS / FAIL |

## 路径结果

| 检查点 | Bluetooth | Web |
|---|---|---|
| 目标版本 | | |
| ESP32 暂存 size/CRC 一致 | PASS / FAIL | PASS / FAIL |
| Application 回 `CMD_OTA_READY` | PASS / FAIL | PASS / FAIL |
| Bootloader 分块 ACK、Flash/CRC | PASS / FAIL | PASS / FAIL |
| ESP32 报告 OTA complete | PASS / FAIL | PASS / FAIL |
| 重启后 `VERSION` 命中目标版本 | PASS / FAIL | PASS / FAIL |
| OLED/TFT、看门狗、传感器刷新 | PASS / FAIL | PASS / FAIL |
| 日志 / 失败现象 | | |

## 现场安全与证据

- [ ] 雾化驱动板断开；未知负载不主动通电。
- [ ] OTA 前后记录 Relay 1、Relay 2、buzzer 状态。
- [ ] 保存 ESP32 USB log 或脚本完整输出。
- [ ] Web 路径记录页面截图或 `/api/status` 最终 JSON。
- [ ] 失败时记录电源电压、串口、发生阶段、能否由 Bootloader/Application 恢复。
