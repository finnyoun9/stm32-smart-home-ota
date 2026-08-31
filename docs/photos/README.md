# Breadboard photos / 面包板实拍

面包板阶段的实拍留档，配合 [CHANGELOG.md](../../CHANGELOG.md) 的"双屏本地 UI"里程碑一起看。

- `stm32-smart-home-final-wiring.jpg`：最终验收接线实物图（STM32F103、ESP32、ST7789 TFT、SSD1306 OLED、传感器、EC11、继电器与灯带）。
- `breadboard-overview-2026-08-17.jpg`：整体接线（ESP32、STM32、EC11、BH1750、ST7789 TFT、SSD1306 OLED、PIR）。
- `oled-utf8-fix-tft-vs-ssd1306-2026-08-17.jpg`：ST7789 TFT（正确中文）与 SSD1306 OLED（当时未做 UTF-8 解码/位转置，中文显示为乱码）对比实拍——`oled_show_utf8()` 修复前的问题现场，见下方说明。
- `breadboard-dev-progress-1.jpg` / `breadboard-dev-progress-2.jpg`：更早期的面包板搭建过程实拍（ESP32、STM32 Blue Pill、BH1750、AHT20、SSD1306、EC11、HC-SR501、逻辑分析仪接线），非最终装配状态。
