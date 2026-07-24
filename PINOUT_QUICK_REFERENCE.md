# STM32F407VET6 Telemetry System - Quick Reference

## Pin Assignment Summary

### I2C2 - MPU6050 IMU
- **PB10** → I2C2_SCL (400kHz, needs external 4.7kΩ pull-up)
- **PB11** → I2C2_SDA (400kHz, needs external 4.7kΩ pull-up)

### SPI2 - ESP32
- **PC2** → SPI2_MISO
- **PC3** → SPI2_MOSI
- **PB13** → SPI2_SCK
- **PB12** → SPI2_CS (GPIO, Active LOW)

### USART2 - GPS (9600 baud, DMA)
- **PA2** → TX
- **PA3** → RX

### USART3 - Debug (115200 baud)
- **PD8** → TX
- **PD9** → RX

### USB OTG FS - Virtual COM Port
- **PA11** → USB_DM
- **PA12** → USB_DP

### GPIO - LEDs
- **PD12** → Status LED
- **PA6** → Error LED
- **PA7** → Error LED

## Clock Configuration
- **HSE:** 8 MHz
- **System Clock:** 72 MHz
- **USB Clock:** 48 MHz

## Important Notes
⚠️ I2C requires external pull-ups (4.7kΩ)  
⚠️ SPI CS (PB12) must be HIGH when idle  
⚠️ GPS uses DMA on USART2 RX  
⚠️ USB re-enumeration forces PA12 LOW for 100ms at startup

---
For detailed information, see [PINOUT.md](file:///c:/Users/jair3/STM32CubeIDE/workspace_1.17.0/Telemetry/PINOUT.md)
