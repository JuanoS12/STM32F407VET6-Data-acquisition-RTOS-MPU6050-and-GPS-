# STM32F407VET6-Data-acquisition-RTOS-MPU6050-and-GPS-
# STM32F407 Telemetry System

## Documentation

This document provides comprehensive technical documentation for a production-ready telemetry system based on the STM32F407VETx microcontroller. The system implements a fault-tolerant, scalable architecture using FreeRTOS for real-time task management, collecting data from multiple sensors (IMU, GPS) and transmitting telemetry via SPI.

This project has comprehensive documentation in multiple formats:

### 📄 LaTeX Documentation (Primary)
**File:** [`PROJECT_DOCUMENTATION.tex`](PROJECT_DOCUMENTATION.tex)

This is the **primary technical documentation** that must be kept up-to-date with all changes.

**Section 2: Hardware Configuration** contains:
- Complete pinout tables for all peripherals
- Clock configuration details
- External connection diagrams
- Power requirements

> [!IMPORTANT]
> **Whenever you make hardware or configuration changes, update Section 2 of the LaTeX documentation!**

### 📌 Quick Reference Files

- **[PINOUT.md](PINOUT.md)** - Complete pinout documentation with visual diagram
- **[PINOUT_QUICK_REFERENCE.md](PINOUT_QUICK_REFERENCE.md)** - Quick lookup reference

### 🔧 Current Hardware Configuration

**Microcontroller:** STM32F407VET6 (LQFP100)
- **Core:** ARM Cortex-M4F @ 72 MHz
- **Flash:** 512 KB
- **RAM:** 128 KB

**Clock:**
- HSE: 8 MHz → System: 72 MHz, USB: 48 MHz

**Peripherals:**
- **I2C2** (PB10/PB11): MPU6050 IMU @ 400kHz
- **SPI2** (PC2/PC3/PB13/PB12): ESP32 Communication
- **USART2** (PA2/PA3): GPS Module @ 9600 baud (DMA)
- **USART3** (PD8/PD9): Debug UART @ 115200 baud
- **USB OTG FS** (PA11/PA12): Virtual COM Port
- **GPIO** (PD12, PA6, PA7): Status/Error LEDs

## Project Structure

```
Telemetry/
├── Core/
│   ├── Inc/                    # Header files
│   ├── Src/                    # Source files
│       ├── main.c             # Main program & clock config
│       ├── freertos.c         # FreeRTOS tasks
│       ├── gpio.c             # GPIO configuration
│       ├── i2c.c              # I2C configuration
│       ├── spi.c              # SPI configuration
│       ├── usart.c            # UART configuration
│       └── ...
│── PROJECT_DOCUMENTATION.tex  # Complete technical docs
├── Drivers/                    # STM32 HAL drivers
├── Middlewares/                # FreeRTOS
├── USB_DEVICE/                 # USB CDC implementation
├── PINOUT.md                   # Pinout documentation
├── PINOUT_QUICK_REFERENCE.md   # Quick pinout reference
└── README.md                   # This file
```

## Quick Start

1. **Hardware Setup:**
   - Connect MPU6050 to I2C2 (PB10/PB11) with 4.7kΩ pull-ups
   - Connect GPS to USART2 (PA2/PA3)
   - Connect ESP32 to SPI2 (PC2/PC3/PB13/PB12)
   - Connect USB-to-TTL adapter to USART3 (PD8/PD9) for debug output

2. **Build & Flash:**
   - Open project in STM32CubeIDE
   - Build project
   - Flash to STM32F407VET6

3. **Monitor:**
   - Connect to USART3 @ 115200 baud for debug output
   - Watch LED on PD12 for system status

## Features

- **Real-time OS:** FreeRTOS with 6 concurrent tasks
- **Sensor Fusion:** Complementary filter for IMU angles
- **Velocity Estimation:** Kalman filter combining GPS + IMU
- **Fault Tolerance:** Graceful degradation, task watchdog
- **High-speed Telemetry:** SPI @ 2.25 MHz to ESP32
- **Debug Output:** UART @ 115200 baud

## Important Notes

⚠️ **I2C Pull-ups Required:** External 4.7kΩ resistors on PB10 and PB11
⚠️ **SPI CS Control:** PB12 must be HIGH when idle
⚠️ **USB Re-enumeration:** PA12 driven LOW for 100ms at startup
⚠️ **GPS DMA:** USART2 uses DMA for efficient reception

## Documentation Updates

When making changes to the project:

1. ✅ Update code files
2. ✅ Update `PROJECT_DOCUMENTATION.tex` (Section 2 for hardware)
3. ✅ Update `PINOUT.md` if pin assignments change
4. ✅ Rebuild LaTeX documentation to generate PDF

---

**Last Updated:** 2025-11-30
**Version:** 1.0
