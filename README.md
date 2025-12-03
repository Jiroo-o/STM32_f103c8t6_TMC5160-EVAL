# STM32 F103 + TMC5160-EVAL Wiring Guide

This guide details the pin mapping and wiring connections between the **STM32F103 (Blue Pill)** and the **TMC5160-EVAL** board.

## 🔌 1. Signal Connections (Data & Control)
Connect these pins from the Blue Pill to the **X101** header (the long header) on the TMC5160-EVAL.

| Blue Pill Pin (STM32) | TMC5160-EVAL Pin (X101) | Function | Note |
| :--- | :--- | :--- | :--- |
| **PA4** | Pin 30 (SPI1_CSN) | Chip Select | |
| **PA5** | Pin 31 (SPI1_SCK) | Clock | |
| **PA6** | Pin 33 (SPI1_SDO) | MISO (Data Out) | |
| **PA7** | Pin 32 (SPI1_SDI) | MOSI (Data In) | |
| **PB0** | Pin 8 (DRV_ENN) | Enable Driver | |
| **GND** | Pin 2 (GND) | Ground | **Important:** Common Ground is required. |

---

## ⚡ 2. Power Supply - Critical!

| Power Source | Connection on TMC5160-EVAL | Function |
| :--- | :--- | :--- |
| **5V** (from Blue Pill/USB) | X101 Pin 5 (+5V) | Logic Supply (for 3.3V Regulator) |
| **24V** (from 10A PSU) | X102 (Large Terminal) | Motor Supply (VM) **Do NOT use the small connector!** |
| **Capacitor** (2200uF+) | Across X102 Terminals | Surge Protection (Crucial for NEMA 34) |

---

## 🎛️ 3. Jumper Configuration
You **MUST** configure these jumpers on the **X101** header to enable SPI mode and Internal Clock.

| Pin on X101 | Connect to... | Purpose |
| :--- | :--- | :--- |
| **Pin 22** (SPI_MODE) | 3.3V or 5V | Forces SPI Mode |
| **Pin 21** (SD_MODE) | GND | Disables Standalone Mode |
| **Pin 23** (CLK16) | GND | Selects Internal Clock |

---

🚀 **Double-check all connections before powering on!**