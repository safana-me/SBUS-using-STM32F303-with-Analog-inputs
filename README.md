
# SBUS Generation from Analog Inputs using STM32

This project demonstrates SBUS signal generation on an STM32 microcontroller based on analog input readings. The firmware is developed using STM32CubeIDE and tested on the STM32F303 microcontroller.

## 🚀 Overview

- Analog inputs (e.g., from joystick or potentiometers) are read using the ADC peripheral.
- The values are scaled to SBUS-compatible channel ranges (1000–2000).
- A custom SBUS frame is generated from up to 16 channel values.
- The SBUS signal is transmitted via UART at 100000 baud with even parity and inverted TX.

## 🛠️ Tools & Technologies

- **Microcontroller**: STM32F303
- **IDE**: STM32CubeIDE
- **Protocol**: SBUS (100000 bps, 8E2, inverted)
- **Communication**: UART
- **Language**: C (HAL-based)


## 📡 SBUS Protocol Notes

- 25-byte frame
- 16 channels (11-bit values)
- 1 start byte (`0x0F`), 22 data bytes, 1 flag byte, 1 end byte (`0x00`)
- Requires inverted UART TX line

## Configuration Settings
UART - baudrate-10000, parity- Even, 8E2
Enable UART Tx inverted
Enable ADC channels

## 🧠 Credits

Developed by **Safana M E**  
Embedded Systems Engineer | STM32, UAVs, RC Systems  
[LinkedIn](https://www.linkedin.com/in/safana-m-e-5326501b9)

---

### 📁 How to Use

1. Open the project in STM32CubeIDE.
2. Flash the firmware to an STM32F303-based board.
3. Connect analog inputs to ADC1 channels.
4. Monitor the SBUS output from USART2 TX pin using an SBUS-compatible receiver or logic analyzer.
