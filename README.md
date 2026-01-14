![Uploading image.png…]()

# KLS6030HC STM32 CAN-Bus Reader

This project enables an STM32F407V Discovery board to read data from a KLS6030HC motor driver via CAN-Bus and transmit parsed metrics (RPM, Speed, Current) over UART.

## Features

- **Device Support**: Designed for STM32F407V Discovery and KLS6030HC Motor Driver.
- **CAN Protocol**: Reads Extended CAN frames (29-bit ID) from the motor controller.
  - **Target ID**: `0x0CF11E05`
  - **Baud Rate**: 250 kbps (Default configuration)
- **Data Metrics**:
  - Motor RPM
  - Vehicle Speed (Calculated based on wheel diameter and gear ratio)
  - Motor Current
- **Output**: Transmits formatted data via UART (115200 baud) for monitoring (e.g., via Serial Terminal).

## Hardware Requirements

- **Microcontroller**: STM32F407V Discovery Board
- **Motor Control**: KLS6030HC Motor Driver
- **CAN Transceiver**: 3.3V CAN Transceiver (e.g., SN65HVD230) required between STM32 and Motor Driver.

## Configuration

### Pinout
- **UART2**:
  - `PA2`: TX (Connected to ST-LINK Virtual COM Port)
  - `PA3`: RX
- **CAN1**:
  - Refer to your hardware setup (Commonly `PA11`/`PA12` or `PB8`/`PB9` or `PD0`/`PD1`)
- **LEDs** (Debug):
  - `PD14` (Red): Error Indication

### Software Settings (main.c)
- **Wheel Diameter**: 0.56 Meters (Configurable via `WHEEL_DIAMETER_METERS`)
- **Gear Ratio**: 4.0 (Configurable via `GEAR_RATIO`)
- **CAN ID**: `0x0CF11E05` (KLS Message 1)

## Usage

1. **Connect Hardware**: Setup the CAN bus connections between the STM32, Transceiver, and KLS6030HC.
2. **Flash Firmware**: Compile and upload the code to the STM32F407V.
3. **Monitor**: Open a Serial Terminal connected to the ST-LINK Virtual COM Port.
   - **Baud Rate**: 115200
   - **Data Bits**: 8
   - **Parity**: None
   - **Stop Bits**: 1
4. **Operation**: 
   - The board will print a startup message.
   - Once CAN messages are received from the KLS6030HC, it will stream RPM, Speed, and Current data.

## Note on Baud Rate
The current CAN configuration uses a prescaler of 16 with HSI (16MHz), resulting in **250 kbps**. Ensure your KLS6030HC is configured to match this baud rate, or adjust the `MX_CAN1_Init` function in `main.c`.
