# Pin Connection Guide

## BNO085 Sensor Connections

Connect the BNO085 sensor to the STM32L432KC MCU as follows:

| BNO085 Pin | STM32L432KC Pin | Function | Notes |
|------------|-----------------|----------|-------|
| VIN        | 3.3V            | Power    | Sensor power supply |
| GND        | GND             | Ground   | Common ground |
| SCK        | PB3             | SPI Clock | SPI1_SCK |
| MOSI       | PB5             | SPI Data Out | SPI1_MOSI (MCU to BNO085) |
| MISO       | PB4             | SPI Data In | SPI1_MISO (BNO085 to MCU) |
| CS         | PA11            | Chip Select | Active low, GPIO output |
| PS0/WAKE   | PA12            | Wake Signal | Active low, GPIO output |
| INT        | PA15            | Interrupt | Active low, GPIO input (data ready) |
| RST        | (optional)      | Reset    | Can be left floating or tied to MCU reset |

### SPI Configuration
- **SPI Mode:** Mode 0 (CPOL=0, CPHA=0)
- **SPI Clock:** 1.25MHz (80MHz / 64)
- **CS Control:** Manual (GPIO controlled, not hardware CS)

## Button Connections

| Button Function | STM32L432KC Pin | Connection | Notes |
|-----------------|-----------------|------------|-------|
| Calibration     | PA8             | Button to GND | Pull-up enabled, goes LOW when pressed |
| Kick Drum       | PA10            | Button to GND | Pull-up enabled, goes LOW when pressed |

**Button Wiring:**
- One side of button → MCU pin (PA8 or PA10)
- Other side of button → GND
- MCU has internal pull-up resistors enabled

## Audio Output

| Function | STM32L432KC Pin | Connection | Notes |
|----------|-----------------|------------|-------|
| Audio Out | PA4 | DAC output | Connect to amplifier/speaker via coupling capacitor |

**Audio Wiring:**
- PA4 → 10µF coupling capacitor → Amplifier input
- Amplifier ground → MCU GND

## Power Connections

| Function | Connection | Notes |
|----------|------------|-------|
| MCU VDD   | 3.3V       | Main power supply |
| MCU GND   | GND        | Common ground |
| BNO085 VIN | 3.3V      | Same power supply as MCU |
| BNO085 GND | GND        | Same ground as MCU |

## Complete Connection Diagram

```
BNO085 Sensor:
┌─────────────┐
│   VIN  ─────┼──→ 3.3V
│   GND  ─────┼──→ GND
│   SCK  ─────┼──→ PB3 (SPI1_SCK)
│   MOSI ─────┼──→ PB5 (SPI1_MOSI)
│   MISO ─────┼──→ PB4 (SPI1_MISO)
│   CS   ─────┼──→ PA11 (GPIO output)
│   PS0  ─────┼──→ PA12 (GPIO output)
│   INT  ─────┼──→ PA15 (GPIO input)
│   RST  ─────┼──→ (optional, can float)
└─────────────┘

Buttons:
Calibration Button:
  ┌───┐
  │   │──→ PA8 (pull-up enabled)
  └───┘
    │
   GND

Kick Button:
  ┌───┐
  │   │──→ PA10 (pull-up enabled)
  └───┘
    │
   GND

Audio Output:
PA4 ──[10µF]──→ Amplifier Input
                (Amplifier GND → MCU GND)
```

## Pin Summary Table

| MCU Pin | Function | Direction | Connected To |
|---------|----------|-----------|--------------|
| PB3     | SPI1_SCK | Output    | BNO085 SCK |
| PB5     | SPI1_MOSI | Output   | BNO085 MOSI |
| PB4     | SPI1_MISO | Input    | BNO085 MISO |
| PA11    | CS (BNO085) | Output | BNO085 CS |
| PA12    | PS0/WAKE | Output    | BNO085 PS0/WAKE |
| PA15    | INT (BNO085) | Input | BNO085 INT |
| PA8     | Calibration Button | Input | Button (to GND) |
| PA10    | Kick Button | Input | Button (to GND) |
| PA4     | DAC Output | Output | Audio amplifier |

## Important Notes

1. **SPI Pins:** PB3, PB4, PB5 are configured as alternate function (SPI1) in the code
2. **CS Pin:** PA11 is manually controlled as GPIO, not hardware CS
3. **Pull-ups:** Buttons use internal pull-up resistors (configured in code)
4. **Power:** Both MCU and BNO085 need 3.3V power supply
5. **Ground:** All GND connections must be common
6. **BNO085 RST:** Can be left floating or connected to MCU reset for synchronized reset

## Changing Pin Assignments

If you need to change pin assignments, modify these locations:

1. **SPI Pins:** `mcu2/STM32L432KC_SPI.h` (lines 12-15)
2. **BNO085 Control Pins:** `mcu2/main.c` (lines 35-37)
3. **Button Pins:** `mcu2/main.c` (lines 29-30)

Then update the pin configuration in `main.c` where `BNO085_CS_PIN`, `BNO085_PS0_WAKE_PIN`, and `BNO085_INT_PIN` are assigned.

