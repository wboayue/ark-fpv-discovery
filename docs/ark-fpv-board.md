# ARK FPV — Board Layout Reference

Pin map for the [ARK FPV](https://arkelectron.com/product/ark-fpv/) flight controller
(STM32H743VIH6, Cortex-M7). Compiled from the ArduPilot and Betaflight hardware
definitions — see [Sources](#sources). Verify against your own board revision before
trusting any single line; the two upstreams occasionally disagree (noted inline).

## MCU

| | |
|---|---|
| Part | STM32H743VIH6 (Cortex-M7) |
| Flash | 2 MB internal @ `0x08000000` |
| RAM | 128 KB (AXI/region used by `memory.x`) |
| SYSCLK | 400 MHz (this firmware) |

## Sensors

| Sensor | Part | Bus | Address / CS | Interrupt (DRDY) |
|---|---|---|---|---|
| IMU (gyro+accel) | **IIM-42653** | SPI1 | CS `PI9` | `PF2` (EXTI) |
| Barometer | **BMP388/BMP390** | I2C2 (internal) | `0x76` | — |
| Magnetometer | **IIS2MDC / LIS2MDL** | I2C4 (internal) | `0x1E` | — |
| Aux IMU (optional) | ADIS16507 | SPI6 (external) | DRDY `PD11` | — |

- IMU SPI clock range: 2 MHz – 16 MHz.
- BMP388 and BMP390 are register-compatible; ArduPilot's comment says BMP390, its driver line says BMP388. Same `0x76` address either way.
- IIS2MDC (ArduPilot) and LIS2MDL (Betaflight) are the same ST magnetometer family at `0x1E`.
- **Mounting orientation differs by firmware convention:** ArduPilot `ROTATION_YAW_270`, Betaflight `CW90_DEG_FLIP` (mag `CW180_DEG`). Determine empirically for this firmware.

## SPI buses

| Bus | SCK | MISO | MOSI | Devices |
|---|---|---|---|---|
| SPI1 | `PA5` | `PG9` | `PB5` | IIM-42653 IMU (CS `PI9`) |
| SPI6 | `PB3` | `PA6` | `PG14` | external connector (ADIS16507, CS via `PI10`) |

## I2C buses

| Bus | SCL | SDA | Use |
|---|---|---|---|
| I2C1 | `PB8` | `PB9` | external — GPS module / external compass |
| I2C2 | `PF1` | `PF0` | internal barometer (BMP388/390 @ `0x76`) |
| I2C4 | `PF14` | `PF15` | internal magnetometer (IIS2MDC/LIS2MDL @ `0x1E`) |

## LEDs

| Color | Pin |
|---|---|
| Red | `PE3` |
| Green | `PE4` |
| Blue | `PE5` |

**Active-low** (confirmed on hardware): driving the pin LOW lights the LED, HIGH turns it off.

## Beeper / buzzer

| Function | Pin | Timer |
|---|---|---|
| Beeper | `PF9` | TIM14_CH1 |

## UART / serial ports

| Port | TX | RX | Flow control | Typical use |
|---|---|---|---|---|
| USART1 | `PB6` | `PB7` | — | GPS |
| USART2 | `PD5` | `PA3` | — | VTX (RX used for DJI) |
| USART3 | `PD8` | `PD9` | — | debug / MSP (Betaflight) |
| UART4 | `PH13` | `PH14` | — | ESC telemetry |
| UART5 | `PC12` | `PD2` | — | VTX / DJI Air Unit |
| USART6 | `PC6` | `PC7` | — | RC input |
| UART7 | `PE8` | `PF6` | RTS `PF8`, CTS `PE10` | telemetry |

(ArduPilot does not expose USART3; otherwise the two sources agree on these pins.)

## ADC — power monitoring

| Channel | Pin | Notes |
|---|---|---|
| Battery voltage | `PB0` | Betaflight VBAT scale 210 |
| Battery current | `PC2` | Betaflight current scale 120 |
| External 3V3 sense | `PA0` | Betaflight only |
| External 5V sense | `PB1` | Betaflight only |
| External 12V sense | `PA4` | Betaflight only |

## Motor / PWM outputs

| Output | Pin | Timer | Notes |
|---|---|---|---|
| M1 | `PI0` | TIM5_CH4 | bidirectional-capable (M1–M4) |
| M2 | `PH12` | TIM5_CH3 | |
| M3 | `PH11` | TIM5_CH2 | |
| M4 | `PH10` | TIM5_CH1 | |
| M5 | `PI5` | TIM8_CH1 | |
| M6 | `PI6` | TIM8_CH2 | |
| M7 | `PI7` | TIM8_CH3 | |
| M8 | `PI2` | TIM8_CH4 | |
| Aux/LED strip | `PD12` | TIM4_CH1 | ArduPilot PWM(9) |

## USB

USB CDC (this firmware): OTG_HS on `PA11` (DM) / `PA12` (DP), clocked from HSI48.
See `src/main.rs`.

## Sources

- ArduPilot: [`libraries/AP_HAL_ChibiOS/hwdef/ARK_FPV/hwdef.dat`](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/ARK_FPV/hwdef.dat)
- Betaflight: [`configs/ARK_FPV/config.h`](https://github.com/betaflight/config/blob/master/configs/ARK_FPV/config.h)
