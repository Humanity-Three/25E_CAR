# 25E Hardware Wiring

This project is based on TI MSPM0G3507, package `LQFP-64(PM)`.

The current software focus is line tracking with an 8-channel grayscale sensor. MPU6050 and encoder wiring is reserved, but these functions are not required for the current line-tracking test.

## Power Notes

- Connect all module grounds to the MSPM0 board GND.
- Do not power motors from the MSPM0 3.3 V rail.
- Motor driver logic level should match the board IO level.
- Check whether each external module needs 3.3 V or 5 V before powering it.

## Grayscale Sensor

The grayscale sensor is multiplexed with `AD0/AD1/AD2` and one data output.

| Sensor Pin | MSPM0 Pin | SysConfig Name | Direction |
|---|---:|---|---|
| AD0 | PA14 | `GRAY_SENSOR_AD0` | Output |
| AD1 | PA15 | `GRAY_SENSOR_AD1` | Output |
| AD2 | PA16 | `GRAY_SENSOR_AD2` | Output |
| OUT / DATA | PA17 | `GRAY_SENSOR_DATA` | Input |
| VCC | Sensor power | - | Power |
| GND | GND | - | Ground |

Current code treats high level as black line:

```c
#define GRAYSCALE_BLACK_LEVEL 1U
```

If the car reacts opposite to the track, change it to `0U` in `hardware/Grayscale_Sensor/grayscale_sensor.h`.

## Motor Driver

The code assumes a TB6612-like dual motor driver.

| Driver Pin | MSPM0 Pin | SysConfig Name | Function |
|---|---:|---|---|
| PWMA | PA12 | `MOTOR_PWM_C0` | Left motor PWM |
| PWMB | PA13 | `MOTOR_PWM_C1` | Right motor PWM |
| STBY | PB13 | `MOTOR_STBY` | Driver enable |
| AIN1 | PB15 | `MOTOR_AIN_1` | Left motor direction |
| AIN2 | PB16 | `MOTOR_AIN_2` | Left motor direction |
| BIN1 | PB2 | `MOTOR_BIN_1` | Right motor direction |
| BIN2 | PB3 | `MOTOR_BIN_2` | Right motor direction |
| VM | Motor battery + | - | Motor power |
| VCC | Logic power | - | Logic power |
| GND | Common GND | - | Ground |

If a motor spins in the wrong direction, swap that motor's two output wires or swap its two direction pins in software.

## Keys

The keys are configured as active-low inputs with pull-up resistors.

| Key | MSPM0 Pin | SysConfig Name | Meaning |
|---|---:|---|---|
| KEY1 | PB6 | `KEY_1` | Enter/input line-tracking mode |
| KEY2 | PB7 | `KEY_2` | Confirm/start or stop |

## OLED I2C

| OLED Pin | MSPM0 Pin | SysConfig Name |
|---|---:|---|
| SDA | PA0 | `OLED_I2C_SDA` |
| SCL | PA1 | `OLED_I2C_SCL` |
| VCC | Module power | - |
| GND | GND | - |

I2C instance: `I2C0`.

## Buzzer

| Buzzer Pin | MSPM0 Pin | SysConfig Name |
|---|---:|---|
| Signal | PA18 | `BEEP` |
| VCC/GND | According to module | - |

## Reserved: MPU6050

MPU6050 is reserved for later, but it is not needed for the current line-tracking function.

| MPU6050 Pin | MSPM0 Pin | SysConfig Name |
|---|---:|---|
| SDA | PA10 | `MPU_6050_I2C_SDA` |
| SCL | PA11 | `MPU_6050_I2C_SCL` |
| VCC | Module power | - |
| GND | GND | - |

I2C instance: `I2C1`.

## Reserved: Encoders

Encoders are reserved and are not required for the current line-tracking test.

| Encoder Signal | MSPM0 Pin | SysConfig Name |
|---|---:|---|
| Left B | PA26 | `ENCODER_B1` |
| Left A | PB23 | `ENCODER_A1` |
| Right B | PB24 | `ENCODER_B2` |
| Right A | PB27 | `ENCODER_A2` |

## Reserved: K230 Targeting

K230 targeting is planned but no dedicated K230 communication wiring is finalized in the current SysConfig.

Recommended next step:

- Reserve one UART for K230 communication.
- Connect MSPM0 TX to K230 RX.
- Connect MSPM0 RX to K230 TX.
- Connect GND to GND.
- Confirm IO voltage compatibility before wiring.

## Current Bring-Up Checklist

1. Connect grayscale sensor, motor driver, keys, and common ground.
2. Keep MPU6050 and encoders disconnected if they are not being tested.
3. Build and flash the firmware.
4. Press KEY1 to enter line-tracking input state.
5. Press KEY2 to start line tracking.
6. If steering is reversed, first check `GRAYSCALE_BLACK_LEVEL`, then motor direction wiring.
