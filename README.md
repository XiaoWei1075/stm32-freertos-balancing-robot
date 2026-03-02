[![Build Status](https://img.shields.io/badge/build-passing-lightgrey)](https://example.com/build)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://example.com/license)
[![Platform: STM32F1](https://img.shields.io/badge/platform-STM32F1-009688)](https://example.com/platform)
[![Language: C](https://img.shields.io/badge/language-C-A8B9CC)](https://example.com/language)

## Project Description
STM32F1-based dual-wheel self-balancing robot firmware using FreeRTOS (CMSIS-RTOS v2). Cascaded PID (outer speed/position, inner angle) stabilizes the robot while a Kalman filter estimates attitude from an I2C IMU. PID gains can be tuned at runtime over a USART Bluetooth link.

## Features
- Kalman filter attitude estimation (angle + gyro bias fusion)
- Cascaded PID loops (inner angle, outer speed/position)
- FreeRTOS task-based architecture via CMSIS-RTOS v2
- Runtime tuning and control over Bluetooth UART (HC-06)
- STM32 HAL peripheral drivers (I2C, USART, timers/PWM, GPIO)

## Hardware Requirements
| Component | Example Part | Interface |
|---|---|---|
| MCU | STM32F103 | — |
| IMU | BMI160 | I2C |
| Motor driver | `TB6612` | PWM + GPIO |
| Motors | `<DC_GEAR_MOTORS>` | PWM |
| Bluetooth module | HC-06 | USART |
| Power supply | `<MOTOR_SUPPLY_VOLTAGE> for motors, <MCU_SUPPLY_VOLTAGE> for MCU` | — |

## Software Requirements
- STM32CubeMX `v6.15.0`
- Keil MDK-ARM uVision 5
- STM32 HAL (CubeF1)
- FreeRTOS + CMSIS-RTOS v2 (CubeMX middleware)

## Installation & Build
1. Clone the repo:
   ```bash
   git clone https://github.com/XiaoWei1075/stm32-freertos-balancing-robot.git
   cd stm32-freertos-balancing-robot
   ```
2. Open the CubeMX project: `<REPO_ROOT>/<PROJECT>.ioc`.
3. In CubeMX, set **Project Manager → Toolchain/IDE** to **MDK-ARM**.
4. Generate code (CubeMX generates `Core/`, `Drivers/`, `Middlewares/`, and the Keil project).
5. Open the Keil project: `<REPO_ROOT>/<PROJECT>.uvprojx`.
6. Connect the target via ST-Link and power the board.
7. In uVision 5, select **Options for Target → Debug → ST-Link Debugger**.
8. Build and flash: **Project → Build Target**, then **Flash → Download**.

## Project Structure
```text
.
├─ Algorithm/                 # Balance control, filters, Bluetooth shell
│  ├─ control.c/.h            # imuTask/pidTask/shellTask + cascaded control
│  └─ filter.c/.h             # Kalman + complementary filters
├─ Core/                      # Startup, main(), interrupt handlers
│  ├─ Inc/                    # Application headers
│  └─ Src/                    # Application sources
├─ Drivers/                   # STM32 HAL + CMSIS
│  └─ BMI160_Core/            # Bosch BMI160 driver + STM32 I2C porting wrapper
├─ Middlewares/               # Third-party middleware
│  └─ FreeRTOS/               # FreeRTOS kernel + CMSIS-RTOS wrapper
└─ MDK-ARM/                   # Keil uVision project (.uvprojx)
```

## Configuration
| Parameter | Location (file + line hint) | Default | Description |
|---|---|---:|---|
| `Ve_Kp`, `Ve_Ki` | `Algorithm/control.c` (search `Ve_Kp`) | `2.087`, `0.015` | Velocity-loop PI gains |
| `balance_Kp`, `balance_Kd` | `Algorithm/control.c` (search `balance_Kp`) | `565.4866776`, `30.1327412` | Balance-loop PD gains |
| `turn_Kp`, `turn_Kd` | `Algorithm/control.c` (search `turn_Kp`) | `17.45329`, `1.5` | Turning (gyro-z rate) PD gains |
| `yaw_Kp`, `yaw_Kd` | `Algorithm/control.c` (search `yaw_Kp`) | `350.0`, `7.0` | Yaw-hold PD gains |
| `vc_to_ta_stand`, `vc_to_ta_move` | `Algorithm/control.c` (search `vc_to_ta_`) | `0.2`, `0.3` | Velocity→target-angle coupling factors |
| `MAX_PWM` | `Algorithm/control.c` (search `MAX_PWM`) | `7000` | PWM output clamp |
| `PWM_FREQ_HZ` | `BalancedRobot.ioc` (search `TIM3.Period`) | `<DERIVED_FROM_TIMER>` | Motor PWM frequency (TIM3 configuration) |
| `BT_USART_BAUD` | `BalancedRobot.ioc` (search `USART3.BaudRate`) | `9600` | Bluetooth UART baud rate (USART3) |

## Usage / Bluetooth Tuning
- Connect to the Bluetooth module (`HC-06`) and send commands.
- Protocol: `[CMD][VALUE];` over USART3; whitespace is ignored; multiple commands can be concatenated.

`<CMD>` is a single character; `<VALUE>` is optional for non-parameter commands.

| Command | Example | Effect |
|---|---|---|
| Start motors | `t;` | Enables motor standby and starts control |
| Stop motors | `s;` | Disables motor standby |
| Zero targets | `z;` | Sets target velocity and turn rate to 0 |
| Set target velocity | `v1.0;` | Sets target linear velocity (sign selects direction) |
| Set target turn rate | `g-30;` | Sets target yaw rate in deg/s |
| Velocity-loop Kp | `02.1;` | Sets `Ve_Kp` |
| Velocity-loop Ki | `10.02;` | Sets `Ve_Ki` |
| Balance-loop Kp | `2565;` | Sets `balance_Kp` |
| Balance-loop Kd | `330;` | Sets `balance_Kd` |
| Turn-loop Kp | `417.5;` | Sets `turn_Kp` |
| Turn-loop Kd | `51.2;` | Sets `turn_Kd` |
| Yaw-hold Kp | `6350;` | Sets `yaw_Kp` |
| Yaw-hold Kd | `77;` | Sets `yaw_Kd` |
| Print status | `p;` | Prints pitch/yaw/speed and all gains over UART |

## API Reference
| Function | File | Description |
|---|---|---|
| `Kalman_Filter_x(float Accel, float Gyro, float dt)` | `Algorithm/filter.c` | Pitch 1D Kalman filter (returns angle in deg) |
| `Kalman_Filter_y(float Accel, float Gyro, float dt)` | `Algorithm/filter.c` | Roll 1D Kalman filter (returns angle in deg) |
| `Complementary_Filter_Yaw(float Yaw, float encoder_z_rate, float gyro_z_rate, float dt)` | `Algorithm/filter.c` | Yaw complementary filter (encoder + gyro) |
| `imuTask(void *param)` | `Algorithm/control.c` | IMU + encoder sampling task; waits BMI160 data-ready IRQ |
| `pidTask(void *param)` | `Algorithm/control.c` | Control task; computes balance/velocity/steering and drives PWM |
| `shellTask(void *param)` | `Algorithm/control.c` | Bluetooth shell; parses `[CMD][VALUE];` commands |
| `BMI160_init(void)` | `Drivers/BMI160_Core/Src/bmi160_wrapper.c` | Initialize BMI160 (I2C) and enable data-ready interrupt |
| `bmi160ReadAccelGyro(BMI160_t *DataStruct, uint8_t ZeroVelocityUpdate)` | `Drivers/BMI160_Core/Src/bmi160_wrapper.c` | Read accel/gyro and update bias when stationary |
| `Set_PWM(int PWM1, int PWM2)` | `Core/Src/tim.c` | Apply left/right motor PWM outputs |

## Contributing
- Create a fork and branch from `main` (`feature/<name>` or `fix/<name>`).
- Use conventional commits where possible (e.g., `feat:`, `fix:`, `refactor:`).
- Keep PRs small and focused; include test/bench notes (logs/screenshots) when behavior changes.
- Follow existing naming/style; prefer STM32 HAL conventions and CMSIS types.

## License
MIT — see `LICENSE`. [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://example.com/license)
