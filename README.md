# Phantom Autopilot

Hardware-accelerated flight control system using Extended Kalman Filter for real-time attitude estimation on STM32F411 microcontroller.

![Flight Orientation Visualization](Screenshot%20from%202026-01-14%2015-38-48.png)

## Overview

This project implements a real-time quaternion-based Extended Kalman Filter (EKF) for flight attitude estimation and control. The STM32F411 processes IMU data from FlightGear simulator, computes optimal orientation estimates, and outputs control commands to stabilize the aircraft.

**For detailed technical documentation, debugging challenges, and implementation notes, see [DEVLOG.md](DEVLOG.md).**

## Key Features

- Custom Extended Kalman Filter implementation with quaternion-based attitude representation
- Circular DMA for low-latency sensor data streaming
- Proportional control for aileron and elevator surfaces
- Real-time 3D visualization using Python and Matplotlib
- Non-blocking UART communication architecture
- Gimbal lock avoidance through quaternion mathematics

## System Architecture

### Hardware

- **Microcontroller:** STM32F411RETx (Cortex-M4F, 100MHz, 512KB Flash)
- **Debugger:** ST-Link V2 with JTAG
- **Communication:** Dual UART (USART1: debug telemetry, USART2: FlightGear interface)

### Software Stack

- **IDE:** STM32CubeIDE 2.0.0
- **HAL:** STM32F4xx Hardware Abstraction Layer
- **Algorithms:** Extended Kalman Filter, quaternion kinematics, PID control
- **Simulator:** FlightGear (IMU data source)

## Project Structure

```
phantom_autopilot/
├── Core/
│   ├── Inc/                    # Header files
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── stm32f4xx_it.h
│   ├── Src/                    # Source files
│   │   ├── main.c              # Main application code
│   │   ├── stm32f4xx_hal_msp.c
│   │   ├── stm32f4xx_it.c
│   │   └── system_stm32f4xx.c
│   └── Startup/
│       └── startup_stm32f411retx.s
├── Drivers/                    # STM32 HAL drivers
├── Debug/                      # Build output
├── quaternion_visual.py        # Python visualization script
├── phantom_autopilot.ioc      # STM32CubeMX configuration
└── README.md
```

## Quick Start

### 1. Build and Flash Firmware

```bash
cd /home/sihoon/STM32CubeIDE/workspace_2.0.0/phantom_autopilot
make -C Debug/
st-flash write Debug/phantom_autopilot.bin 0x8000000
```

Or use STM32CubeIDE:

1. Open project
2. Build (Ctrl+B)
3. Run/Debug (F11)

### 2. Run FlightGear Simulator

The STM32 expects sensor data in CSV format via UART:

```
sim_time, ax, ay, az, gx, gy, gz\n
```

### 3. Visualize Orientation (Optional)

```bash
pip install pyserial matplotlib numpy
python quaternion_visual.py
```

## Technical Documentation

This README provides a quick overview. For in-depth technical details, see:

**[DEVLOG.md](DEVLOG.md)** - Engineering notebook with:

- Detailed problem analysis and solutions
- EKF implementation details
- DMA architecture and optimization
- Debugging methodologies
- Performance tuning strategies

## Data Protocol

### Input (USART2 RX)

CSV format from FlightGear:

```
0.025, 0.0, 0.0, -9.81, 0.01, -0.02, 0.00
```

Fields: `sim_time, ax, ay, az, gx, gy, gz`

### Output (USART2 TX)

Control commands:

```
aileron, elevator\n
```

Range: [-1.0, 1.0]

### Debug (USART1 TX)

Telemetry data:

```
qw, qi, qj, qk, ax, ay, az, gx, gy, gz, sim_time, dt\n
```

## Project Structure

```
phantom_autopilot/
├── Core/
│   ├── Inc/                    # Header files
│   │   ├── main.h             # Application definitions
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── stm32f4xx_it.h     # Interrupt handlers
│   ├── Src/                    # Source files
│   │   ├── main.c              # EKF, control loops, UART
│   │   ├── stm32f4xx_it.c      # DMA/UART ISRs
│   │   └── system_stm32f4xx.c
│   └── Startup/
├── Drivers/                    # STM32 HAL drivers
├── Debug/                      # Build artifacts
├── quaternion_visual.py        # Python visualization
├── phantom_autopilot.ioc       # CubeMX config
├── DEVLOG.md                   # Engineering notebook
└── README.md
```

## Configuration

### UART Settings

- **Baudrate:** 115200
- **Data bits:** 8
- **Stop bits:** 1
- **Parity:** None
- **Flow control:** None

### EKF Parameters

Tunable in `main.c`:

- Process noise (Q)
- Measurement noise (R)
- Initial covariance (P)

### Control Gains

PID gains stored in `state->PID` array (currently P-only control).

## Troubleshooting

### Common Issues

**Problem:** No serial communication  
**Solution:** Check UART connections, verify baudrate, ensure ST-Link virtual COM port is enabled

**Problem:** EKF diverges to NaN  
**Solution:** Check dt calculation, verify quaternion normalization, ensure sensor data is valid

**Problem:** Control lag  
**Solution:** Verify circular DMA is configured, check for blocking operations in main loop

**Problem:** Permission denied (Linux)  
**Solution:** `sudo usermod -a -G dialout $USER` then logout/login

For detailed debugging strategies, see [DEVLOG.md](DEVLOG.md).

## References

- Madgwick, S. O. H. (2010). "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
- STM32F4xx Reference Manual (RM0383)
- ARM Cortex-M4 Technical Reference Manual

## License

Open source - available for educational and research purposes.

---

**Status:** Active development - Performance optimization phase  
**Last Updated:** January 22, 2026
