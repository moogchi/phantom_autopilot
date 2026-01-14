# Phantom Autopilot

Real-time quaternion-based flight orientation visualization system for STM32F411 microcontroller.

![Flight Orientation Visualization](Screenshot%20from%202026-01-14%2015-38-48.png)

## Overview

This project captures and visualizes 3D flight orientation data from an STM32F411 microcontroller in real-time. The system outputs quaternion data via UART, which is then visualized as a rotating 3D box using Python and Matplotlib.

## Features

- 📡 **Real-time serial communication** - Reads quaternion data from STM32 via UART
- 📦 **3D visualization** - Displays flight orientation as a color-coded 3D box
- 🎯 **Coordinate system display** - Visual reference axes (X/Y/Z)
- 🔄 **Live updates** - ~20 FPS visualization refresh rate
- 🎨 **Color-coded faces** - Each face has distinct colors for easy orientation tracking

## Hardware Requirements

- STM32F411RETx microcontroller
- USB-to-Serial adapter (or ST-Link with virtual COM port)
- Flight controller or IMU sensor (for quaternion data generation)

## Software Requirements

### STM32 Firmware

- STM32CubeIDE 2.0.0 or later
- STM32F4 HAL Driver
- UART configured for data transmission

### Python Visualization

```bash
pip install pyserial matplotlib numpy
```

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

## Usage

### 1. Flash STM32 Firmware

1. Open the project in STM32CubeIDE
2. Build the project (Ctrl+B)
3. Flash to your STM32F411 board
4. Ensure UART is configured and connected

### 2. Run Visualization

```bash
python quaternion_visual.py
```

The script will:

1. List available serial ports
2. Prompt you to select your device
3. Open a 3D visualization window
4. Display the rotating box based on incoming quaternion data

### 3. Data Format

The STM32 should output quaternion data via UART in CSV format:

```
w, x, y, z
```

Example:

```
0.2454, -0.1136, -0.9061, -0.3252
0.2491, -0.1211, -0.9083, -0.3134
```

**Quaternion format:** `[w, x, y, z]` (scalar-first convention)

## Configuration

### Serial Port Settings

Default baudrate: **115200**

To change the port or baudrate, edit `quaternion_visual.py`:

```python
visualizer = QuaternionVisualizer(port='/dev/ttyUSB0', baudrate=115200)
```

### Box Dimensions

To modify the visualization box size, edit the `create_box_vertices()` function:

```python
w, h, d = 2.0, 0.5, 1.0  # width, height, depth
```

## Visualization Details

### Color Scheme

- **Cyan** - Bottom face
- **Yellow** - Top face
- **Red** - Front face
- **Green** - Back face
- **Blue** - Left face
- **Magenta** - Right face

### Coordinate System

- **Red axis** - X (Forward)
- **Green axis** - Y (Right)
- **Blue axis** - Z (Up)

## Development

### Building the Firmware

```bash
cd /home/sihoon/STM32CubeIDE/workspace_2.0.0/phantom_autopilot
make -C Debug/
```

### Flashing

Use ST-Link or your preferred flashing tool:

```bash
st-flash write Debug/phantom_autopilot.bin 0x8000000
```

## Troubleshooting

### No Serial Ports Found

- Check USB connection
- Verify driver installation (CH340, FTDI, or ST-Link VCP)
- On Linux: ensure user is in `dialout` group
  ```bash
  sudo usermod -a -G dialout $USER
  ```

### Visualization Not Updating

- Verify STM32 is sending data (check with serial monitor)
- Confirm baudrate matches between STM32 and Python script
- Check quaternion data format matches `[w, x, y, z]`

### Permission Denied on Linux

```bash
sudo chmod 666 /dev/ttyUSB0  # or your specific port
```

## License

This project is open source and available for educational and research purposes.

## Acknowledgments

- STM32 HAL Library by STMicroelectronics
- Matplotlib for 3D visualization capabilities
- PySerial for serial communication

## Author

Developed for real-time flight dynamics visualization and autopilot development.

---

**Note:** This project is designed for educational purposes and flight simulation testing. Ensure proper safety measures when using with actual flight hardware.
