# Delto TCP Communication Library

[![CI](https://github.com/tesollodelto/dg_tcp_comm/actions/workflows/ci.yml/badge.svg)](https://github.com/tesollodelto/dg_tcp_comm/actions/workflows/ci.yml)
![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros)

A unified TCP communication library for DELTO grippers (DG-3F, DG-4F, DG-5F).

## Supported Models

| Model ID | Model Name | Motors | Bytes/Motor | GPIO | Sensor |
|----------|------------|--------|-------------|------|--------|
| 0x3F01 | DG-3F B | 12 | 5 | Yes | -- |
| 0x3F02 | DG-3F M | 12 | 8 | Yes | F/T, TACTILE_M |
| 0x4F02 | DG-4F | 18 | 8 | Yes | F/T, TACTILE_M |
| 0x5F12 | DG-5F Left | 20 | 8 | Yes | F/T, TACTILE_M |
| 0x5F22 | DG-5F Right | 20 | 8 | Yes | F/T, TACTILE_M |
| 0x5F14 | DG-5F-S Left | 20 | 8 | -- | F/T, TACTILE_S |
| 0x5F24 | DG-5F-S Right | 20 | 8 | -- | F/T, TACTILE_S |
| 0x5F34 | DG-5F-S15 Left | 15 | 8 | -- | TACTILE_S |
| 0x5F44 | DG-5F-S15 Right | 15 | 8 | -- | TACTILE_S |

## Sensor Types

The sensor type is auto-detected from firmware via `GET_VERSION`:

| Type | Value | Description | Data per finger |
|------|-------|-------------|-----------------|
| `NONE` | 0x00 | No sensor | -- |
| `FT_6AXIS` | 0x01 | 6-axis force/torque | 12 bytes |
| `FT_3AXIS` | 0x02 | 3-axis force/torque | 12 bytes |
| `TACTILE_M` | 0x03 | Tactile matrix 3x5 | 15 bytes (uint8) |
| `FT_4AXIS` | 0x04 | 4-axis force/torque | 12 bytes |
| `TACTILE_S` | 0x05 | Tactile matrix 3x6 | 36 bytes (uint16) |

## Usage

### Basic Example

```cpp
#include "delto_tcp_comm/delto_developer_TCP.hpp"

// Connect to DG-5F (with GPIO and F/T sensor enabled)
DeltoTCP::Communication comm("169.254.186.72", 502, 0x5F12, true, true);
comm.Connect();

// Read data
auto data = comm.GetData();
for (size_t i = 0; i < data.joint.size(); i++) {
    std::cout << "Joint " << i << ": " << data.joint[i] << " rad" << std::endl;
}

// Send duty command
std::vector<int> duty(20, 0);
comm.SendDuty(duty);

// GPIO control
comm.SetGPIO(true, false, false);

// Set F/T sensor offset (zero calibration)
comm.SetFTSensorOffset();

comm.Disconnect();
```

### Adding as Dependency

`package.xml`:
```xml
<depend>delto_tcp_comm</depend>
```

`CMakeLists.txt`:
```cmake
find_package(delto_tcp_comm REQUIRED)

target_link_libraries(your_target
  delto_tcp_comm::delto_tcp_comm
)
```

## Protocol

### Motor Data (8 bytes/motor - New Models)
| Offset | Size | Description |
|--------|------|-------------|
| 0 | 1 | Motor ID |
| 1-2 | 2 | Position (0.1°) |
| 3-4 | 2 | Current (mA) |
| 5-6 | 2 | Temperature (0.1°C) |
| 7 | 1 | Velocity (1 RPM) |

### Motor Data (5 bytes/motor - Legacy DG3F-B)
| Offset | Size | Description |
|--------|------|-------------|
| 0 | 1 | Motor ID |
| 1-2 | 2 | Position (0.1°) |
| 3-4 | 2 | Current (mA) |

### F/T Sensor Data (12 bytes/finger)
- 6 axes x 2 bytes per finger
- Force (Fx, Fy, Fz): 0.1 N units
- Torque (Tx, Ty, Tz): 1 mNm units

### Tactile M Data (15 bytes/finger)
- 3x5 grid, 1 byte per cell (uint8)

### Tactile S Data (36 bytes/finger)
- 3x6 grid, 2 bytes per cell (uint16)

### GPIO Data (4 bytes)
- Output 1-3, Input 1
- Level: 0 = 0V, 1 = 24V

## Build

```bash
cd ~/your_ws
colcon build --packages-select delto_tcp_comm
```
