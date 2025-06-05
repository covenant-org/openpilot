# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Quick Start Commands

### Setup and Build
```bash
# Initial setup (installs dependencies and sets up environment)
./tools/op.sh setup

# Build the entire project
./tools/op.sh build

# Build specific target
scons -j$(nproc) selfdrive/controls/controlsd

# Clean build
scons -c && scons -j$(nproc)
```

### Testing
```bash
# Run all tests
./tools/op.sh test

# Run specific test file
python -m pytest selfdrive/test/test_controlsd.py -v

# Run tests with coverage
pytest --cov=selfdrive selfdrive/test/

# Hardware-in-the-loop testing
cd tools/sim && python run_bridge.py
```

### Development Tools
```bash
# Code linting and formatting
./tools/op.sh lint

# Type checking
mypy selfdrive/

# Start development environment
source .venv/bin/activate

# Replay a drive for debugging
cd tools/replay && python replay.py --route <route_name>
```

## Architecture Overview

### Core System Design
openpilot is a **process-based real-time system** with ~40+ independent daemons managed by a central process manager. Communication happens via **message passing** using Cap'n Proto serialization and msgq/ZMQ transport.

### Key Process Categories

**Control & Planning (100Hz)**:
- `controlsd`: Main vehicle control loop, lateral/longitudinal control
- `plannerd`: Path planning and trajectory generation
- `radard`: Radar processing and object tracking

**Perception (20Hz)**:
- `modeld`: Neural network model inference for driving
- `dmonitoringd`: Driver monitoring system
- `camerad`: Camera capture and processing

**Hardware Interfaces**:
- `pandad`: CAN bus communication via Panda device
- `sensord`: IMU and sensor data processing
- `locationd`: GPS and localization

**System Services**:
- `manager`: Process lifecycle management
- `loggerd`: Data logging and storage
- `athenad`: Cloud connectivity and OTA updates

### Message System
- **cereal/**: Cap'n Proto schema definitions for all inter-process messages
- **msgq/**: High-performance message queue implementation
- Messages flow through named channels (e.g., "carState", "controlsState", "modelV2")
- Real-time constraints enforced with 100Hz control loops

### Directory Structure

**selfdrive/**: Core driving algorithms and control logic
- `controls/`: Vehicle control systems (steering, acceleration, braking)
- `locationd/`: Localization, calibration, and sensor fusion
- `modeld/`: Neural network model execution
- `car/`: Vehicle-specific interfaces and parameters

**system/**: System services and hardware abstraction
- `manager/`: Process management and configuration
- `camerad/`: Camera drivers and image processing
- `loggerd/`: Data recording and compression

**tools/**: Development utilities
- `replay/`: Drive replay system for debugging
- `sim/`: Simulation environment
- `cabana/`: CAN message viewer and DBC editor

**third_party/**: External dependencies (OpenCL, Qt, etc.)

## Development Workflow

### Code Organization
- Python code follows PEP 8 with type hints (mypy enforced)
- C++ code uses modern C++17 standards
- Real-time code paths avoid dynamic allocation
- Safety-critical functions have comprehensive tests

### Testing Strategy
- **Unit tests**: pytest for Python components
- **Integration tests**: Hardware-in-the-loop with real/simulated CAN
- **Replay testing**: Validate changes against recorded drives
- **Performance tests**: Ensure real-time constraints are met

### Build System (SCons)
- Multi-platform support: TICI (larch64), PC (x86_64), macOS
- Incremental builds with dependency tracking
- Debug/release configurations with sanitizer support
- Cross-compilation for embedded targets

### Safety Considerations
- **Panda safety**: Hardware-enforced CAN message validation
- **Watchdog systems**: Process monitoring and automatic restart
- **Graceful degradation**: System continues operating with component failures
- **Real-time constraints**: 100Hz control loop timing is critical

## Fork-Specific Notes (Drone/UAV)

This appears to be a drone/UAV fork with significant modifications:

### Key Additions
- **Mavlink integration**: Communication with flight controllers via MAVSDK
- **Altitude control**: Z-axis control for drone operations
- **Remote camera**: Gazebo simulation and remote camera streaming
- **Modified safety**: Drone-specific safety constraints and parameters

### Modified Components
- `pandad/`: Extended with Mavlink fake panda for drone communication
- `car/`: Drone-specific vehicle parameters and control logic
- `selfdrive/controls/`: Adapted for 3D movement and hover control
- **DBC files**: Custom `drone.dbc` for drone-specific CAN messages

### Simulation Environment
- Gazebo integration for hardware-in-the-loop testing
- Remote camera streaming via WebRTC
- Custom launch scripts for drone simulation scenarios

## Model and Data Handling

### Neural Networks
- Models stored in Git LFS (`.thneed` format for SNPE acceleration)
- Model metadata in `selfdrive/modeld/models/`
- GPU acceleration via OpenCL/SNPE on supported hardware

### Logging and Replay
- **qlogs**: Compressed logs with all system messages
- **segments**: 1-minute chunks for efficient storage/processing
- **comma connect**: Cloud storage and route sharing (if configured)

## Performance Notes

- **Memory management**: Shared memory pools for camera frames
- **CPU affinity**: Critical processes pinned to specific cores
- **Thermal management**: Automatic throttling on overheating
- **Storage optimization**: Log compression and automatic cleanup