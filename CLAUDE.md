# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What is Klipper?

Klipper is a 3D printer firmware that combines the power of a general-purpose computer (like a Raspberry Pi) with one or more microcontrollers. It uses a split architecture where computation-heavy tasks run on the host computer and real-time control happens on the microcontroller.

## Common Development Commands

### Building the Firmware
```bash
# Configure the build (interactive menu)
make menuconfig

# Build the firmware (creates out/klipper.elf or out/klipper.bin)
make

# Clean build artifacts
make clean

# Clean everything including configuration
make distclean

# Update configuration with defaults
make olddefconfig
```

### Host Software Dependencies
```bash
# Install Python dependencies for klippy (host software)
pip install -r scripts/klippy-requirements.txt
```

### Testing
```bash
# Run regression tests
python3 scripts/test_klippy.py test/klippy/*.test

# Run specific test
python3 scripts/test_klippy.py test/klippy/basic.test

# Test with verbose output
python3 scripts/test_klippy.py -v test/klippy/*.test
```

## Architecture Overview

### Split Architecture Design
- **Host Computer** (`klippy/`): Python-based software handling complex calculations, planning, and user interface
- **Microcontroller** (`src/`): C-based firmware for precise real-time stepper control
- **Communication**: Custom binary protocol for clock synchronization and command passing

### Key Directories

#### Microcontroller Firmware (`src/`)
- **Platform directories**: `avr/`, `stm32/`, `rp2040/`, `atsam/`, `atsamd/`, `lpc176x/`, `pru/`, `hc32f460/`, `ar100/`, `linux/`
- **`generic/`**: Shared helper code across architectures  
- **`simulator/`**: Test stubs for cross-platform compilation

#### Host Software (`klippy/`)
- **Core modules**: `gcode.py`, `toolhead.py`, `mcu.py`, `stepper.py`, `clocksync.py`
- **`kinematics/`**: Robot movement implementations (cartesian, delta, corexy, etc.)
- **`extras/`**: 100+ extensible modules (bed_mesh, input_shaper, displays, sensors, etc.)
- **`chelper/`**: C extensions for performance-critical operations

#### Other Important Directories
- **`config/`**: Example printer configuration files for various hardware
- **`test/`**: Automated test cases with configs and test scripts
- **`lib/`**: Third-party dependencies

### Code Flow Patterns

#### Microcontroller Architecture
- Uses `DECL_INIT()` macros for initialization functions
- Uses `DECL_TASK()` macros for recurring tasks  
- Event-driven scheduler with precise timer control
- Hardware abstraction through generic GPIO/timer interfaces

#### Host Software Architecture
- Modular plugin system with dynamic loading based on configuration
- Event reactor pattern for handling asynchronous operations
- Command protocol for MCU communication
- Extensive use of Python's introspection for configuration parsing

### Build System
- Uses Kconfig for flexible hardware configuration (like Linux kernel)
- Cross-compilation support with `CROSS_PREFIX`
- Link Time Optimization (LTO) enabled for size/performance
- Board-specific includes through symbolic linking

### Testing Framework
- Configuration tests for different MCU platforms in `test/configs/`
- Functional tests for host software features in `test/klippy/*.test`
- Main test runner: `scripts/test_klippy.py`
- Tests cover kinematics, G-code processing, hardware features, and calibration routines
- Simple unit tests can be created as standalone Python scripts (e.g., `test_curvature.py`)

### Bed Mesh Curvature Analysis
The bed_mesh module includes curvature analysis to detect bed irregularities:

#### Features
- **Automatic detection**: Runs after BED_MESH_CALIBRATE completes
- **Manual command**: `BED_MESH_CHECK THRESHOLD=0.05` 
- **Configuration**: Set `curvature_threshold` in bed_mesh config section
- **Implementation**: Uses finite differences on interpolated mesh_matrix data
- **Edge handling**: Forward/backward differences for boundary points, central differences for interior

#### API Access for External Applications
Curvature data is exposed through the bed_mesh status for Moonraker, Fluidd, KlipperScreen, etc:

```python
# Access via printer status
status = printer.lookup_object('bed_mesh').get_status()

# Available curvature data:
curvature_x_matrix = status['curvature_x_matrix']    # X-direction curvature (d²z/dx²)
curvature_y_matrix = status['curvature_y_matrix']    # Y-direction curvature (d²z/dy²)  
curvature_warnings = status['curvature_warnings']    # List of high-curvature areas

# Each warning contains:
# {'x': float, 'y': float, 'curvature_x': float, 'curvature_y': float, 'total_curvature': float}
```

#### ZMesh Class Storage
- `curvature_x_matrix`: 2D array of X-direction curvature values (d²z/dx²)
- `curvature_y_matrix`: 2D array of Y-direction curvature values (d²z/dy²)
- `curvature_warnings`: List of detected irregularities with coordinates and curvature values

#### Implementation Details
- **Finite Differences**: Uses 3-point finite difference formulas for curvature calculation
- **Boundary Handling**: Forward/backward differences for edge points, central differences for interior
- **Chinese Comments**: All new code includes detailed Chinese comments for maintainability
- **Automatic Integration**: Curvature calculation runs automatically after mesh generation
- **Configurable Thresholds**: User-configurable sensitivity via `curvature_threshold` parameter