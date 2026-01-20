# Laser Diode Test Equipment Simulation Environment

A high-fidelity Python-based simulation environment for laser diode test automation equipment featuring precision motion control, 3D visualization, and comprehensive safety systems.

## Features

### Core Simulation
- **6-Axis Stage Controller**: Precision positioning with X, Y, Z, Rx, Ry, Rz control
- **3-Axis Gantry System**: Automated chip loading and positioning
- **Probe Stage Controller**: Controlled probe press/unpress with force monitoring
- **S-Curve Motion Planning**: Smooth, jerk-limited trajectory generation
- **Micron-Level Precision**: 0.1 micron positioning accuracy

### Safety & Collision Detection
- **Real-Time Collision Detection**: Predictive safety interlocks
- **Configurable Safety Zones**: Restricted area enforcement
- **Emergency Stop System**: Immediate halt on safety violations
- **Force Limiting**: Probe force protection and monitoring

### User Interface
- **3D Visualization**: Real-time stage position display with OpenGL/PyVista
- **Multi-Stage Control**: Tabbed interface for individual stage control
- **Command Editor**: JSON-based command creation with validation
- **System Monitoring**: Live status, warnings, and performance metrics
- **Position Trails**: Visual tracking of stage movement history

### Command System
- **JSON Command Interface**: Professional Modbus-like command structure
- **Command Queue Management**: Priority-based execution scheduling
- **Batch Command Processing**: Automated test sequence execution

## Installation

### Prerequisites
- Python 3.8 or higher
- Qt5 compatible display system (Linux: X11, Windows: Win32, macOS: Cocoa)

### Quick Install

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd motion_control_simulation
   ```

2. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Run the application**:
   ```bash
   python main.py
   ```

### Optional: 3D Visualization

For the enhanced 3D visualization experience, install PyVista:
```bash
pip install pyvista pyvistaqt vtk
```

If PyVista is not available, the application will automatically fall back to a 2D display mode.

## Usage

### Basic Operation

1. **Start the application**:
   ```bash
   python main.py
   ```

2. **Home all stages**:
   - Go to Tools → Home All Stages, or
   - Use the EMERGENCY STOP reset procedure, or
   - Send a HOME command via the command panel

3. **Move stages**:
   - Use the position controls in the left panel
   - Enter target coordinates and click "Move"
   - Adjust velocity/acceleration as needed

4. **Probe operations**:
   - Position probe above target
   - Set target Z position and force limit
   - Click "Press Probe" to engage
   - Use "Retract" to withdraw probe

5. **Chip handling**:
   - Position gantry at chip source
   - Click "Load Chip" to pick up chip
   - Move to destination position
   - Click "Unload Chip" to release

### JSON Commands

#### Move Command
```json
{
  "command_id": "ST0001",
  "command_type": "MOVE",
  "target_stage": "six_axis",
  "parameters": {
    "position": {
      "position": {
        "x": 10500,
        "y": 5200,
        "z": 100,
        "rotation": {
          "rx": 0.0,
          "ry": 0.0,
          "rz": 45.0
        }
      },
      "velocity": 50000,
      "acceleration": 500000
    },
    "execution": {
      "mode": "immediate",
      "priority": 5,
      "timeout": 30.0
    }
  }
}
```

#### Probe Press Command
```json
{
  "command_id": "PR0001",
  "command_type": "PRESS_PROBE",
  "target_stage": "probe_stage",
  "parameters": {
    "target_z": -5000,
    "max_force": 800
  },
  "execution": {
    "mode": "queued",
    "priority": 7
  }
}
```

#### Home Command
```json
{
  "command_id": "HM0001",
  "command_type": "HOME",
  "target_stage": null,
  "execution": {
    "mode": "immediate"
  }
}
```

### Command Line Options

```bash
# Run with custom configuration
python main.py --config my_config.json

# Run in headless mode (for testing)
python main.py --headless

# Enable debug logging
python main.py --debug

# Show version
python main.py --version
```

## Configuration

### Stage Parameters

Default motion parameters can be adjusted via the GUI or configuration files:

```json
{
  "stages": {
    "six_axis": {
      "max_velocity": 100000,
      "max_acceleration": 500000,
      "max_jerk": 10000000,
      "position_tolerance": 0.1,
      "velocity_tolerance": 10.0
    }
  }
}
```

### Safety Settings

```json
{
  "safety": {
    "min_clearance": 1000.0,
    "prediction_time": 2.0,
    "auto_emergency_stop": true,
    "warning_distance": 2000.0,
    "force_limit_probe": 1000.0
  }
}
```

## Development

### Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=core --cov=gui

# Run GUI tests
pytest tests/test_gui.py
```

### Code Style

```bash
# Format code
black .

# Lint code
flake8 .

# Type checking
mypy .
```

### Project Structure

```
motion_control_simulation/
├── core/                   # Core simulation engine
│   ├── data_structures.py  # 3D vectors, positions, states
│   ├── motion_controller.py # Base motion control
│   ├── stage_controllers.py # Specific stage implementations
│   ├── system_controller.py # Main system orchestrator
│   └── collision_detector.py # Safety collision detection
├── gui/                    # User interface
│   ├── main_window.py      # Main application window
│   ├── visualization.py    # 3D visualization widget
│   ├── control_panels.py   # Stage control panels
│   └── main_window.py      # Main application window
├── config/                 # Configuration management
│   ├── command_schema.json # JSON command validation
│   ├── command_schema.py   # Schema validation logic
│   └── settings_manager.py # Persistent settings
├── widgets/                # Reusable GUI widgets
└── tests/                  # Unit and integration tests
```

## System Requirements

### Minimum Requirements
- **CPU**: Intel Core i3 or equivalent
- **RAM**: 4GB
- **GPU**: Integrated graphics with OpenGL 3.3 support
- **Storage**: 100MB free space
- **OS**: Windows 10, macOS 10.14, or Ubuntu 18.04

### Recommended Requirements
- **CPU**: Intel Core i7 or equivalent
- **RAM**: 8GB or higher
- **GPU**: Dedicated graphics with 2GB VRAM
- **Storage**: 500MB free space (for logs and configurations)

## Performance

- **Update Frequency**: 1kHz for motion control
- **GUI Refresh Rate**: 20 FPS for visualization
- **Position Precision**: 0.1 micron resolution
- **Memory Usage**: <500MB typical operating memory
- **Startup Time**: <3 seconds on recommended hardware

## Troubleshooting

### Common Issues

1. **PyVista Not Available**
   - Application will fall back to 2D mode
   - For full 3D visualization: `pip install pyvista pyvistaqt vtk`

2. **High CPU Usage**
   - Reduce GUI update rate in Settings
   - Close unnecessary background applications
   - Ensure adequate cooling

3. **Motion Not Responding**
   - Check if Emergency Stop is active
   - Verify not in the command queue
   - Restart the application if needed

4. **Collision Detection False Positives**
   - Adjust safety minimum clearance settings
   - Check stage positioning accuracy
   - Report potential bugs

## License

This project is provided for simulation and testing purposes. See LICENSE file for details.

## Support

For support and contributions:
- Documentation: See `docs/` directory
- Issues: Report via project issue tracker
- Contributions: Follow development guidelines in CONTRIBUTING.md

---

*Version 1.0 - Simulation Environment for Laser Diode Test Equipment*# laser-demo
# laser-demo
