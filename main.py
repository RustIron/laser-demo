#!/usr/bin/env python3
"""
Main entry point for the Laser Diode Test Equipment Simulation Environment.

This application provides a high-fidelity simulation of laser diode test automation equipment
featuring:
- 6-axis stage for precision positioning
- 3-axis gantry for chip loading
- 3-axis probe stage with press/unpress functionality
- Real-time 3D visualization
- JSON-based command interface
- Advanced collision detection and safety systems

Usage:
    python main.py [options]

Options:
    --config FILE     Load configuration from FILE
    --headless        Run without GUI (for testing)
    --debug           Enable debug logging
    --help            Show this help message

Author: Simulation Team
Version: 1.0
"""

import sys
import argparse
import logging
import os
from pathlib import Path

# Add the project root to Python path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from PyQt5.QtWidgets import QApplication, QMessageBox
from PyQt5.QtCore import Qt, QSettings
from PyQt5.QtGui import QIcon

from gui.main_window import MainWindow
from core.system_controller import SystemController
from config.settings_manager import get_settings_manager


def setup_logging(debug=False):
    """Setup logging configuration."""
    level = logging.DEBUG if debug else logging.INFO

    # Create logs directory if it doesn't exist
    logs_dir = project_root / "logs"
    logs_dir.mkdir(exist_ok=True)

    # Configure logging
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(logs_dir / "simulation.log"),
            logging.StreamHandler(sys.stdout) if debug else logging.NullHandler()
        ]
    )

    return logging.getLogger(__name__)


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Laser Diode Test Equipment Simulation Environment",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python main.py                           # Run with GUI
    python main.py --config test_config.json # Load custom configuration
    python main.py --headless               # Run without GUI (for testing)
    python main.py --debug                  # Enable debug logging

JSON command examples:
    {
        "command_id": "ST0001",
        "command_type": "MOVE",
        "target_stage": "six_axis",
        "parameters": {
            "position": {
                "position": {
                    "x": 10500,
                    "y": 5200,
                    "z": 100
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
        """
    )

    parser.add_argument(
        '--config',
        type=str,
        help='Load configuration from JSON file'
    )

    parser.add_argument(
        '--headless',
        action='store_true',
        help='Run without GUI (for automated testing)'
    )

    parser.add_argument(
        '--debug',
        action='store_true',
        help='Enable debug logging'
    )

    parser.add_argument(
        '--version',
        action='version',
        version='Laser Diode Test Equipment Simulation v1.0'
    )

    return parser.parse_args()


def check_dependencies():
    """Check if required dependencies are available."""
    missing = []

    try:
        import PyQt5
    except ImportError:
        missing.append("PyQt5")

    try:
        import numpy
    except ImportError:
        missing.append("NumPy")

    try:
        import pyvista
    except ImportError:
        print("Warning: PyVista not available. 3D visualization will be disabled.")

    if missing:
        print(f"Error: Missing required dependencies: {', '.join(missing)}")
        print("\nTo install dependencies, run:")
        print("pip install -r requirements.txt")
        return False

    return True


def load_configuration(config_file: str = None):
    """Load application configuration."""
    settings_manager = get_settings_manager()

    if config_file:
        try:
            # Import configuration
            success = settings_manager.import_settings(config_file)
            if success:
                print(f"Configuration loaded from {config_file}")
            else:
                print(f"Failed to load configuration from {config_file}")
        except Exception as e:
            print(f"Error loading configuration: {e}")

    # Validate settings
    validation = settings_manager.validate_settings()
    if not validation['valid']:
        print("Configuration validation errors:")
        for issue in validation['issues']:
            print(f"  - {issue}")
        return False

    return True


def run_headless_test():
    """Run simulation in headless mode for testing."""
    logger = logging.getLogger(__name__)
    logger.info("Running simulation in headless mode")

    # Initialize system controller
    system_controller = SystemController()
    system_controller.start()

    # Run test sequence
    test_commands = [
        {
            "command_id": "HM0001",
            "command_type": "HOME",
            "target_stage": None
        },
        {
            "command_id": "ST0002",
            "command_type": "MOVE",
            "target_stage": "six_axis",
            "parameters": {
                "position": {
                    "position": {
                        "x": 10000,
                        "y": 5000,
                        "z": 1000
                    }
                }
            }
        },
        {
            "command_id": "GT0003",
            "command_type": "GET_STATUS"
        }
    ]

    # Execute test commands
    for command in test_commands:
        logger.info(f"Executing command: {command['command_id']}")
        result = system_controller.execute_command(command)
        logger.info(f"Result: {result}")

        # Wait for completion
        import time
        time.sleep(1)

    # Cleanup
    system_controller.stop()
    logger.info("Headless test completed")


def main():
    """Main application entry point."""
    # Parse command line arguments
    args = parse_arguments()

    # Setup logging
    logger = setup_logging(args.debug)

    # Check dependencies
    if not check_dependencies():
        sys.exit(1)

    # Load configuration
    if not load_configuration(args.config):
        logger.error("Configuration loading failed")
        sys.exit(1)

    # Check if running headless
    if args.headless:
        run_headless_test()
        return

    # Initialize Qt application
    app = QApplication(sys.argv)
    app.setApplicationName("Laser Diode Test Equipment Simulation")
    app.setApplicationVersion("1.0")
    app.setOrganizationName("Test Equipment Simulation")

    # Set application style
    app.setStyle('Fusion')

    # Enable high DPI support
    app.setAttribute(Qt.AA_EnableHighDpiScaling)
    app.setAttribute(Qt.AA_UseHighDpiPixmaps)

    # Create and show main window
    try:
        main_window = MainWindow()
        main_window.show()

        # Show welcome message
        QMessageBox.information(
            main_window,
            "Welcome",
            "Laser Diode Test Equipment Simulation\n\n"
            "This is a high-fidelity simulation environment for\n"
            "laser diode test automation equipment.\n\n"
            "For help, see the Help > About menu."
        )

        logger.info("Application started successfully")

    except Exception as e:
        logger.error(f"Failed to create main window: {e}")
        QMessageBox.critical(
            None,
            "Startup Error",
            f"Failed to start the application:\n{e}"
        )
        sys.exit(1)

    # Run application event loop
    try:
        exit_code = app.exec_()
        logger.info("Application shutting down")
        sys.exit(exit_code)

    except KeyboardInterrupt:
        logger.info("Application interrupted by user")
        sys.exit(0)

    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()