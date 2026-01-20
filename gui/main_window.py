"""
Main application window for the laser diode test equipment simulation.
"""

import json
import time
from typing import Dict, Any, Optional
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSplitter,
    QTabWidget, QTextEdit, QPushButton, QLabel, QStatusBar,
    QMessageBox, QFileDialog, QMenuBar, QMenu, QAction,
    QGroupBox, QProgressBar, QTextBrowser
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot, QThread
from PyQt5.QtGui import QIcon, QFont, QColor, QPalette

from .visualization import VisualizationWidget
from .control_panels import StageControlPanel, SystemStatusPanel, CommandPanel
from ..core.system_controller import SystemController
from ..core.data_structures import SystemState, CollisionWarning
from ..config.settings_manager import get_settings_manager


class MainWindow(QMainWindow):
    """Main application window."""

    def __init__(self):
        super().__init__()
        self.system_controller = None
        self.settings_manager = get_settings_manager()

        # Initialize UI
        self.setup_ui()
        self.setup_menu()
        self.setup_status_bar()

        # Setup system controller
        self.setup_system_controller()

        # Setup update timers
        self.setup_timers()

        # Load settings
        self.load_settings()

        # Initialize system
        self.initialize_system()

    def setup_ui(self):
        """Setup the main user interface."""
        self.setWindowTitle("Laser Diode Test Equipment Simulation")
        self.setGeometry(100, 100, 1400, 900)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Create main splitter
        main_splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(main_splitter)

        # Left panel - Stage controls
        self.left_panel = self.create_left_panel()
        main_splitter.addWidget(self.left_panel)
        main_splitter.setSizes([400, 600, 400])  # Width proportions

        # Center panel - Visualization
        self.center_panel = self.create_center_panel()
        main_splitter.addWidget(self.center_panel)

        # Right panel - Status and logs
        self.right_panel = self.create_right_panel()
        main_splitter.addWidget(self.right_panel)

    def create_left_panel(self) -> QWidget:
        """Create the left control panel."""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Stage controls (tabbed)
        self.stage_tabs = QTabWidget()
        self.stage_tabs.setTabPosition(QTabWidget.North)

        # Add stage control panels
        self.six_axis_panel = StageControlPanel("six_axis", "6-Axis Stage")
        self.stage_tabs.addTab(self.six_axis_panel, "6-Axis")

        self.gantry_panel = StageControlPanel("three_axis_gantry", "3-Axis Gantry")
        self.stage_tabs.addTab(self.gantry_panel, "Gantry")

        self.probe_panel = StageControlPanel("probe_stage", "Probe Stage")
        self.stage_tabs.addTab(self.probe_panel, "Probe")

        layout.addWidget(self.stage_tabs)

        # Emergency stop button
        self.emergency_stop_button = QPushButton("EMERGENCY STOP")
        self.emergency_stop_button.setStyleSheet("""
            QPushButton {
                background-color: #ff4444;
                color: white;
                font-weight: bold;
                font-size: 14px;
                padding: 10px;
                border: none;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #cc0000;
            }
            QPushButton:pressed {
                background-color: #990000;
            }
        """)
        self.emergency_stop_button.clicked.connect(self.emergency_stop)

        # Reset emergency stop button
        self.reset_emergency_button = QPushButton("Reset Emergency Stop")
        self.reset_emergency_button.setStyleSheet("""
            QPushButton {
                background-color: #ffaa00;
                color: black;
                font-weight: bold;
                padding: 8px;
                border: none;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #ff8800;
            }
        """)
        self.reset_emergency_button.clicked.connect(self.reset_emergency_stop)
        self.reset_emergency_button.setEnabled(False)

        # Add buttons to layout
        button_layout = QVBoxLayout()
        button_layout.addWidget(self.emergency_stop_button)
        button_layout.addWidget(self.reset_emergency_button)
        layout.addLayout(button_layout)

        # Add stretch to fill remaining space
        layout.addStretch()

        return panel

    def create_center_panel(self) -> QWidget:
        """Create the center visualization panel."""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Visualization widget
        self.visualization = VisualizationWidget()
        layout.addWidget(self.visualization)

        # Position tracker
        self.position_info = QTextEdit()
        self.position_info.setReadOnly(True)
        self.position_info.setMaximumHeight(150)
        self.position_info.setFont(QFont("Courier", 9))
        layout.addWidget(QLabel("Position Information:"))
        layout.addWidget(self.position_info)

        return panel

    def create_right_panel(self) -> QWidget:
        """Create the right status and logs panel."""
        panel = QWidget()
        layout = QVBoxLayout(panel)

        # System status
        self.status_panel = SystemStatusPanel()
        layout.addWidget(self.status_panel)

        # Command panel
        self.command_panel = CommandPanel()
        self.command_panel.command_sent.connect(self.on_command_sent)
        layout.addWidget(self.command_panel)

        # Logs
        logs_group = QGroupBox("System Logs")
        logs_layout = QVBoxLayout(logs_group)

        self.log_browser = QTextBrowser()
        self.log_browser.setFont(QFont("Courier", 9))
        logs_layout.addWidget(self.log_browser)

        # Log controls
        log_controls = QHBoxLayout()
        self.clear_logs_button = QPushButton("Clear Logs")
        self.clear_logs_button.clicked.connect(self.clear_logs)
        self.save_logs_button = QPushButton("Save Logs")
        self.save_logs_button.clicked.connect(self.save_logs)
        log_controls.addWidget(self.clear_logs_button)
        log_controls.addWidget(self.save_logs_button)
        log_controls.addStretch()
        logs_layout.addLayout(log_controls)

        layout.addWidget(logs_group)

        return panel

    def setup_menu(self):
        """Setup the menu bar."""
        menubar = self.menuBar()

        # File menu
        file_menu = menubar.addMenu("File")

        # Load configuration
        load_config_action = QAction("Load Configuration", self)
        load_config_action.setShortcut("Ctrl+O")
        load_config_action.triggered.connect(self.load_configuration)
        file_menu.addAction(load_config_action)

        # Save configuration
        save_config_action = QAction("Save Configuration", self)
        save_config_action.setShortcut("Ctrl+S")
        save_config_action.triggered.connect(self.save_configuration)
        file_menu.addAction(save_config_action)

        file_menu.addSeparator()

        # Exit
        exit_action = QAction("Exit", self)
        exit_action.setShortcut("Ctrl+Q")
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # View menu
        view_menu = menubar.addMenu("View")

        # Reset view
        reset_view_action = QAction("Reset View", self)
        reset_view_action.triggered.connect(self.visualization.reset_view)
        view_menu.addAction(reset_view_action)

        # Take screenshot
        screenshot_action = QAction("Take Screenshot", self)
        screenshot_action.triggered.connect(self.take_screenshot)
        view_menu.addAction(screenshot_action)

        # Tools menu
        tools_menu = menubar.addMenu("Tools")

        # Home all stages
        home_all_action = QAction("Home All Stages", self)
        home_all_action.triggered.connect(self.home_all_stages)
        tools_menu.addAction(home_all_action)

        # Test collision detection
        test_collision_action = QAction("Test Collision Detection", self)
        test_collision_action.triggered.connect(self.test_collision_detection)
        tools_menu.addAction(test_collision_action)

        # Help menu
        help_menu = menubar.addMenu("Help")

        # About
        about_action = QAction("About", self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)

    def setup_status_bar(self):
        """Setup the status bar."""
        self.status_bar = QStatusBar()

        # Connected status
        self.connection_label = QLabel("System: Initializing...")
        self.status_bar.addWidget(self.connection_label)

        # Command queue status
        self.queue_label = QLabel("Queue: 0 commands")
        self.status_bar.addWidget(self.queue_label)

        # Update rate
        self.update_rate_label = QLabel("Update: 0 Hz")
        self.status_bar.addWidget(self.update_rate_label)

        self.setStatusBar(self.status_bar)

    def setup_system_controller(self):
        """Setup the system controller and callbacks."""
        self.system_controller = SystemController()

        # Add callbacks
        self.system_controller.add_status_callback(self.on_system_status_change)
        self.system_controller.add_position_callback(self.on_position_update)
        self.system_controller.add_command_callback(self.on_command_status_update)

        # Connect collision detector callbacks
        self.system_controller.collision_detector.add_warning_callback(self.on_collision_warning)
        self.system_controller.collision_detector.add_emergency_stop_callback(self.on_emergency_stop)

    def setup_timers(self):
        """Setup update timers."""
        # General update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_status_display)
        self.update_timer.start(100)  # 10 Hz update rate

        # Update rate calculation timer
        self.update_count = 0
        self.update_rate_timer = QTimer()
        self.update_rate_timer.timeout.connect(self.calculate_update_rate)
        self.update_rate_timer.start(1000)  # Calculate rate every second

    def initialize_system(self):
        """Initialize the system controller."""
        if self.system_controller:
            success = self.system_controller.start()
            if success:
                self.log_message("System initialized successfully")
                self.connection_label.setText("System: Ready")
            else:
                self.log_message("Failed to initialize system", "ERROR")
                self.connection_label.setText("System: Error")

    def load_settings(self):
        """Load GUI settings."""
        gui_settings = self.settings_manager.get_gui_settings()

        # Set window geometry
        self.setGeometry(
            gui_settings.window_x,
            gui_settings.window_y,
            gui_settings.window_width,
            gui_settings.window_height
        )

        # Apply other settings
        self.visualization.max_trail_length = gui_settings.trail_length
        # TODO: Apply more GUI settings

    def save_settings(self):
        """Save GUI settings."""
        gui_settings = self.settings_manager.get_gui_settings()

        # Update window geometry
        gui_settings.window_x = self.x()
        gui_settings.window_y = self.y()
        gui_settings.window_width = self.width()
        gui_settings.window_height = self.height()

        # Update trail length
        gui_settings.trail_length = self.visualization.max_trail_length

        self.settings_manager.update_gui_settings(gui_settings)
        self.settings_manager.save_settings()

    @pyqtSlot()
    def update_status_display(self):
        """Update the status display."""
        if not self.system_controller:
            return

        # Update system status
        status = self.system_controller.get_system_status()
        self.status_panel.update_status(status)

        # Update command queue status
        queue_size = len(self.system_controller.command_queue)
        self.queue_label.setText(f"Queue: {queue_size} commands")

        # Update position information
        positions = self.system_controller.get_all_positions()
        self.update_position_display(positions)

        self.update_count += 1

    def update_position_display(self, positions: Dict[str, Any]):
        """Update the position information display."""
        text = ""
        for stage_name, stage_data in positions.items():
            position = stage_data['position']
            text += f"{stage_name.upper()}:\n"
            text += f"  Pos: X={position.get('x', 0):.1f}, "
            text += f"Y={position.get('y', 0):.1f}, "
            text += f"Z={position.get('z', 0):.1f} μm\n"
            if stage_data.get('is_moving', False):
                text += "  Status: Moving\n"
            else:
                text += "  Status: Idle\n"
            text += "-" * 30 + "\n"

        self.position_info.setText(text)

    def calculate_update_rate(self):
        """Calculate and display update rate."""
        rate = self.update_count
        self.update_count = 0
        self.update_rate_label.setText(f"Update: {rate} Hz")

    @pyqtSlot(str, SystemState)
    def on_system_status_change(self, message: str, state: SystemState, status: Any = None):
        """Handle system status changes."""
        # Update status bar
        if state == SystemState.EMERGENCY_STOP:
            self.connection_label.setText("System: EMERGENCY STOP")
            self.connection_label.setStyleSheet("color: red;")
        elif state == SystemState.ERROR:
            self.connection_label.setText("System: Error")
            self.connection_label.setStyleSheet("color: orange;")
        elif state == SystemState.MOVING:
            self.connection_label.setText("System: Moving")
            self.connection_label.setStyleSheet("color: blue;")
        else:  # IDLE or TESTING
            self.connection_label.setText("System: Ready")
            self.connection_label.setStyleSheet("color: green;")

        # Update emergency stop buttons
        if state == SystemState.EMERGENCY_STOP:
            self.emergency_stop_button.setEnabled(False)
            self.reset_emergency_button.setEnabled(True)
        else:
            self.emergency_stop_button.setEnabled(True)
            self.reset_emergency_button.setEnabled(False)

        # Log message
        self.log_message(message)

    @pyqtSlot(dict)
    def on_position_update(self, positions: Dict[str, Any]):
        """Handle position updates."""
        # Update visualization
        for stage_name, stage_data in positions.items():
            if stage_name in ['six_axis', 'three_axis_gantry', 'probe_stage']:
                from ..core.data_structures import StagePosition
                position = StagePosition.from_dict(stage_data['position'])
                self.visualization.update_stage_position(stage_name, position)

    @pyqtSlot(CollisionWarning)
    def on_collision_warning(self, warning: CollisionWarning):
        """Handle collision warnings."""
        # Show in visualization
        self.visualization.show_collision_warning(warning)

        # Log warning
        self.log_message(f"Collision: {warning.message}", "WARNING")

    @pyqtSlot(CollisionWarning)
    def on_emergency_stop(self, warning: CollisionWarning):
        """Handle emergency stop events."""
        self.log_message(f"Auto Emergency Stop: {warning.message}", "ERROR")
        QMessageBox.critical(self, "Emergency Stop", warning.message)

    @pyqtSlot(dict)
    def on_command_status_update(self, command_data: Dict[str, Any]):
        """Handle command status updates."""
        self.command_panel.update_command_status(command_data)

    @pyqtSlot(dict)
    def on_command_sent(self, command_data: Dict[str, Any]):
        """Handle command sending."""
        if self.system_controller:
            # Determine execution mode
            execution = command_data.get('execution', {})
            mode = execution.get('mode', 'immediate')
            priority = execution.get('priority', 5)

            if mode == 'immediate':
                # Execute immediately
                result = self.system_controller.execute_command(command_data)
                self.log_message(f"Command executed: {result}")
            else:
                # Add to queue
                command_id = self.system_controller.add_command(command_data, priority)
                self.log_message(f"Command queued: {command_id}")

    def emergency_stop(self):
        """Trigger emergency stop."""
        if self.system_controller:
            self.system_controller.emergency_stop()
            self.log_message("Emergency stop triggered by user", "WARNING")

    def reset_emergency_stop(self):
        """Reset emergency stop."""
        if self.system_controller:
            self.system_controller.reset_emergency_stop()
            self.log_message("Emergency stop reset")

    def home_all_stages(self):
        """Home all stages."""
        if self.system_controller:
            result = self.system_controller.home_all_stages()
            self.log_message(f"Home command result: {result}")

    def test_collision_detection(self):
        """Test collision detection functionality."""
        # Move stages close to each other to trigger warnings
        self.log_message("Testing collision detection...", "INFO")
        # TODO: Implement test sequence

    def load_configuration(self):
        """Load configuration from file."""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Configuration", "", "JSON Files (*.json)")
        if filename:
            try:
                with open(filename, 'r') as f:
                    config = json.load(f)
                # TODO: Apply configuration to system
                self.log_message(f"Configuration loaded from {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load configuration: {e}")

    def save_configuration(self):
        """Save configuration to file."""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Configuration", "", "JSON Files (*.json)")
        if filename:
            try:
                # TODO: Export current configuration
                config = {}  # Placeholder
                with open(filename, 'w') as f:
                    json.dump(config, f, indent=2)
                self.log_message(f"Configuration saved to {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save configuration: {e}")

    def take_screenshot(self):
        """Take a screenshot of the visualization."""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Screenshot", "", "PNG Files (*.png)")
        if filename:
            screenshot_path = self.visualization.get_screenshot(filename)
            self.log_message(f"Screenshot saved to {screenshot_path}")

    def clear_logs(self):
        """Clear the log browser."""
        self.log_browser.clear()

    def save_logs(self):
        """Save logs to file."""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Logs", "", "Text Files (*.txt)")
        if filename:
            try:
                with open(filename, 'w') as f:
                    f.write(self.log_browser.toPlainText())
                self.log_message(f"Logs saved to {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save logs: {e}")

    def show_about(self):
        """Show about dialog."""
        QMessageBox.about(self, "About",
            "Laser Diode Test Equipment Simulation\n\n"
            "Version 1.0\n"
            "A high-fidelity simulation environment for\n"
            "laser diode test automation equipment.\n\n"
            "Features:\n"
            "• 6-axis stage control\n"
            "• 3-axis gantry movement\n"
            "• Probe stage with press/unpress\n"
            "• Real-time collision detection\n"
            "• 3D visualization\n"
            "• JSON command interface")

    def log_message(self, message: str, level: str = "INFO"):
        """Add a message to the log."""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] [{level}] {message}"

        # Add to log browser
        self.log_browser.append(log_entry)

        # Auto-scroll to bottom
        scrollbar = self.log_browser.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def closeEvent(self, event):
        """Handle window close event."""
        # Save settings
        self.save_settings()

        # Stop system controller
        if self.system_controller:
            self.system_controller.stop()

        event.accept()