"""
Control panels for the GUI interface.
"""

from typing import Dict, Any
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QLineEdit, QDoubleSpinBox, QGroupBox,
    QTextEdit, QTableWidget, QTableWidgetItem, QHeaderView,
    QCheckBox, QComboBox, QProgressBar
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor

from ..core.data_structures import SystemState, CollisionWarning


class StageControlPanel(QWidget):
    """Control panel for individual stage."""

    status_changed = pyqtSignal(str, dict)

    def __init__(self, stage_name: str, display_name: str):
        super().__init__()
        self.stage_name = stage_name
        self.display_name = display_name
        self.setup_ui()

    def setup_ui(self):
        """Setup the stage control panel UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(5)

        # Stage title
        title_label = QLabel(self.display_name)
        title_label.setFont(QFont("Arial", 10, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # Status group
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()

        self.status_label = QLabel("Status: Unknown")
        self.status_label.setStyleSheet("color: gray;")
        status_layout.addWidget(self.status_label)

        self.position_labels = {}
        for axis in ['X', 'Y', 'Z']:
            label = QLabel(f"{axis}: 0.000 μm")
            label.setFont(QFont("Courier", 9))
            self.position_labels[axis] = label
            status_layout.addWidget(label)

        # Add rotation controls for 6-axis stage
        if self.stage_name == 'six_axis':
            for axis in ['Rx', 'Ry', 'Rz']:
                label = QLabel(f"{axis}: 0.000°")
                label.setFont(QFont("Courier", 9))
                self.position_labels[axis] = label
                status_layout.addWidget(label)

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        # Position controls group
        controls_group = QGroupBox("Position Controls")
        controls_layout = QGridLayout()

        # Position input fields
        self.position_inputs = {}
        row = 0
        for axis in ['X', 'Y', 'Z']:
            label = QLabel(f"{axis} (μm):")
            controls_layout.addWidget(label, row, 0)

            spinbox = QDoubleSpinBox()
            spinbox.setRange(-200000, 200000)  # ±200mm
            spinbox.setDecimals(1)
            spinbox.setSuffix(" μm")
            self.position_inputs[axis.lower()] = spinbox
            controls_layout.addWidget(spinbox, row, 1)
            row += 1

        # Add rotation controls for 6-axis stage
        if self.stage_name == 'six_axis':
            for axis in ['Rx', 'Ry', 'Rz']:
                label = QLabel(f"{axis} (°):")
                controls_layout.addWidget(label, row, 0)

                spinbox = QDoubleSpinBox()
                spinbox.setRange(-360, 360)
                spinbox.setDecimals(3)
                spinbox.setSuffix("°")
                self.position_inputs[axis.lower()] = spinbox
                controls_layout.addWidget(spinbox, row, 1)
                row += 1

        controls_group.setLayout(controls_layout)
        layout.addWidget(controls_group)

        # Motion parameters group
        motion_group = QGroupBox("Motion Parameters")
        motion_layout = QGridLayout()

        # Velocity
        motion_layout.addWidget(QLabel("Velocity:"), 0, 0)
        self.velocity_input = QDoubleSpinBox()
        self.velocity_input.setRange(1, 1000000)  # 1μm/s to 1m/s
        self.velocity_input.setDecimals(0)
        self.velocity_input.setSuffix(" μm/s")
        self.velocity_input.setValue(50000)  # 50mm/s default
        motion_layout.addWidget(self.velocity_input, 0, 1)

        # Acceleration
        motion_layout.addWidget(QLabel("Acceleration:"), 1, 0)
        self.acceleration_input = QDoubleSpinBox()
        self.acceleration_input.setRange(1, 10000000)  # 1μm/s² to 10m/s²
        self.acceleration_input.setDecimals(0)
        self.acceleration_input.setSuffix(" μm/s²")
        self.acceleration_input.setValue(500000)  # 500mm/s² default
        motion_layout.addWidget(self.acceleration_input, 1, 1)

        motion_group.setLayout(motion_layout)
        layout.addWidget(motion_group)

        # Action buttons
        buttons_layout = QHBoxLayout()

        self.move_button = QPushButton("Move")
        self.move_button.clicked.connect(self.move_to_position)
        buttons_layout.addWidget(self.move_button)

        self.home_button = QPushButton("Home")
        self.home_button.clicked.connect(self.home_stage)
        buttons_layout.addWidget(self.home_button)

        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_stage)
        buttons_layout.addWidget(self.stop_button)

        layout.addLayout(buttons_layout)

        # Stage-specific controls
        if self.stage_name == 'probe_stage':
            self.setup_probe_controls(layout)
        elif self.stage_name == 'three_axis_gantry':
            self.setup_gantry_controls(layout)

        layout.addStretch()

    def setup_probe_controls(self, layout):
        """Setup probe stage specific controls."""
        probe_group = QGroupBox("Probe Controls")
        probe_layout = QVBoxLayout()

        # Press controls
        press_layout = QHBoxLayout()
        press_layout.addWidget(QLabel("Target Z:"))

        self.probe_z_input = QDoubleSpinBox()
        self.probe_z_input.setRange(-15000, 15000)  # ±15mm
        self.probe_z_input.setDecimals(1)
        self.probe_z_input.setSuffix(" μm")
        self.probe_z_input.setValue(0)
        press_layout.addWidget(self.probe_z_input)

        probe_layout.addLayout(press_layout)

        # Force limit
        force_layout = QHBoxLayout()
        force_layout.addWidget(QLabel("Force Limit:"))

        self.force_limit_input = QDoubleSpinBox()
        self.force_limit_input.setRange(1, 10000)
        self.force_limit_input.setDecimals(1)
        self.force_limit_input.setSuffix(" units")
        self.force_limit_input.setValue(1000)
        force_layout.addWidget(self.force_limit_input)

        probe_layout.addLayout(force_layout)

        # Probe action buttons
        probe_buttons = QHBoxLayout()

        self.press_probe_button = QPushButton("Press Probe")
        self.press_probe_button.clicked.connect(self.press_probe)
        probe_buttons.addWidget(self.press_probe_button)

        self.retract_probe_button = QPushButton("Retract")
        self.retract_probe_button.clicked.connect(self.retract_probe)
        probe_buttons.addWidget(self.retract_probe_button)

        probe_layout.addLayout(probe_buttons)

        probe_group.setLayout(probe_layout)
        layout.addWidget(probe_group)

    def setup_gantry_controls(self, layout):
        """Setup gantry specific controls."""
        gantry_group = QGroupBox("Chip Handling")
        gantry_layout = QVBoxLayout()

        # Chip status
        self.chip_status_label = QLabel("Chip: Not Loaded")
        self.chip_status_label.setStyleSheet("color: red;")
        gantry_layout.addWidget(self.chip_status_label)

        # Action buttons
        gantry_buttons = QHBoxLayout()

        self.load_chip_button = QPushButton("Load Chip")
        self.load_chip_button.clicked.connect(self.load_chip)
        gantry_buttons.addWidget(self.load_chip_button)

        self.unload_chip_button = QPushButton("Unload Chip")
        self.unload_chip_button.clicked.connect(self.unload_chip)
        gantry_buttons.addWidget(self.unload_chip_button)

        gantry_layout.addLayout(gantry_buttons)

        gantry_group.setLayout(gantry_layout)
        layout.addWidget(gantry_group)

    def update_status(self, position_data: Dict[str, Any]):
        """Update the stage status display."""
        # Update position labels
        position = position_data.get('position', {})
        for axis, value in position.items():
            if axis.upper() in self.position_labels:
                if axis in ['x', 'y', 'z']:
                    label_text = f"{axis.upper()}: {value:.1f} μm"
                else:  # Rotation axes
                    label_text = f"{axis.upper()}: {value:.3f}°"
                self.position_labels[axis.upper()].setText(label_text)

        # Update status
        is_moving = position_data.get('is_moving', False)
        emergency_stop = position_data.get('emergency_stop', False)

        if emergency_stop:
            self.status_label.setText("Status: Emergency Stop")
            self.status_label.setStyleSheet("color: red;")
        elif is_moving:
            self.status_label.setText("Status: Moving")
            self.status_label.setStyleSheet("color: blue;")
        else:
            self.status_label.setText("Status: Idle")
            self.status_label.setStyleSheet("color: green;")

    def move_to_position(self):
        """Move stage to specified position."""
        position = {}

        # Collect position values
        for axis, input_widget in self.position_inputs.items():
            position[axis] = input_widget.value()

        # Create command
        command = {
            "command_id": f"ST{self.stage_name[:2].upper()}01",
            "command_type": "MOVE",
            "target_stage": self.stage_name,
            "parameters": {
                "position": {"position": position},
                "velocity": self.velocity_input.value(),
                "acceleration": self.acceleration_input.value()
            },
            "execution": {
                "mode": "immediate",
                "priority": 5
            }
        }

        self.status_changed.emit("move_command", command)

    def home_stage(self):
        """Home the stage."""
        command = {
            "command_id": f"HM{self.stage_name[:2].upper()}01",
            "command_type": "HOME",
            "target_stage": self.stage_name,
            "execution": {
                "mode": "immediate"
            }
        }

        self.status_changed.emit("home_command", command)

    def stop_stage(self):
        """Stop stage movement."""
        command = {
            "command_id": f"ST{self.stage_name[:2].upper()}01",
            "command_type": "EMERGENCY_STOP",
            "target_stage": self.stage_name,
            "execution": {
                "mode": "immediate",
                "priority": 10
            }
        }

        self.status_changed.emit("stop_command", command)

    def press_probe(self):
        """Press probe (only for probe stage)."""
        command = {
            "command_id": "PR0001",
            "command_type": "PRESS_PROBE",
            "target_stage": "probe_stage",
            "parameters": {
                "target_z": self.probe_z_input.value(),
                "max_force": self.force_limit_input.value()
            },
            "execution": {
                "mode": "immediate",
                "priority": 7
            }
        }

        self.status_changed.emit("press_probe", command)

    def retract_probe(self):
        """Retract probe (only for probe stage)."""
        command = {
            "command_id": "PR0002",
            "command_type": "RETRACT_PROBE",
            "target_stage": "probe_stage",
            "execution": {
                "mode": "immediate"
            }
        }

        self.status_changed.emit("retract_probe", command)

    def load_chip(self):
        """Load chip (only for gantry)."""
        command = {
            "command_id": "CH0001",
            "command_type": "LOAD_CHIP",
            "target_stage": "three_axis_gantry",
            "parameters": {
                "source_position": {
                    "position": {
                        "x": 0, "y": 0, "z": 30000
                    }
                },
                "destination_position": {
                    "position": {
                        "x": 50000, "y": 50000, "z": 10000
                    }
                }
            },
            "execution": {
                "mode": "queued",
                "priority": 8
            }
        }

        self.status_changed.emit("load_chip", command)

    def unload_chip(self):
        """Unload chip (only for gantry)."""
        command = {
            "command_id": "CH0002",
            "command_type": "UNLOAD_CHIP",
            "target_stage": "three_axis_gantry",
            "execution": {
                "mode": "immediate"
            }
        }

        self.status_changed.emit("unload_chip", command)


class SystemStatusPanel(QWidget):
    """Panel showing overall system status."""

    def __init__(self):
        super().__init__()
        self.setup_ui()

    def setup_ui(self):
        """Setup the system status panel UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(5)

        # System state
        state_group = QGroupBox("System State")
        state_layout = QVBoxLayout()

        self.system_state_label = QLabel("State: Initializing")
        self.system_state_label.setFont(QFont("Arial", 10, QFont.Bold))
        state_layout.addWidget(self.system_state_label)

        self.system_message_label = QLabel("System initializing...")
        self.system_message_label.setWordWrap(True)
        state_layout.addWidget(self.system_message_label)

        state_group.setLayout(state_layout)
        layout.addWidget(state_group)

        # Active stages
        active_group = QGroupBox("Active Stages")
        active_layout = QVBoxLayout()

        self.active_stages_label = QLabel("None")
        active_layout.addWidget(self.active_stages_label)

        active_group.setLayout(active_layout)
        layout.addWidget(active_group)

        # Warnings
        warnings_group = QGroupBox("Collision Warnings")
        warnings_layout = QVBoxLayout()

        self.warnings_list = QTextEdit()
        self.warnings_list.setReadOnly(True)
        self.warnings_list.setMaximumHeight(150)
        self.warnings_list.setFont(QFont("Courier", 9))
        warnings_layout.addWidget(self.warnings_list)

        warnings_group.setLayout(warnings_layout)
        layout.addWidget(warnings_group)

        # Statistics
        stats_group = QGroupBox("System Statistics")
        stats_layout = QGridLayout()

        self.uptime_label = QLabel("Uptime: 0:00:00")
        stats_layout.addWidget(self.uptime_label, 0, 0)

        self.update_rate_label = QLabel("Update Rate: 0 Hz")
        stats_layout.addWidget(self.update_rate_label, 0, 1)

        self.commands_processed_label = QLabel("Commands: 0")
        stats_layout.addWidget(self.commands_processed_label, 1, 0)

        self.collisions_detected_label = QLabel("Collisions: 0")
        stats_layout.addWidget(self.collisions_detected_label, 1, 1)

        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)

        layout.addStretch()

    def update_status(self, status):
        """Update the system status display."""
        # Update system state
        self.system_state_label.setText(f"State: {status.state.value.upper()}")

        # Color code the state
        if status.state == SystemState.EMERGENCY_STOP:
            self.system_state_label.setStyleSheet("color: red;")
        elif status.state == SystemState.ERROR:
            self.system_state_label.setStyleSheet("color: orange;")
        elif status.state == SystemState.MOVING:
            self.system_state_label.setStyleSheet("color: blue;")
        else:
            self.system_state_label.setStyleSheet("color: green;")

        # Update message
        self.system_message_label.setText(status.message)

        # Update active stages
        if status.active_stages:
            active_text = ", ".join(status.active_stages)
        else:
            active_text = "None"
        self.active_stages_label.setText(active_text)

        # Update warnings
        if status.warnings:
            warnings_text = ""
            for warning in status.warnings:
                timestamp = time.strftime("%H:%M:%S", time.localtime(warning.timestamp))
                warnings_text += f"[{timestamp}] {warning.message}\n"
            self.warnings_list.setText(warnings_text)

            # Color code based on severity
            if any(w.severity.value == 'critical' for w in status.warnings):
                self.warnings_list.setStyleSheet("background-color: #ffeeee;")
            else:
                self.warnings_list.setStyleSheet("background-color: #ffffee;")
        else:
            self.warnings_list.setText("No warnings")
            self.warnings_list.setStyleSheet("background-color: white;")


class CommandPanel(QWidget):
    """Panel for sending and monitoring commands."""

    command_sent = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.command_history = []
        self.setup_ui()

    def setup_ui(self):
        """Setup the command panel UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(5)

        # Command editor
        editor_group = QGroupBox("Command Editor")
        editor_layout = QVBoxLayout()

        self.command_editor = QTextEdit()
        self.command_editor.setMaximumHeight(120)
        self.command_editor.setFont(QFont("Courier", 9))
        self.command_editor.setPlainText(
            '{\n'
            '  "command_id": "ST0001",\n'
            '  "command_type": "MOVE",\n'
            '  "target_stage": "six_axis",\n'
            '  "parameters": {\n'
            '    "position": {\n'
            '      "position": {\n'
            '        "x": 10000,\n'
            '        "y": 5000,\n'
            '        "z": 1000\n'
            '      }\n'
            '    }\n'
            '  }\n'
            '}'
        )
        editor_layout.addWidget(self.command_editor)

        # Command control buttons
        command_buttons = QHBoxLayout()

        self.send_button = QPushButton("Send Command")
        self.send_button.clicked.connect(self.send_command)
        command_buttons.addWidget(self.send_button)

        self.queue_button = QPushButton("Queue Command")
        self.queue_button.clicked.connect(self.queue_command)
        command_buttons.addWidget(self.queue_button)

        self.validate_button = QPushButton("Validate")
        self.validate_button.clicked.connect(self.validate_command)
        command_buttons.addWidget(self.validate_button)

        command_buttons.addStretch()

        editor_layout.addLayout(command_buttons)
        editor_group.setLayout(editor_layout)
        layout.addWidget(editor_group)

        # Command templates
        template_group = QGroupBox("Command Templates")
        template_layout = QVBoxLayout()

        self.template_combo = QComboBox()
        self.template_combo.addItems([
            "Move Stage",
            "Home Stage",
            "Press Probe",
            "Retract Probe",
            "Load Chip",
            "Get Status"
        ])
        self.template_combo.currentTextChanged.connect(self.load_template)
        template_layout.addWidget(self.template_combo)

        template_group.setLayout(template_layout)
        layout.addWidget(template_group)

        # Command history
        history_group = QGroupBox("Command History")
        history_layout = QVBoxLayout()

        self.history_table = QTableWidget(0, 5)
        self.history_table.setHorizontalHeaderLabels([
            "ID", "Type", "Target", "Status", "Time"
        ])
        self.history_table.horizontalHeader().setStretchLastSection(True)
        self.history_table.setMaximumHeight(150)
        history_layout.addWidget(self.history_table)

        # Clear history button
        clear_button = QPushButton("Clear History")
        clear_button.clicked.connect(self.clear_history)
        history_layout.addWidget(clear_button)

        history_group.setLayout(history_layout)
        layout.addWidget(history_group)

    def send_command(self):
        """Send the command in the editor."""
        try:
            command_text = self.command_editor.toPlainText()
            command = json.loads(command_text)

            # Set execution mode to immediate
            command['execution'] = command.get('execution', {})
            command['execution']['mode'] = 'immediate'

            self.command_sent.emit(command)

        except json.JSONDecodeError as e:
            self.show_error(f"Invalid JSON: {e}")
        except Exception as e:
            self.show_error(f"Error: {e}")

    def queue_command(self):
        """Queue the command in the editor."""
        try:
            command_text = self.command_editor.toPlainText()
            command = json.loads(command_text)

            # Set execution mode to queued
            command['execution'] = command.get('execution', {})
            command['execution']['mode'] = 'queued'

            self.command_sent.emit(command)

        except json.JSONDecodeError as e:
            self.show_error(f"Invalid JSON: {e}")
        except Exception as e:
            self.show_error(f"Error: {e}")

    def validate_command(self):
        """Validate the command in the editor."""
        try:
            command_text = self.command_editor.toPlainText()
            command = json.loads(command_text)

            # TODO: Use command schema validator
            self.show_info("Command is valid JSON")

        except json.JSONDecodeError as e:
            self.show_error(f"Invalid JSON: {e}")

    def load_template(self, template_name: str):
        """Load a command template."""
        templates = {
            "Move Stage": {
                "command_id": "ST0001",
                "command_type": "MOVE",
                "target_stage": "six_axis",
                "parameters": {
                    "position": {
                        "position": {
                            "x": 10000,
                            "y": 5000,
                            "z": 1000
                        }
                    },
                    "velocity": 50000,
                    "acceleration": 500000
                }
            },
            "Home Stage": {
                "command_id": "HM0001",
                "command_type": "HOME",
                "target_stage": "six_axis"
            },
            "Press Probe": {
                "command_id": "PR0001",
                "command_type": "PRESS_PROBE",
                "target_stage": "probe_stage",
                "parameters": {
                    "target_z": -5000,
                    "max_force": 1000
                }
            },
            "Retract Probe": {
                "command_id": "PR0002",
                "command_type": "RETRACT_PROBE",
                "target_stage": "probe_stage"
            },
            "Load Chip": {
                "command_id": "CH0001",
                "command_type": "LOAD_CHIP",
                "target_stage": "three_axis_gantry",
                "parameters": {
                    "source_position": {
                        "position": {"x": 0, "y": 0, "z": 30000}
                    },
                    "destination_position": {
                        "position": {"x": 50000, "y": 50000, "z": 10000}
                    }
                }
            },
            "Get Status": {
                "command_id": "GT0001",
                "command_type": "GET_STATUS"
            }
        }

        if template_name in templates:
            command = templates[template_name]
            self.command_editor.setPlainText(json.dumps(command, indent=2))

    def update_command_status(self, command_data: Dict[str, Any]):
        """Update command in history table."""
        command_id = command_data.get('command_id', 'Unknown')
        command_type = command_data.get('command_data', {}).get('command_type', 'Unknown')
        target = command_data.get('command_data', {}).get('target_stage', 'System')
        status = command_data.get('status', 'Unknown')
        timestamp = time.strftime("%H:%M:%S", time.localtime(command_data.get('timestamp', time.time())))

        # Find existing row or add new one
        row = -1
        for i in range(self.history_table.rowCount()):
            if self.history_table.item(i, 0).text() == command_id:
                row = i
                break

        if row == -1:
            row = self.history_table.rowCount()
            self.history_table.insertRow(row)

        # Update row
        self.history_table.setItem(row, 0, QTableWidgetItem(command_id))
        self.history_table.setItem(row, 1, QTableWidgetItem(command_type))
        self.history_table.setItem(row, 2, QTableWidgetItem(target))
        self.history_table.setItem(row, 3, QTableWidgetItem(status))
        self.history_table.setItem(row, 4, QTableWidgetItem(timestamp))

        # Color code status
        if status == 'completed':
            self.history_table.item(row, 3).setBackground(QColor(200, 255, 200))
        elif status == 'failed':
            self.history_table.item(row, 3).setBackground(QColor(255, 200, 200))
        elif status == 'executing':
            self.history_table.item(row, 3).setBackground(QColor(200, 200, 255))

        # Auto-scroll to bottom
        self.history_table.scrollToBottom()

        # Limit history size
        if self.history_table.rowCount() > 100:
            self.history_table.removeRow(0)

    def clear_history(self):
        """Clear the command history table."""
        self.history_table.setRowCount(0)

    def show_error(self, message: str):
        """Show an error message."""
        messagebox = QMessageBox()
        messagebox.setIcon(QMessageBox.Critical)
        messagebox.setWindowTitle("Error")
        messagebox.setText(message)
        messagebox.exec_()

    def show_info(self, message: str):
        """Show an info message."""
        messagebox = QMessageBox()
        messagebox.setIcon(QMessageBox.Information)
        messagebox.setWindowTitle("Info")
        messagebox.setText(message)
        messagebox.exec_()


import time
import json