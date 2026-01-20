"""
System controller that orchestrates all stages and manages the simulation environment.
"""

from typing import Dict, List, Optional, Callable, Any
import time
import threading
from dataclasses import dataclass
from enum import Enum
import json

from .data_structures import (
    Vector3D, StagePosition, MotionState, StageType
)
from .stage_controllers import (
    SixAxisStageController, ThreeAxisGantryController,
    ProbeStageController
)
from .collision_detector import CollisionDetector, CollisionWarning


class SystemState(Enum):
    """Overall system state."""
    INITIALIZING = "initializing"
    IDLE = "idle"
    MOVING = "moving"
    TESTING = "testing"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class SystemStatus:
    """Current system status information."""
    state: SystemState
    timestamp: float
    message: str
    warnings: List[CollisionWarning]
    active_stages: List[str]


class Command:
    """Queued command for execution."""

    def __init__(self, command_id: str, command_data: Dict[str, Any]):
        self.command_id = command_id
        self.command_data = command_data
        self.timestamp = time.time()
        self.status = "queued"  # queued, executing, completed, failed
        self.result = None
        self.error_message = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert command to dictionary representation."""
        return {
            'command_id': self.command_id,
            'command_data': self.command_data,
            'timestamp': self.timestamp,
            'status': self.status,
            'result': self.result,
            'error_message': self.error_message
        }


class SystemController:
    """Main system controller managing all stages and operations."""

    def __init__(self):
        # Stage controllers
        self.stages = {
            'six_axis': SixAxisStageController(),
            'three_axis_gantry': ThreeAxisGantryController(),
            'probe_stage': ProbeStageController()
        }

        # Collision detection
        self.collision_detector = CollisionDetector()
        self._setup_collision_detector()

        # System state
        self.system_state = SystemState.INITIALIZING
        self.startup_time = time.time()
        self.emergency_stop_active = False

        # Command queue and execution
        self.command_queue = []
        self.command_history = []
        self.max_command_history = 1000
        self.current_command = None
        self.command_id_counter = 1000

        # Update thread (1kHz)
        self.update_thread = None
        self.running = False
        self.update_interval = 0.001  # 1ms for 1kHz update

        # Callbacks
        self.status_callbacks = []
        self.position_callbacks = []
        self.command_callbacks = []

        # Initialize system
        self._initialize_system()

    def _setup_collision_detector(self):
        """Set up collision detection with all stages."""
        # Register stages with collision detector
        for name, stage in self.stages.items():
            self.collision_detector.register_stage(name, stage)

        # Add collision and emergency stop callbacks
        self.collision_detector.add_warning_callback(self._on_collision_warning)
        self.collision_detector.add_emergency_stop_callback(self._on_emergency_stop)

    def _initialize_system(self):
        """Initialize the system to ready state."""
        try:
            # Home all stages
            self.home_all_stages()

            # Enable collision detection
            self.collision_detector.enable()

            # Set system state
            self.system_state = SystemState.IDLE
            self._notify_status_change(
                "System initialized successfully",
                SystemState.IDLE
            )

        except Exception as e:
            self.system_state = SystemState.ERROR
            self._notify_status_change(
                f"System initialization failed: {str(e)}",
                SystemState.ERROR
            )

    def start(self):
        """Start the system controller update loop."""
        if self.running:
            return False

        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
        return True

    def stop(self):
        """Stop the system controller."""
        self.running = False
        if self.update_thread:
            self.update_thread.join(timeout=1.0)

    def emergency_stop(self):
        """Trigger emergency stop - halt all motion immediately."""
        self.emergency_stop_active = True
        self.system_state = SystemState.EMERGENCY_STOP

        # Stop all stages
        for stage in self.stages.values():
            stage.emergency_stop()

        # Clear command queue
        self.command_queue.clear()
        if self.current_command:
            self.current_command.status = "failed"
            self.current_command.error_message = "Emergency stop triggered"
            self._finalize_command(self.current_command)
            self.current_command = None

        self._notify_status_change(
            "Emergency stop triggered by user",
            SystemState.EMERGENCY_STOP
        )

    def reset_emergency_stop(self):
        """Reset emergency stop state."""
        self.emergency_stop_active = False

        # Reset all stages
        for stage in self.stages.items():
            stage.reset_emergency_stop()

        self.system_state = SystemState.IDLE
        self._notify_status_change("Emergency stop cleared", SystemState.IDLE)

    def add_command(self, command_data: Dict[str, Any],
                   priority: int = 5) -> str:
        """
        Add a command to the execution queue.

        Args:
            command_data: Command dictionary with type, target, parameters
            priority: Command priority (1-10, higher = more urgent)

        Returns:
            Command ID for tracking
        """
        command_id = f"CMD{self.command_id_counter:04d}"
        self.command_id_counter += 1

        command = Command(command_id, command_data)

        # Insert command based on priority
        inserted = False
        for i, cmd in enumerate(self.command_queue):
            if priority > cmd.command_data.get('priority', 5):
                self.command_queue.insert(i, command)
                inserted = True
                break

        if not inserted:
            self.command_queue.append(command)

        self._notify_command_status(command)
        return command_id

    def execute_command(self, command_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute a single command immediately (bypassing queue).

        Args:
            command_data: Command dictionary

        Returns:
            Execution result with status and any output
        """
        if self.emergency_stop_active:
            return {
                'success': False,
                'error': 'Emergency stop is active'
            }

        command_type = command_data.get('command_type')
        target_stage = command_data.get('target_stage')
        parameters = command_data.get('parameters', {})

        try:
            if command_type == 'MOVE':
                return self._handle_move_command(target_stage, parameters)
            elif command_type == 'HOME':
                return self._handle_home_command(target_stage)
            elif command_type == 'PRESS_PROBE':
                return self._handle_press_probe_command(parameters)
            elif command_type == 'RETRACT_PROBE':
                return self._handle_retract_probe_command()
            elif command_type == 'LOAD_CHIP':
                return self._handle_load_chip_command(parameters)
            elif command_type == 'UNLOAD_CHIP':
                return self._handle_unload_chip_command()
            elif command_type == 'GET_STATUS':
                return self._handle_get_status_command()
            elif command_type == 'GET_POSITION':
                return self._handle_get_position_command(target_stage)
            else:
                return {
                    'success': False,
                    'error': f'Unknown command type: {command_type}'
                }

        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }

    def _update_loop(self):
        """Main update loop running at 1kHz."""
        while self.running:
            start_time = time.time()

            # Update all stages
            for stage in self.stages.values():
                stage.update()

            # Check collisions
            warnings = self.collision_detector.check_all_collisions()

            # Process command queue
            self._process_command_queue()

            # Update system state based on stage states
            self._update_system_state()

            # Notify position callbacks
            self._notify_position_updates()

            # Sleep to maintain 1kHz update rate
            elapsed = time.time() - start_time
            if elapsed < self.update_interval:
                time.sleep(self.update_interval - elapsed)

    def _process_command_queue(self):
        """Process queued commands."""
        if self.current_command is None and self.command_queue:
            # Get next command
            self.current_command = self.command_queue.pop(0)
            self.current_command.status = "executing"
            self._notify_command_status(self.current_command)

            # Execute command
            try:
                result = self.execute_command(self.current_command.command_data)
                if result.get('success', False):
                    self.current_command.status = "completed"
                    self.current_command.result = result
                else:
                    self.current_command.status = "failed"
                    self.current_command.error_message = result.get('error', 'Unknown error')

            except Exception as e:
                self.current_command.status = "failed"
                self.current_command.error_message = str(e)

            # Finalize command
            self._finalize_command(self.current_command)
            self.current_command = None

    def _finalize_command(self, command: Command):
        """Finalize command execution and move to history."""
        self.command_history.append(command.to_dict())

        # Limit history size
        if len(self.command_history) > self.max_command_history:
            self.command_history.pop(0)

        self._notify_command_status(command)

    def _handle_move_command(self, target_stage: str,
                           parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Handle MOVE command."""
        if target_stage not in self.stages:
            return {
                'success': False,
                'error': f'Unknown stage: {target_stage}'
            }

        stage = self.stages[target_stage]

        # Parse position
        pos_dict = parameters.get('position', {})
        position = StagePosition.from_dict(pos_dict)

        # Get motion parameters
        velocity = parameters.get('velocity')
        acceleration = parameters.get('acceleration')

        # Execute move
        success = stage.move_to_position(position, velocity, acceleration)

        return {
            'success': success,
            'message': f'Move command sent to {target_stage}'
        }

    def _handle_home_command(self, target_stage: Optional[str]) -> Dict[str, Any]:
        """Handle HOME command."""
        if target_stage:
            # Home specific stage
            if target_stage not in self.stages:
                return {
                    'success': False,
                    'error': f'Unknown stage: {target_stage}'
                }
            return self._home_single_stage(target_stage)
        else:
            # Home all stages
            return self.home_all_stages()

    def _handle_press_probe_command(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Handle PRESS_PROBE command."""
        probe_stage = self.stages['probe_stage']
        target_z = parameters.get('target_z', 0)
        max_force = parameters.get('max_force')

        success = probe_stage.press_probe(target_z, max_force)

        return {
            'success': success,
            'message': f'Probe press command sent to Z={target_z}'
        }

    def _handle_retract_probe_command(self) -> Dict[str, Any]:
        """Handle RETRACT_PROBE command."""
        probe_stage = self.stages['probe_stage']
        success = probe_stage.retract_probe()

        return {
            'success': success,
            'message': 'Probe retraction command sent'
        }

    def _handle_load_chip_command(self, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """Handle LOAD_CHIP command."""
        gantry = self.stages['three_axis_gantry']

        # Parse source and destination positions
        source_pos = StagePosition.from_dict(parameters.get('source_position', {}))
        dest_pos = StagePosition.from_dict(parameters.get('destination_position', {}))

        # Move to source
        gantry.move_to_position(source_pos)
        # TODO: Wait for motion complete

        # Pick up chip
        gantry.set_chip_loaded(True)

        # Move to destination
        gantry.move_to_position(dest_pos)
        # TODO: Wait for motion complete

        return {
            'success': True,
            'message': 'Chip loaded successfully'
        }

    def _handle_unload_chip_command(self) -> Dict[str, Any]:
        """Handle UNLOAD_CHIP command."""
        gantry = self.stages['three_axis_gantry']
        gantry.set_chip_loaded(False)

        return {
            'success': True,
            'message': 'Chip unloaded successfully'
        }

    def _handle_get_status_command(self) -> Dict[str, Any]:
        """Handle GET_STATUS command."""
        return {
            'success': True,
            'status': self.get_system_status().to_dict()
        }

    def _handle_get_position_command(self, target_stage: str) -> Dict[str, Any]:
        """Handle GET_POSITION command."""
        if target_stage not in self.stages:
            return {
                'success': False,
                'error': f'Unknown stage: {target_stage}'
            }

        stage = self.stages[target_stage]
        position = stage.get_current_state().position

        return {
            'success': True,
            'position': position.to_dict()
        }

    def home_all_stages(self) -> Dict[str, Any]:
        """Home all stages to their home positions."""
        results = {}

        for name, stage in self.stages.items():
            success = self._home_single_stage(name)
            results[name] = success.get('success', False)

        all_success = all(results.values())

        if all_success:
            self._notify_status_change("All stages homed", self.system_state)
        else:
            self._notify_status_change("Some stages failed to home", SystemState.ERROR)

        return {
            'success': all_success,
            'results': results,
            'message': 'All stages homed successfully' if all_success else 'Some stages failed to home'
        }

    def _home_single_stage(self, stage_name: str) -> Dict[str, Any]:
        """Home a single stage."""
        stage = self.stages[stage_name]
        home_pos = stage._get_home_position()

        success = stage.move_to_position(home_pos)

        return {
            'success': success,
            'message': f'Stage {stage_name} homed' if success else f'Stage {stage_name} homing failed'
        }

    def _update_system_state(self):
        """Update overall system state based on stage states."""
        if self.emergency_stop_active:
            return

        # Check if any stage is moving
        stages_moving = [name for name, stage in self.stages.items() if stage.is_moving]

        if stages_moving:
            if self.system_state != SystemState.MOVING and self.system_state != SystemState.TESTING:
                self.system_state = SystemState.MOVING
                self._notify_status_change(
                    f"Stages moving: {', '.join(stages_moving)}",
                    SystemState.MOVING
                )
        else:
            if self.system_state == SystemState.MOVING:
                self.system_state = SystemState.IDLE
                self._notify_status_change("All stages stopped", SystemState.IDLE)

    def get_system_status(self) -> SystemStatus:
        """Get current system status."""
        warnings = self.collision_detector.get_current_warnings()
        active_stages = [name for name, stage in self.stages.items() if stage.is_moving]

        return SystemStatus(
            state=self.system_state,
            timestamp=time.time(),
            message=self._get_status_message(),
            warnings=warnings,
            active_stages=active_stages
        )

    def get_all_positions(self) -> Dict[str, Any]:
        """Get current positions of all stages."""
        positions = {}

        for name, stage in self.stages.items():
            state = stage.get_current_state()
            positions[name] = {
                'position': state.position.to_dict(),
                'velocity': {
                    'x': state.velocity.x,
                    'y': state.velocity.y,
                    'z': state.velocity.z
                },
                'is_moving': stage.is_moving,
                'emergency_stop': stage.emergency_stop_triggered
            }

        return positions

    def _get_status_message(self) -> str:
        """Get appropriate status message for current state."""
        messages = {
            SystemState.INITIALIZING: "System initializing...",
            SystemState.IDLE: "System ready",
            SystemState.MOVING: "Stages in motion",
            SystemState.TESTING: "Testing in progress",
            SystemState.ERROR: "System error",
            SystemState.EMERGENCY_STOP: "Emergency stop active"
        }

        return messages.get(self.system_state, "Unknown state")

    def _on_collision_warning(self, warning: CollisionWarning):
        """Handle collision warning from collision detector."""
        self._notify_status_change(
            f"Collision warning: {warning.message}",
            SystemState.ERROR if warning.severity.value == "critical" else self.system_state
        )

    def _on_emergency_stop(self, warning: CollisionWarning):
        """Handle emergency stop from collision detector."""
        self.emergency_stop()
        self._notify_status_change(
            f"Auto emergency stop: {warning.message}",
            SystemState.EMERGENCY_STOP
        )

    def _notify_status_change(self, message: str, state: SystemState):
        """Notify all status callbacks of state change."""
        status = self.get_system_status()
        for callback in self.status_callbacks:
            callback(message, state, status)

    def _notify_position_updates(self):
        """Notify all position callbacks."""
        positions = self.get_all_positions()
        for callback in self.position_callbacks:
            callback(positions)

    def _notify_command_status(self, command: Command):
        """Notify all command callbacks."""
        for callback in self.command_callbacks:
            callback(command.to_dict())

    def add_status_callback(self, callback: Callable):
        """Add status change callback."""
        self.status_callbacks.append(callback)

    def add_position_callback(self, callback: Callable):
        """Add position update callback."""
        self.position_callbacks.append(callback)

    def add_command_callback(self, callback: Callable):
        """Add command status callback."""
        self.command_callbacks.append(callback)