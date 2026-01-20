"""
JSON command schema validation and processing utilities.
"""

import json
import jsonschema
from typing import Dict, Any, List, Optional
from pathlib import Path


class CommandValidator:
    """Validates JSON commands against the defined schema."""

    def __init__(self):
        self.schema = None
        self.validator = None
        self._load_schema()

    def _load_schema(self):
        """Load the command schema from JSON file."""
        schema_path = Path(__file__).parent / "command_schema.json"
        try:
            with open(schema_path, 'r') as f:
                self.schema = json.load(f)
            self.validator = jsonschema.Draft7Validator(self.schema)
        except Exception as e:
            raise RuntimeError(f"Failed to load command schema: {e}")

    def validate_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate a command against the schema.

        Args:
            command: Command dictionary to validate

        Returns:
            Validation result with success flag and any errors
        """
        if not self.validator:
            return {
                'valid': False,
                'error': 'Schema not loaded'
            }

        try:
            # Validate against schema
            self.validator.validate(command)

            # Additional validation checks
            additional_errors = self._additional_validation(command)

            if additional_errors:
                return {
                    'valid': False,
                    'errors': additional_errors
                }

            return {
                'valid': True,
                'errors': []
            }

        except jsonschema.ValidationError as e:
            return {
                'valid': False,
                'error': str(e),
                'error_path': list(e.path) if e.path else []
            }
        except Exception as e:
            return {
                'valid': False,
                'error': f'Validation error: {str(e)}'
            }

    def _additional_validation(self, command: Dict[str, Any]) -> List[str]:
        """
        Perform additional validation beyond the JSON schema.

        Args:
            command: Command dictionary

        Returns:
            List of additional validation error messages
        """
        errors = []

        command_type = command.get('command_type')

        if command_type == 'MOVE':
            errors.extend(self._validate_move_command(command))
        elif command_type == 'PRESS_PROBE':
            errors.extend(self._validate_press_probe_command(command))
        elif command_type == 'LOAD_CHIP':
            errors.extend(self._validate_load_chip_command(command))

        return errors

    def _validate_move_command(self, command: Dict[str, Any]) -> List[str]:
        """Additional validation for MOVE commands."""
        errors = []

        target_stage = command.get('target_stage')
        parameters = command.get('parameters', {})
        position = parameters.get('position', {})

        if not position:
            errors.append("MOVE command requires position parameter")
            return errors

        # Stage-specific position validation
        if target_stage == 'six_axis':
            # Six-axis stage can have rotation
            if 'rotation' not in position:
                position['rotation'] = {'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
        elif target_stage in ['three_axis_gantry', 'probe_stage']:
            # These stages only use linear axes
            if 'rotation' in position:
                errors.append(f"{target_stage} does not support rotation")

        # Check reasonableness of positions (within typical ranges)
        linear_pos = position.get('position', {})
        for axis, pos in linear_pos.items():
            if abs(pos) > 200000:  # 200mm limit
                errors.append(f"{axis} position {pos} microns exceeds reasonable range")

        return errors

    def _validate_press_probe_command(self, command: Dict[str, Any]) -> List[str]:
        """Additional validation for PRESS_PROBE commands."""
        errors = []

        parameters = command.get('parameters', {})
        target_z = parameters.get('target_z')

        if target_z is None:
            errors.append("PRESS_PROBE command requires target_z parameter")
        elif target_z < -15000 or target_z > 15000:
            errors.append(f"target_z {target_z} microns is outside probe range (-15000 to 15000)")

        max_force = parameters.get('max_force')
        if max_force is not None and max_force < 0:
            errors.append("max_force must be positive")

        return errors

    def _validate_load_chip_command(self, command: Dict[str, Any]) -> List[str]:
        """Additional validation for LOAD_CHIP commands."""
        errors = []

        parameters = command.get('parameters', {})
        source_pos = parameters.get('source_position')
        dest_pos = parameters.get('destination_position')

        if not source_pos:
            errors.append("LOAD_CHIP command requires source_position")
        if not dest_pos:
            errors.append("LOAD_CHIP command requires destination_position")

        return errors

    def get_schema_info(self) -> Dict[str, Any]:
        """Get information about the available command schema."""
        if not self.schema:
            return {}

        return {
            'title': self.schema.get('title', 'Unknown'),
            'description': self.schema.get('description', ''),
            'command_types': self.schema.get('properties', {}).get('command_type', {}).get('enum', []),
            'stages': self.schema.get('properties', {}).get('target_stage', {}).get('enum', [])
        }


class CommandTemplates:
    """Provides templates and examples for common commands."""

    @staticmethod
    def get_move_template(stage: str, position: Dict[str, Any],
                         velocity: float = None, acceleration: float = None,
                         command_id: str = "MO0001") -> Dict[str, Any]:
        """Get a template for MOVE command."""
        template = {
            "command_id": command_id,
            "command_type": "MOVE",
            "target_stage": stage,
            "parameters": {
                "position": position
            },
            "execution": {
                "mode": "immediate",
                "priority": 5,
                "timeout": 30.0
            }
        }

        if velocity is not None:
            template["parameters"]["velocity"] = velocity
        if acceleration is not None:
            template["parameters"]["acceleration"] = acceleration

        return template

    @staticmethod
    def get_home_template(stage: Optional[str] = None, command_id: str = "HM0001") -> Dict[str, Any]:
        """Get a template for HOME command."""
        template = {
            "command_id": command_id,
            "command_type": "HOME",
            "target_stage": stage,
            "execution": {
                "mode": "immediate"
            }
        }

        return template

    @staticmethod
    def get_press_probe_template(target_z: float, max_force: float = None,
                               command_id: str = "PR0001") -> Dict[str, Any]:
        """Get a template for PRESS_PROBE command."""
        template = {
            "command_id": command_id,
            "command_type": "PRESS_PROBE",
            "target_stage": "probe_stage",
            "parameters": {
                "target_z": target_z
            },
            "execution": {
                "mode": "queued",
                "priority": 7
            }
        }

        if max_force is not None:
            template["parameters"]["max_force"] = max_force

        return template

    @staticmethod
    def get_load_chip_template(source: Dict[str, Any], destination: Dict[str, Any],
                             command_id: str = "CH0001") -> Dict[str, Any]:
        """Get a template for LOAD_CHIP command."""
        template = {
            "command_id": command_id,
            "command_type": "LOAD_CHIP",
            "target_stage": "three_axis_gantry",
            "parameters": {
                "source_position": source,
                "destination_position": destination
            },
            "execution": {
                "mode": "queued",
                "priority": 8
            }
        }

        return template

    @staticmethod
    def get_emergency_stop_template(command_id: str = "ES0001") -> Dict[str, Any]:
        """Get a template for EMERGENCY_STOP command."""
        return {
            "command_id": command_id,
            "command_type": "EMERGENCY_STOP",
            "target_stage": None,
            "execution": {
                "mode": "immediate",
                "priority": 10
            }
        }


class CommandBatch:
    """Manages batches of commands for execution."""

    def __init__(self, description: str = ""):
        self.description = description
        self.commands: List[Dict[str, Any]] = []
        self.metadata = {
            "created_at": None,
            "created_by": "user",
            "batch_id": None
        }

    def add_command(self, command: Dict[str, Any]):
        """Add a command to the batch."""
        self.commands.append(command)

    def add_commands(self, commands: List[Dict[str, Any]]):
        """Add multiple commands to the batch."""
        self.commands.extend(commands)

    def to_json(self, indent: int = 2) -> str:
        """Convert batch to JSON string."""
        batch_data = {
            "metadata": self.metadata,
            "description": self.description,
            "commands": self.commands
        }
        return json.dumps(batch_data, indent=indent)

    @classmethod
    def from_json(cls, json_str: str) -> 'CommandBatch':
        """Create batch from JSON string."""
        batch_data = json.loads(json_str)
        batch = cls(batch_data.get('description', ''))
        batch.metadata = batch_data.get('metadata', {})
        batch.commands = batch_data.get('commands', [])
        return batch

    def validate_all(self, validator: CommandValidator) -> List[Dict[str, Any]]:
        """Validate all commands in the batch."""
        results = []
        for i, command in enumerate(self.commands):
            result = validator.validate_command(command)
            result['command_index'] = i
            result['command_id'] = command.get('command_id', f'Command_{i}')
            results.append(result)
        return results

    def get_execution_plan(self) -> List[Dict[str, Any]]:
        """Get a summarized execution plan for the batch."""
        plan = []
        for command in self.commands:
            plan.append({
                'command_id': command.get('command_id'),
                'command_type': command.get('command_type'),
                'target_stage': command.get('target_stage'),
                'priority': command.get('execution', {}).get('priority', 5),
                'mode': command.get('execution', {}).get('mode', 'immediate')
            })
        return plan


def validate_command(command: Dict[str, Any]) -> Dict[str, Any]:
    """
    Global function to validate a single command.

    Args:
        command: Command dictionary to validate

    Returns:
        Validation result
    """
    validator = CommandValidator()
    return validator.validate_command(command)


def get_command_schema() -> Dict[str, Any]:
    """
    Get the command schema.

    Returns:
        Command schema dictionary
    """
    validator = CommandValidator()
    return validator.schema