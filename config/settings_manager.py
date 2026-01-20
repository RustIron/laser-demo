"""
Settings management for persistent configuration storage.
"""

import json
import os
from pathlib import Path
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict
import time


@dataclass
class StageSettings:
    """Settings for individual stage controllers."""
    max_velocity: float
    max_acceleration: float
    max_jerk: float
    position_tolerance: float
    velocity_tolerance: float
    enabled: bool = True


@dataclass
class GUISettings:
    """GUI user preferences."""
    window_width: int = 1200
    window_height: int = 800
    window_x: int = 100
    window_y: int = 100
    theme: str = "default"
    font_size: int = 10
    update_interval: int = 50  # ms
    show_position_trail: bool = True
    trail_length: int = 100
    auto_rotate_3d: bool = False
    rotation_speed: float = 1.0
    camera_distance: float = 200000  # microns


@dataclass
class SafetySettings:
    """Safety and collision detection settings."""
    min_clearance: float = 1000.0  # microns
    prediction_time: float = 2.0  # seconds
    auto_emergency_stop: bool = True
    warning_distance: float = 2000.0  # microns
    force_limit_probe: float = 1000.0  # force units


@dataclass
class SystemSettings:
    """Complete system configuration."""
    stages: Dict[str, StageSettings]
    gui: GUISettings
    safety: SafetySettings
    version: str = "1.0"
    last_updated: float = 0.0


class SettingsManager:
    """Manages application settings with persistence."""

    def __init__(self, config_dir: Optional[str] = None):
        if config_dir is None:
            # Use default settings directory
            self.config_dir = Path.home() / '.laser_test_simulator'
        else:
            self.config_dir = Path(config_dir)

        # Ensure config directory exists
        self.config_dir.mkdir(parents=True, exist_ok=True)

        # Settings file paths
        self.settings_file = self.config_dir / 'settings.json'
        self.backup_file = self.config_dir / 'settings_backup.json'

        # Default settings
        self.default_settings = self._create_default_settings()
        self.settings = None

        # Load settings
        self.load_settings()

    def _create_default_settings(self) -> SystemSettings:
        """Create default system settings."""
        return SystemSettings(
            stages={
                'six_axis': StageSettings(
                    max_velocity=100000,  # 100mm/s
                    max_acceleration=500000,  # 500mm/s²
                    max_jerk=10000000,  # 10000mm/s³
                    position_tolerance=0.1,  # 0.1 micron
                    velocity_tolerance=10.0  # 10 microns/s
                ),
                'three_axis_gantry': StageSettings(
                    max_velocity=200000,  # 200mm/s
                    max_acceleration=1000000,  # 1000mm/s²
                    max_jerk=20000000,  # 20000mm/s³
                    position_tolerance=0.5,  # 0.5 micron
                    velocity_tolerance=20.0  # 20 microns/s
                ),
                'probe_stage': StageSettings(
                    max_velocity=50000,  # 50mm/s
                    max_acceleration=250000,  # 250mm/s²
                    max_jerk=5000000,  # 5000mm/s³
                    position_tolerance=0.01,  # 0.01 micron
                    velocity_tolerance=1.0  # 1 micron/s
                )
            },
            gui=GUISettings(),
            safety=SafetySettings()
        )

    def load_settings(self) -> bool:
        """
        Load settings from file.

        Returns:
            True if settings loaded successfully, False otherwise
        """
        try:
            if self.settings_file.exists():
                with open(self.settings_file, 'r') as f:
                    settings_data = json.load(f)

                # Convert to SystemSettings object
                self.settings = self._dict_to_settings(settings_data)
                return True
            else:
                # Use default settings
                self.settings = self.default_settings
                self.save_settings()  # Create the file
                return False

        except Exception as e:
            print(f"Failed to load settings: {e}")
            self.settings = self.default_settings
            return False

    def save_settings(self) -> bool:
        """
        Save current settings to file.

        Returns:
            True if saved successfully, False otherwise
        """
        try:
            # Create backup before saving
            if self.settings_file.exists():
                import shutil
                shutil.copy2(self.settings_file, self.backup_file)

            # Update timestamp
            if self.settings:
                self.settings.last_updated = time.time()

            # Convert to dictionary and save
            settings_data = self._settings_to_dict(self.settings)
            with open(self.settings_file, 'w') as f:
                json.dump(settings_data, f, indent=2)

            return True

        except Exception as e:
            print(f"Failed to save settings: {e}")
            return False

    def restore_defaults(self) -> bool:
        """
        Restore settings to defaults.

        Returns:
            True if restored successfully
        """
        self.settings = self.default_settings
        return self.save_settings()

    def backup_settings(self, backup_name: str = None) -> str:
        """
        Create a backup of current settings.

        Args:
            backup_name: Optional custom backup name

        Returns:
            Path to backup file
        """
        if backup_name is None:
            timestamp = int(time.time())
            backup_name = f'settings_backup_{timestamp}.json'

        backup_path = self.config_dir / backup_name

        settings_data = self._settings_to_dict(self.settings)
        with open(backup_path, 'w') as f:
            json.dump(settings_data, f, indent=2)

        return str(backup_path)

    def load_backup(self, backup_path: str) -> bool:
        """
        Load settings from backup file.

        Args:
            backup_path: Path to backup file

        Returns:
            True if loaded successfully
        """
        try:
            with open(backup_path, 'r') as f:
                settings_data = json.load(f)

            self.settings = self._dict_to_settings(settings_data)
            return self.save_settings()

        except Exception as e:
            print(f"Failed to load backup: {e}")
            return False

    def get_stage_settings(self, stage_name: str) -> StageSettings:
        """Get settings for a specific stage."""
        if stage_name in self.settings.stages:
            return self.settings.stages[stage_name]
        else:
            # Return default stage settings
            return StageSettings(
                max_velocity=50000,
                max_acceleration=250000,
                max_jerk=5000000,
                position_tolerance=0.1,
                velocity_tolerance=10.0
            )

    def update_stage_settings(self, stage_name: str, settings: StageSettings):
        """Update settings for a specific stage."""
        if stage_name not in self.settings.stages:
            self.settings.stages[stage_name] = settings
        else:
            self.settings.stages[stage_name] = settings

    def get_gui_settings(self) -> GUISettings:
        """Get GUI settings."""
        return self.settings.gui

    def update_gui_settings(self, settings: GUISettings):
        """Update GUI settings."""
        self.settings.gui = settings

    def get_safety_settings(self) -> SafetySettings:
        """Get safety settings."""
        return self.settings.safety

    def update_safety_settings(self, settings: SafetySettings):
        """Update safety settings."""
        self.settings.safety = settings

    def export_settings(self, export_path: str) -> bool:
        """
        Export settings to external file.

        Args:
            export_path: Path to export file

        Returns:
            True if exported successfully
        """
        try:
            settings_data = self._settings_to_dict(self.settings)
            with open(export_path, 'w') as f:
                json.dump(settings_data, f, indent=2)
            return True
        except Exception as e:
            print(f"Failed to export settings: {e}")
            return False

    def import_settings(self, import_path: str) -> bool:
        """
        Import settings from external file.

        Args:
            import_path: Path to import file

        Returns:
            True if imported successfully
        """
        try:
            with open(import_path, 'r') as f:
                settings_data = json.load(f)

            self.settings = self._dict_to_settings(settings_data)
            return self.save_settings()
        except Exception as e:
            print(f"Failed to import settings: {e}")
            return False

    def _settings_to_dict(self, settings: SystemSettings) -> Dict[str, Any]:
        """Convert SystemSettings to dictionary."""
        return asdict(settings)

    def _dict_to_settings(self, data: Dict[str, Any]) -> SystemSettings:
        """Convert dictionary to SystemSettings."""
        # Convert stage settings
        stages = {}
        for name, stage_data in data.get('stages', {}).items():
            stages[name] = StageSettings(**stage_data)

        # Convert GUI settings
        gui = GUISettings(**data.get('gui', {}))

        # Convert safety settings
        safety = SafetySettings(**data.get('safety', {}))

        return SystemSettings(
            stages=stages,
            gui=gui,
            safety=safety,
            version=data.get('version', '1.0'),
            last_updated=data.get('last_updated', time.time())
        )

    def get_settings_summary(self) -> Dict[str, Any]:
        """Get a summary of current settings."""
        return {
            'version': self.settings.version,
            'last_updated': self.settings.last_updated,
            'stages_configured': len(self.settings.stages),
            'gui_theme': self.settings.gui.theme,
            'safety_enabled': self.settings.safety.auto_emergency_stop,
            'min_clearance': self.settings.safety.min_clearance
        }

    def validate_settings(self) -> Dict[str, Any]:
        """
        Validate current settings.

        Returns:
            Validation result with any issues found
        """
        issues = []

        # Check stage settings
        for name, stage in self.settings.stages.items():
            if stage.max_velocity <= 0:
                issues.append(f"{name}: max_velocity must be positive")
            if stage.max_acceleration <= 0:
                issues.append(f"{name}: max_acceleration must be positive")
            if stage.max_jerk <= 0:
                issues.append(f"{name}: max_jerk must be positive")
            if stage.position_tolerance <= 0:
                issues.append(f"{name}: position_tolerance must be positive")

        # Check GUI settings
        if self.settings.gui.window_width <= 0 or self.settings.gui.window_height <= 0:
            issues.append("GUI window dimensions must be positive")
        if self.settings.gui.update_interval <= 0:
            issues.append("GUI update interval must be positive")

        # Check safety settings
        if self.settings.safety.min_clearance <= 0:
            issues.append("Safety minimum clearance must be positive")
        if self.settings.safety.prediction_time <= 0:
            issues.append("Safety prediction time must be positive")

        return {
            'valid': len(issues) == 0,
            'issues': issues
        }

    def list_backups(self) -> List[str]:
        """List available backup files."""
        backups = []
        for file in self.config_dir.glob('settings_backup_*.json'):
            backups.append(str(file))
        return sorted(backups)

    def cleanup_old_backups(self, keep_count: int = 5):
        """Clean up old backup files, keeping only the most recent."""
        backups = self.list_backups()
        if len(backups) > keep_count:
            for backup in backups[:-keep_count]:
                try:
                    os.remove(backup)
                except Exception as e:
                    print(f"Failed to remove backup {backup}: {e}")


# Global settings manager instance
_settings_manager = None


def get_settings_manager(config_dir: Optional[str] = None) -> SettingsManager:
    """Get global settings manager instance."""
    global _settings_manager
    if _settings_manager is None:
        _settings_manager = SettingsManager(config_dir)
    return _settings_manager


def load_stage_settings(stage_name: str) -> StageSettings:
    """Load settings for a specific stage."""
    manager = get_settings_manager()
    return manager.get_stage_settings(stage_name)


def save_stage_settings(stage_name: str, settings: StageSettings):
    """Save settings for a specific stage."""
    manager = get_settings_manager()
    manager.update_stage_settings(stage_name, settings)
    manager.save_settings()