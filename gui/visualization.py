"""
3D visualization widget for real-time stage position display.
"""

from typing import Dict, Any, List, Tuple, Optional
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QSlider, QLabel, QGroupBox
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor

try:
    import pyvista as pv
    from pyvistaqt import QtInteractor
    PYVISTA_AVAILABLE = True
except ImportError:
    PYVISTA_AVAILABLE = False
    print("PyVista not available. Using fallback 2D visualization.")

from ..core.data_structures import (
    Vector3D, StagePosition, BoundingBox, CollisionWarning,
    StageType
)


class VisualizationWidget(QWidget):
    """3D visualization widget for stage positions and collisions."""

    # Signals
    view_updated = pyqtSignal(dict)
    collision_detected = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()

        # Visualization data
        self.stage_positions = {}
        self.stage_meshes = {}
        self.collision_warnings = []
        self.position_trail = []
        self.max_trail_length = 100

        # Camera parameters
        self.camera_distance = 200000  # microns
        self.camera_rotation = [30, -60]  # Azimuth, elevation
        self.auto_rotate = False

        # Setup plotter
        self.setup_visualization()

        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_visualization)
        self.update_timer.start(50)  # 20 FPS

    def setup_ui(self):
        """Setup the user interface layout."""
        layout = QVBoxLayout(self)

        # Control panel
        control_layout = QHBoxLayout()

        # View controls
        view_group = QGroupBox("View Controls")
        view_layout = QVBoxLayout()

        # Reset view button
        self.reset_button = QPushButton("Reset View")
        self.reset_button.clicked.connect(self.reset_view)
        view_layout.addWidget(self.reset_button)

        # Zoom slider
        zoom_layout = QHBoxLayout()
        zoom_layout.addWidget(QLabel("Zoom:"))
        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setRange(10, 500)
        self.zoom_slider.setValue(100)
        self.zoom_slider.valueChanged.connect(self.on_zoom_changed)
        zoom_layout.addWidget(self.zoom_slider)
        view_layout.addLayout(zoom_layout)

        # Auto-rotate checkbox
        self.auto_rotate_check = QPushButton("Auto Rotate: OFF")
        self.auto_rotate_check.setCheckable(True)
        self.auto_rotate_check.clicked.connect(self.toggle_auto_rotate)
        view_layout.addWidget(self.auto_rotate_check)

        view_group.setLayout(view_layout)
        control_layout.addWidget(view_group)

        # Options panel
        options_group = QGroupBox("Display Options")
        options_layout = QVBoxLayout()

        # Show trail checkbox
        self.show_trail_check = QPushButton("Show Trail: ON")
        self.show_trail_check.setCheckable(True)
        self.show_trail_check.setChecked(True)
        self.show_trail_check.clicked.connect(self.toggle_trail)
        options_layout.addWidget(self.show_trail_check)

        # Show axes checkbox
        self.show_axes_check = QPushButton("Show Axes: ON")
        self.show_axes_check.setCheckable(True)
        self.show_axes_check.setChecked(True)
        self.show_axes_check.clicked.connect(self.toggle_axes)
        options_layout.addWidget(self.show_axes_check)

        # Show grid checkbox
        self.show_grid_check = QPushButton("Show Grid: ON")
        self.show_grid_check.setCheckable(True)
        self.show_grid_check.setChecked(True)
        self.show_grid_check.clicked.connect(self.toggle_grid)
        options_layout.addWidget(self.show_grid_check)

        options_group.setLayout(options_layout)
        control_layout.addWidget(options_group)

        layout.addLayout(control_layout)

        # Visualization widget will be added here
        self.viz_container = QWidget()
        self.viz_layout = QVBoxLayout(self.viz_container)
        layout.addWidget(self.viz_container)

    def setup_visualization(self):
        """Setup the 3D visualization plotter."""
        if PYVISTA_AVAILABLE:
            # Use PyVista for 3D visualization
            self.plotter = QtInteractor(self.viz_container)
            self.viz_layout.addWidget(self.plotter.interactor)

            # Configure plotter
            self.plotter.add_light(pv.Light(position=(100, 100, 100), intensity=0.5))
            self.plotter.add_light(pv.Light(position=(-100, -100, 100), intensity=0.5))
            self.plotter.add_light(pv.Light(position=(0, 0, 200), intensity=0.8))

            # Add coordinate axes
            self.plotter.add_axes()

            # Add grid
            self.setup_grid()

            # Create stage meshes
            self.create_stage_meshes()

        else:
            # Fallback to 2D placeholder
            from PyQt5.QtWidgets import QLabel
            placeholder = QLabel("3D Visualization (PyVista not installed)")
            placeholder.setAlignment(Qt.AlignCenter)
            placeholder.setStyleSheet("background-color: #f0f0f0; border: 1px solid #ccc;")
            self.viz_layout.addWidget(placeholder)
            self.plotter = None

    def setup_grid(self):
        """Setup the reference grid."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        # Create floor grid
        grid_size = 200000  # 200mm
        grid_divisions = 20
        grid_range = np.linspace(-grid_size/2, grid_size/2, grid_divisions)

        # Create grid lines
        for x in grid_range:
            line = pv.Line([x, -grid_size/2, 0], [x, grid_size/2, 0])
            self.plotter.add_mesh(line, color='gray', line_width=1, opacity=0.3)

        for y in grid_range:
            line = pv.Line([-grid_size/2, y, 0], [grid_size/2, y, 0])
            self.plotter.add_mesh(line, color='gray', line_width=1, opacity=0.3)

        # Add axes labels
        self.plotter.add_text("+X", position=(grid_size/2, 0, 0), font_size=20)
        self.plotter.add_text("+Y", position=(0, grid_size/2, 0), font_size=20)
        self.plotter.add_text("+Z", position=(0, 0, grid_size/4), font_size=20)

    def create_stage_meshes(self):
        """Create mesh representations for each stage."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        # Six-axis stage (main positioning stage)
        six_axis_mesh = pv.Box(bounds=(-50000, 50000, -50000, 50000, -10000, 10000))
        six_axis_actor = self.plotter.add_mesh(
            six_axis_mesh, color='blue', opacity=0.7, name='six_axis'
        )

        # Three-axis gantry (chip loader)
        gantry_mesh = pv.Box(bounds=(-30000, 30000, -30000, 30000, 0, 20000))
        gantry_actor = self.plotter.add_mesh(
            gantry_mesh, color='green', opacity=0.7, name='three_axis_gantry'
        )

        # Probe stage
        probe_mesh = pv.Box(bounds=(-10000, 10000, -10000, 10000, -5000, 5000))
        probe_actor = self.plotter.add_mesh(
            probe_mesh, color='red', opacity=0.7, name='probe_stage'
        )

        # Store actors for manipulation
        self.stage_meshes = {
            'six_axis': {'mesh': six_axis_mesh, 'actor': six_axis_actor},
            'three_axis_gantry': {'mesh': gantry_mesh, 'actor': gantry_actor},
            'probe_stage': {'mesh': probe_mesh, 'actor': probe_actor}
        }

        # Add trajectory trail
        self.trail_actors = []

        # Add safety zones
        self.create_safety_zones()

    def create_safety_zones(self):
        """Create visual representations of safety zones."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        # Probe zone (high priority)
        probe_zone = pv.Box(bounds=(-30000, 30000, -30000, 30000, -15000, 15000))
        self.plotter.add_mesh(
            probe_zone, color='yellow', opacity=0.1, name='probe_zone',
            show_edges=True, line_width=2, edge_color='orange'
        )

        # Chip loading zone
        loading_zone = pv.Box(bounds=(-150000, 150000, -150000, 150000, -10000, 60000))
        self.plotter.add_mesh(
            loading_zone, color='cyan', opacity=0.05, name='loading_zone',
            show_edges=True, line_width=1, edge_color='blue'
        )

    def update_stage_position(self, stage_name: str, position: StagePosition):
        """Update the visual position of a stage."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        if stage_name not in self.stage_meshes:
            return

        pos = position.position
        self.stage_positions[stage_name] = position

        # Update mesh position
        stage_data = self.stage_meshes[stage_name]
        mesh = stage_data['mesh']
        actor = stage_data['actor']

        # Calculate new bounds based on position
        if stage_name == 'six_axis':
            bounds = (
                pos.x - 50000, pos.x + 50000,
                pos.y - 50000, pos.y + 50000,
                pos.z - 10000, pos.z + 10000
            )
        elif stage_name == 'three_axis_gantry':
            bounds = (
                pos.x - 30000, pos.x + 30000,
                pos.y - 30000, pos.y + 30000,
                pos.z, pos.z + 20000
            )
        elif stage_name == 'probe_stage':
            bounds = (
                pos.x - 10000, pos.x + 10000,
                pos.y - 10000, pos.y + 10000,
                pos.z - 5000, pos.z + 5000
            )
        else:
            return

        # Update mesh bounds
        mesh.bounds = bounds
        actor.Setbounds(bounds)

        # Update position trail
        if self.show_trail_check.isChecked():
            self.add_to_trail(stage_name, pos)

    def add_to_trail(self, stage_name: str, position: Vector3D):
        """Add position to trail visualization."""
        trail_point = {
            'stage': stage_name,
            'position': position,
            'timestamp': time.time()
        }

        self.position_trail.append(trail_point)

        # Limit trail length
        if len(self.position_trail) > self.max_trail_length:
            self.position_trail.pop(0)

        # Update trail visualization
        self.update_trail_visualization()

    def update_trail_visualization(self):
        """Update the trail visualization."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        # Clear old trail actors
        for actor in self.trail_actors:
            self.plotter.remove_actor(actor)
        self.trail_actors.clear()

        # Group trail points by stage
        stage_trails = {}
        for point in self.position_trail:
            stage = point['stage']
            if stage not in stage_trails:
                stage_trails[stage] = []
            stage_trails[stage].append(point['position'])

        # Create trail lines for each stage
        stage_colors = {
            'six_axis': 'blue',
            'three_axis_gantry': 'green',
            'probe_stage': 'red'
        }

        for stage, positions in stage_trails.items():
            if len(positions) < 2:
                continue

            # Create line from positions
            points = np.array([[p.x, p.y, p.z] for p in positions])
            line = pv.PolyData(points)
            line.lines = np.array([len(positions), *range(len(positions))])

            color = stage_colors.get(stage, 'gray')
            actor = self.plotter.add_mesh(
                line, color=color, line_width=2, opacity=0.5,
                name=f'trail_{stage}'
            )
            self.trail_actors.append(actor)

    def show_collision_warning(self, warning: CollisionWarning):
        """Display collision warning visually."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        # Create a sphere at collision point
        sphere = pv.Sphere(center=[warning.position.x, warning.position.y, warning.position.z], radius=5000)

        # Color based on severity
        severity_colors = {
            'info': 'blue',
            'warning': 'yellow',
            'critical': 'orange',
            'emergency_stop': 'red'
        }
        color = severity_colors.get(warning.severity.value, 'red')

        actor = self.plotter.add_mesh(
            sphere, color=color, opacity=0.8,
            name=f'collision_{warning.timestamp}'
        )

        # Schedule removal after 5 seconds
        QTimer.singleShot(5000, lambda: self.plotter.remove_actor(actor))

        # Emit signal
        self.collision_detected.emit(warning.message)

    def update_visualization(self):
        """Update the visualization."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        # Auto-rotate if enabled
        if self.auto_rotate:
            current_pos = self.plotter.camera_position
            # Rotate camera around Z axis
            angle_increment = 1.0  # degrees per update (20 FPS = 20 degrees/second)
            # TODO: Implement camera rotation

        # Update view
        self.plotter.render()

        # Emit update signal
        self.view_updated.emit(self.stage_positions)

    def reset_view(self):
        """Reset camera to default position."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        self.plotter.reset_camera()
        self.zoom_slider.setValue(100)

    def on_zoom_changed(self, value):
        """Handle zoom slider change."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        zoom_factor = value / 100.0
        camera = self.plotter.camera
        # Adjust camera distance
        camera.Zoom(zoom_factor)

    def toggle_auto_rotate(self):
        """Toggle auto-rotation."""
        self.auto_rotate = not self.auto_rotate
        self.auto_rotate_check.setText(f"Auto Rotate: {'ON' if self.auto_rotate else 'OFF'}")

    def toggle_trail(self):
        """Toggle position trail display."""
        show = self.show_trail_check.isChecked()
        self.show_trail_check.setText(f"Show Trail: {'ON' if show else 'OFF'}")

        if not show:
            # Clear trail
            for actor in self.trail_actors:
                self.plotter.remove_actor(actor)
            self.trail_actors.clear()
            self.position_trail.clear()

    def toggle_axes(self):
        """Toggle axes display."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        show = self.show_axes_check.isChecked()
        self.show_axes_check.setText(f"Show Axes: {'ON' if show else 'OFF'}")

        # TODO: Toggle axes visibility
        # Need to store reference to axes actor

    def toggle_grid(self):
        """Toggle grid display."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        show = self.show_grid_check.isChecked()
        self.show_grid_check.setText(f"Show Grid: {'ON' if show else 'OFF'}")

        # TODO: Toggle grid visibility
        # Need to store references to grid actors

    def set_view_angle(self, azimuth: float, elevation: float):
        """Set camera view angle."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        camera = self.plotter.camera
        camera.SetPosition(
            self.camera_distance * np.cos(np.radians(elevation)) * np.cos(np.radians(azimuth)),
            self.camera_distance * np.cos(np.radians(elevation)) * np.sin(np.radians(azimuth)),
            self.camera_distance * np.sin(np.radians(elevation))
        )
        camera.SetFocalPoint(0, 0, 0)
        camera.SetViewUp(0, 0, 1)

    def focus_on_stage(self, stage_name: str):
        """Center view on a specific stage."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return

        if stage_name in self.stage_positions:
            pos = self.stage_positions[stage_name].position
            self.set_view_angle(
                pos.x + self.camera_distance * 0.7,
                pos.y + self.camera_distance * 0.7,
                pos.z + self.camera_distance * 0.7
            )

    def get_screenshot(self, filename: str = None) -> str:
        """Capture screenshot of the visualization."""
        if not PYVISTA_AVAILABLE or not self.plotter:
            return ""

        return self.plotter.screenshot(filename)

    def clear_warnings(self):
        """Clear all collision warnings from visualization."""
        # This would require storing references to warning actors
        pass


import time