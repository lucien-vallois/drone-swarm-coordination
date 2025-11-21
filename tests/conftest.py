"""
Configurações compartilhadas para testes do drone swarm coordination.
"""

from unittest.mock import Mock

import numpy as np
import pytest
from backend.coordinator import DroneState, Formation, SwarmConfig
from backend.pathfinding import Pathfinding3D
from backend.physics import DronePhysics, WindField


@pytest.fixture
def sample_config():
    """Configuração básica para testes"""
    return SwarmConfig(
        num_drones=3,
        separation_distance=5.0,
        max_velocity=2.0,
        communication_range=20.0,
        collision_threshold=2.0,
        consensus_timeout=5.0,
        heartbeat_interval=1.0,
        formation=Formation.CIRCLE
    )


@pytest.fixture
def sample_drone_state():
    """Estado de drone de exemplo"""
    return DroneState(
        id=0,
        position=np.array([10.0, 10.0, 5.0]),
        velocity=np.array([1.0, 0.0, 0.0]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        battery_level=1.0,
        is_active=True
    )


@pytest.fixture
def pathfinder_3d():
    """Pathfinder 3D para testes"""
    return Pathfinding3D(bounds=(100, 100, 50))


@pytest.fixture
def drone_physics():
    """Drone physics para testes"""
    return DronePhysics(mass=1.0, max_thrust=20.0)


@pytest.fixture
def wind_field():
    """Campo de vento para testes"""
    return WindField(base_speed=2.0, turbulence=0.5)


@pytest.fixture
def mock_logger():
    """Mock logger para testes"""
    return Mock()


@pytest.fixture
def test_positions():
    """Posições de teste comuns"""
    return {
        'start': np.array([0.0, 0.0, 0.0]),
        'goal': np.array([10.0, 10.0, 10.0]),
        'obstacle': np.array([5.0, 5.0, 5.0]),
        'near_obstacle': np.array([5.5, 5.5, 5.5])
    }
