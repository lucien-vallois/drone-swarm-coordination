"""
Physics simulation for drone swarm
Implements rigid body dynamics, aerodynamics, and environmental interactions
"""

import logging
from typing import Dict, List, Optional, Tuple

import numpy as np

# Import profiling
from backend.profiling import profile_function, profile_physics


class DronePhysics:
    """Physics simulation for individual drone"""

    def __init__(self, mass: float = 1.0, max_thrust: float = 20.0,
                 drag_coefficient: float = 0.1, wind_speed: float = 0.0):
        self.mass = mass  # kg
        self.max_thrust = max_thrust  # N
        self.drag_coefficient = drag_coefficient
        self.wind_speed = wind_speed

        # Physical properties
        self.gravity = np.array([0, 0, -9.81])  # m/s²
        self.air_density = 1.225  # kg/m³
        self.rotor_area = 0.1  # m² (total for 4 rotors)

        # State
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)
        self.orientation = np.array([1, 0, 0, 0])  # Quaternion [w, x, y, z]
        self.angular_velocity = np.zeros(3)

        # Control inputs
        self.thrust = 0.0  # 0-1 normalized
        self.target_orientation = np.array([1, 0, 0, 0])

        self.logger = logging.getLogger(self.__class__.__name__)

    def set_position(self, position: np.ndarray):
        """Set drone position"""
        self.position = position.copy()

    def set_velocity(self, velocity: np.ndarray):
        """Set drone velocity"""
        self.velocity = velocity.copy()

    def set_orientation(self, orientation: np.ndarray):
        """Set drone orientation (quaternion)"""
        self.orientation = orientation.copy() / np.linalg.norm(orientation)  # Normalize

    def set_thrust(self, thrust: float):
        """Set thrust input (0-1)"""
        self.thrust = np.clip(thrust, 0.0, 1.0)

    def set_target_orientation(self, orientation: np.ndarray):
        """Set target orientation for control"""
        self.target_orientation = orientation.copy() / np.linalg.norm(orientation)

    @profile_function("physics.drone_step")
    def step(self, dt: float, wind_vector: np.ndarray = None):
        """Perform one physics simulation step"""

        if wind_vector is None:
            wind_vector = np.array([self.wind_speed, 0, 0])  # Wind along x-axis

        # Calculate forces
        forces = self._calculate_forces(wind_vector)

        # Calculate torques (simplified)
        torques = self._calculate_torques()

        # Integrate linear motion
        self.acceleration = forces / self.mass
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt

        # Integrate angular motion (simplified)
        angular_acceleration = torques / (self.mass * 0.1)  # Simplified inertia
        self.angular_velocity += angular_acceleration * dt

        # Update orientation (simplified - small angle approximation)
        angle = np.linalg.norm(self.angular_velocity) * dt
        if angle > 0:
            axis = self.angular_velocity / np.linalg.norm(self.angular_velocity)
            self._rotate_quaternion(axis, angle)

        # Apply damping
        self.velocity *= 0.99  # Air resistance
        self.angular_velocity *= 0.95  # Angular damping

        # Ensure orientation stays normalized
        self.orientation /= np.linalg.norm(self.orientation)

    def _calculate_forces(self, wind_vector: np.ndarray) -> np.ndarray:
        """Calculate all forces acting on the drone"""

        forces = np.zeros(3)

        # Thrust force (along local z-axis)
        thrust_force = self.thrust * self.max_thrust
        local_thrust = np.array([0, 0, thrust_force])

        # Rotate to world coordinates
        world_thrust = self._rotate_vector_by_quaternion(local_thrust, self.orientation)
        forces += world_thrust

        # Gravity
        forces += self.gravity * self.mass

        # Drag force
        relative_velocity = self.velocity - wind_vector
        speed = np.linalg.norm(relative_velocity)

        if speed > 0:
            drag_magnitude = 0.5 * self.air_density * speed**2 * self.drag_coefficient * self.rotor_area
            drag_direction = -relative_velocity / speed
            drag_force = drag_magnitude * drag_direction
            forces += drag_force

        # Lift force (simplified - perpendicular to velocity)
        if speed > 0:
            lift_magnitude = 0.5 * self.air_density * speed**2 * 0.1 * self.rotor_area  # Simplified lift coefficient
            lift_direction = np.array([0, 0, 1])  # Upward
            # Rotate lift based on orientation
            lift_force = lift_magnitude * self._rotate_vector_by_quaternion(lift_direction, self.orientation)
            forces += lift_force

        return forces

    def _calculate_torques(self) -> np.ndarray:
        """Calculate torques for attitude control (simplified)"""

        # Simple proportional control to target orientation
        current_orientation = self.orientation
        target_orientation = self.target_orientation

        # Quaternion error
        error_quat = self._quaternion_multiply(
            target_orientation,
            self._quaternion_conjugate(current_orientation)
        )

        # Convert to axis-angle for torque calculation
        angle = 2 * np.arccos(np.clip(error_quat[0], -1, 1))
        if angle > 0:
            axis = error_quat[1:] / np.sin(angle / 2)
            torque_magnitude = angle * 0.1  # Proportional gain
            return axis * torque_magnitude

        return np.zeros(3)

    def _rotate_vector_by_quaternion(self, vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
        """Rotate vector by quaternion"""
        q = quaternion
        v = np.array([0, vector[0], vector[1], vector[2]])

        # q * v * q_conjugate
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
        rotated = self._quaternion_multiply(
            self._quaternion_multiply(q, v),
            q_conj
        )

        return rotated[1:]

    def _quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def _quaternion_conjugate(self, q: np.ndarray) -> np.ndarray:
        """Get quaternion conjugate"""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def _rotate_quaternion(self, axis: np.ndarray, angle: float):
        """Rotate quaternion around axis by angle"""
        axis = axis / np.linalg.norm(axis)
        half_angle = angle / 2
        rotation_quat = np.array([
            np.cos(half_angle),
            axis[0] * np.sin(half_angle),
            axis[1] * np.sin(half_angle),
            axis[2] * np.sin(half_angle)
        ])

        self.orientation = self._quaternion_multiply(rotation_quat, self.orientation)

    def get_state(self) -> Dict:
        """Get current physical state"""
        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'acceleration': self.acceleration.copy(),
            'orientation': self.orientation.copy(),
            'angular_velocity': self.angular_velocity.copy(),
            'thrust': self.thrust
        }


class SwarmPhysics:
    """Physics simulation for entire drone swarm"""

    def __init__(self, num_drones: int = 5):
        self.drones: Dict[int, DronePhysics] = {}
        self.wind_field = WindField()

        # Initialize drones
        for i in range(num_drones):
            drone = DronePhysics()
            # Random initial positions
            drone.set_position(np.array([
                np.random.uniform(0, 10),
                np.random.uniform(0, 10),
                np.random.uniform(5, 15)
            ]))
            self.drones[i] = drone

    def step(self, dt: float):
        """Step physics simulation for all drones"""

        # Update wind field
        self.wind_field.update(dt)

        # Step each drone
        for drone_id, drone in self.drones.items():
            # Get local wind
            wind = self.wind_field.get_wind_at_position(drone.position)
            drone.step(dt, wind)

    def set_drone_thrust(self, drone_id: int, thrust: float):
        """Set thrust for specific drone"""
        if drone_id in self.drones:
            self.drones[drone_id].set_thrust(thrust)

    def set_drone_target(self, drone_id: int, target_pos: np.ndarray):
        """Set target position for drone control"""
        if drone_id in self.drones:
            drone = self.drones[drone_id]
            # Simple direction-based control
            direction = target_pos - drone.position
            if np.linalg.norm(direction) > 0:
                direction /= np.linalg.norm(direction)
                # Set target orientation to face direction
                drone.set_target_orientation(np.array([1, 0, 0, 0]))  # Simplified

    def get_swarm_state(self) -> Dict[int, Dict]:
        """Get state of all drones"""
        return {drone_id: drone.get_state() for drone_id, drone in self.drones.items()}

    def add_obstacle_force(self, position: np.ndarray, force_field: callable):
        """Add obstacle avoidance forces (to be implemented)"""
        pass

    def check_collisions(self) -> List[Tuple[int, int]]:
        """Check for collisions between drones"""
        collisions = []
        drone_ids = list(self.drones.keys())

        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                drone1 = self.drones[drone_ids[i]]
                drone2 = self.drones[drone_ids[j]]

                distance = np.linalg.norm(drone1.position - drone2.position)
                if distance < 1.0:  # Collision threshold
                    collisions.append((drone_ids[i], drone_ids[j]))

        return collisions


class WindField:
    """Dynamic wind field simulation"""

    def __init__(self, base_speed: float = 2.0, turbulence: float = 0.5):
        self.base_speed = base_speed
        self.turbulence = turbulence
        self.time = 0.0

        # Wind varies with position and time
        self.wind_grid = {}
        self.grid_resolution = 5.0  # meters

    def update(self, dt: float):
        """Update wind field over time"""
        self.time += dt

    def get_wind_at_position(self, position: np.ndarray) -> np.ndarray:
        """Get wind vector at specific position"""

        # Base wind direction (along x-axis with some variation)
        base_wind = np.array([
            self.base_speed + np.sin(self.time * 0.1) * 0.5,
            np.cos(self.time * 0.05) * 0.3,
            np.sin(self.time * 0.08) * 0.2
        ])

        # Add turbulence based on position
        turbulence_x = np.sin(position[0] * 0.1 + self.time) * self.turbulence
        turbulence_y = np.cos(position[1] * 0.1 + self.time * 1.3) * self.turbulence
        turbulence_z = np.sin(position[2] * 0.05 + self.time * 0.7) * self.turbulence * 0.5

        turbulence = np.array([turbulence_x, turbulence_y, turbulence_z])

        return base_wind + turbulence

    def get_wind_field(self, bounds: Tuple[float, float, float, float, float, float]) -> Dict:
        """Get wind field over a region (for visualization)"""
        # Simplified - return base wind for now
        return {
            'base_speed': self.base_speed,
            'turbulence': self.turbulence,
            'time': self.time
        }


class Environment:
    """Environmental simulation including obstacles and boundaries"""

    def __init__(self, bounds: Tuple[float, float, float]):
        self.bounds = bounds  # (width, height, depth)
        self.obstacles: List[Tuple[np.ndarray, float]] = []  # (position, radius)
        self.no_fly_zones: List[Tuple[np.ndarray, np.ndarray]] = []  # (min_corner, max_corner)

    def add_obstacle(self, position: np.ndarray, radius: float):
        """Add spherical obstacle"""
        self.obstacles.append((position.copy(), radius))

    def add_no_fly_zone(self, min_corner: np.ndarray, max_corner: np.ndarray):
        """Add rectangular no-fly zone"""
        self.no_fly_zones.append((min_corner.copy(), max_corner.copy()))

    def is_position_valid(self, position: np.ndarray) -> bool:
        """Check if position is within bounds and not in obstacles/zones"""

        # Check bounds
        if not (0 <= position[0] <= self.bounds[0] and
                0 <= position[1] <= self.bounds[1] and
                0 <= position[2] <= self.bounds[2]):
            return False

        # Check obstacles
        for obs_pos, obs_radius in self.obstacles:
            if np.linalg.norm(position - obs_pos) <= obs_radius:
                return False

        # Check no-fly zones
        for min_corner, max_corner in self.no_fly_zones:
            if np.all(position >= min_corner) and np.all(position <= max_corner):
                return False

        return True

    def get_obstacle_forces(self, position: np.ndarray, influence_radius: float = 5.0) -> np.ndarray:
        """Calculate repulsive forces from obstacles"""

        total_force = np.zeros(3)

        for obs_pos, obs_radius in self.obstacles:
            diff = position - obs_pos
            distance = np.linalg.norm(diff)

            if distance < influence_radius and distance > obs_radius:
                # Repulsive force inversely proportional to distance
                force_magnitude = 10.0 / (distance - obs_radius + 1.0)**2
                force_direction = diff / distance
                total_force += force_direction * force_magnitude

        return total_force
