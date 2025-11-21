"""
Drone Swarm Coordination System
Implements consensus algorithms, pathfinding, and collision avoidance for multi-drone coordination
"""

import asyncio
import logging
import math
import time
from collections import defaultdict
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

# Import profiling
from backend.profiling import (
    SwarmProfiler,
    get_profiler,
    profile_collision_detection,
    profile_consensus,
    profile_function,
    profile_pathfinding,
    profile_physics,
)


class ConsensusState(Enum):
    FOLLOWER = "follower"
    CANDIDATE = "candidate"
    LEADER = "leader"


class Formation(Enum):
    CIRCLE = "circle"
    LINE = "line"
    GRID = "grid"
    V_FORMATION = "v_formation"
    RANDOM = "random"


@dataclass
class DroneState:
    """Represents the state of a single drone"""
    id: int
    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    orientation: np.ndarray  # Quaternion [w, x, y, z]
    battery_level: float  # 0.0 to 1.0
    is_active: bool = True
    last_update: float = 0.0

    def to_dict(self) -> Dict:
        return {
            'id': self.id,
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist(),
            'orientation': self.orientation.tolist(),
            'battery_level': self.battery_level,
            'is_active': self.is_active,
            'last_update': self.last_update
        }


@dataclass
class SwarmConfig:
    """Configuration for swarm behavior"""
    num_drones: int = 5
    separation_distance: float = 5.0
    max_velocity: float = 2.0
    communication_range: float = 20.0
    collision_threshold: float = 2.0
    consensus_timeout: float = 5.0
    heartbeat_interval: float = 1.0
    formation: Formation = Formation.CIRCLE


class ConsensusAlgorithm:
    """Raft-based consensus algorithm for drone swarm coordination"""

    def __init__(self, drone_id: int, config: SwarmConfig):
        self.drone_id = drone_id
        self.config = config
        self.state = ConsensusState.FOLLOWER
        self.current_term = 0
        self.voted_for = None
        self.log = []
        self.commit_index = 0
        self.last_applied = 0

        # Leader election
        self.votes_received = 0
        self.election_timeout = np.random.uniform(1.0, 2.0)
        self.last_heartbeat = time.time()

        # Peer management
        self.peers: Dict[int, float] = {}  # peer_id -> last_seen

        self.logger = logging.getLogger(f"Consensus-{drone_id}")

    def receive_heartbeat(self, leader_id: int, term: int):
        """Handle heartbeat from leader"""
        if term >= self.current_term:
            self.current_term = term
            self.state = ConsensusState.FOLLOWER
            self.voted_for = leader_id
            self.last_heartbeat = time.time()
            self.logger.debug(f"Received heartbeat from leader {leader_id}")

    def start_election(self):
        """Start leader election"""
        self.current_term += 1
        self.state = ConsensusState.CANDIDATE
        self.voted_for = self.drone_id
        self.votes_received = 1  # Vote for self
        self.last_heartbeat = time.time()
        self.logger.info(f"Starting election for term {self.current_term}")

    def request_vote(self, candidate_id: int, term: int) -> bool:
        """Respond to vote request"""
        if term > self.current_term:
            self.current_term = term
            self.state = ConsensusState.FOLLOWER
            self.voted_for = None

        if term == self.current_term and (self.voted_for is None or self.voted_for == candidate_id):
            self.voted_for = candidate_id
            self.logger.debug(f"Voted for candidate {candidate_id}")
            return True

        return False

    def receive_vote(self, voter_id: int):
        """Receive vote from peer"""
        if self.state == ConsensusState.CANDIDATE:
            self.votes_received += 1
            majority = (self.config.num_drones // 2) + 1

            if self.votes_received >= majority:
                self.state = ConsensusState.LEADER
                self.logger.info(f"Elected as leader for term {self.current_term}")
                # Send initial heartbeat
                self.last_heartbeat = time.time()

    def step(self, current_time: float) -> Optional[Dict]:
        """Perform one step of consensus algorithm"""
        if self.state == ConsensusState.LEADER:
            # Send heartbeats
            if current_time - self.last_heartbeat >= self.config.heartbeat_interval:
                self.last_heartbeat = current_time
                return {
                    'type': 'heartbeat',
                    'term': self.current_term,
                    'leader_id': self.drone_id
                }

        elif self.state in [ConsensusState.FOLLOWER, ConsensusState.CANDIDATE]:
            # Check for election timeout
            if current_time - self.last_heartbeat >= self.election_timeout:
                self.start_election()
                return {
                    'type': 'vote_request',
                    'term': self.current_term,
                    'candidate_id': self.drone_id
                }

        return None

    def is_leader(self) -> bool:
        return self.state == ConsensusState.LEADER


class PathfindingAlgorithm:
    """A* pathfinding with RRT* for 3D navigation"""

    def __init__(self, bounds: Tuple[float, float, float]):
        self.bounds = bounds  # (width, height, depth)
        self.obstacles: List[np.ndarray] = []

    def add_obstacle(self, position: np.ndarray, radius: float):
        """Add spherical obstacle"""
        obstacle = np.concatenate([position, [radius]])
        self.obstacles.append(obstacle)

    def clear_obstacles(self):
        """Remove all obstacles"""
        self.obstacles.clear()

    def is_valid_position(self, position: np.ndarray) -> bool:
        """Check if position is valid (within bounds and not in obstacle)"""
        # Check bounds
        if not (0 <= position[0] <= self.bounds[0] and
                0 <= position[1] <= self.bounds[1] and
                0 <= position[2] <= self.bounds[2]):
            return False

        # Check obstacles
        for obstacle in self.obstacles:
            center = obstacle[:3]
            radius = obstacle[3]
            if np.linalg.norm(position - center) <= radius:
                return False

        return True

    def find_path_astar(self, start: np.ndarray, goal: np.ndarray) -> List[np.ndarray]:
        """Find path using A* algorithm"""
        class Node:
            def __init__(self, position, g_cost=0, h_cost=0, parent=None):
                self.position = position
                self.g_cost = g_cost
                self.h_cost = h_cost
                self.f_cost = g_cost + h_cost
                self.parent = parent

        def heuristic(a, b):
            return np.linalg.norm(a - b)

        def get_neighbors(pos):
            neighbors = []
            step = 1.0
            for dx in [-step, 0, step]:
                for dy in [-step, 0, step]:
                    for dz in [-step, 0, step]:
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        neighbor = pos + np.array([dx, dy, dz])
                        if self.is_valid_position(neighbor):
                            neighbors.append(neighbor)
            return neighbors

        open_set = [Node(start, h_cost=heuristic(start, goal))]
        closed_set = set()

        while open_set:
            # Find node with lowest f_cost
            current = min(open_set, key=lambda x: x.f_cost)
            open_set.remove(current)
            closed_set.add(tuple(current.position))

            # Check if goal reached
            if np.linalg.norm(current.position - goal) < 1.0:
                # Reconstruct path
                path = []
                node = current
                while node:
                    path.append(node.position)
                    node = node.parent
                return path[::-1]  # Reverse path

            # Generate neighbors
            for neighbor_pos in get_neighbors(current.position):
                if tuple(neighbor_pos) in closed_set:
                    continue

                g_cost = current.g_cost + np.linalg.norm(neighbor_pos - current.position)
                h_cost = heuristic(neighbor_pos, goal)

                # Check if neighbor is in open set
                existing = next((n for n in open_set if np.array_equal(n.position, neighbor_pos)), None)
                if existing:
                    if g_cost < existing.g_cost:
                        existing.g_cost = g_cost
                        existing.f_cost = g_cost + existing.h_cost
                        existing.parent = current
                else:
                    open_set.append(Node(neighbor_pos, g_cost, h_cost, current))

        return []  # No path found

    def find_path_rrt_star(self, start: np.ndarray, goal: np.ndarray,
                          max_iterations: int = 1000) -> List[np.ndarray]:
        """Find path using RRT* algorithm"""
        class Node:
            def __init__(self, position, parent=None, cost=0):
                self.position = position
                self.parent = parent
                self.cost = cost
                self.children = []

        def distance(a, b):
            return np.linalg.norm(a - b)

        def nearest_node(point):
            return min(tree, key=lambda n: distance(n.position, point))

        def steer(from_pos, to_pos, max_distance=2.0):
            direction = to_pos - from_pos
            dist = np.linalg.norm(direction)
            if dist > max_distance:
                direction = direction * (max_distance / dist)
            return from_pos + direction

        def is_collision_free(a, b):
            # Simple line collision check
            steps = 10
            for i in range(steps + 1):
                t = i / steps
                point = a + t * (b - a)
                if not self.is_valid_position(point):
                    return False
            return True

        tree = [Node(start)]
        goal_node = None

        for _ in range(max_iterations):
            # Sample random point
            if np.random.random() < 0.1:  # 10% chance to sample goal
                random_point = goal.copy()
            else:
                random_point = np.array([
                    np.random.uniform(0, self.bounds[0]),
                    np.random.uniform(0, self.bounds[1]),
                    np.random.uniform(0, self.bounds[2])
                ])

            # Find nearest node
            nearest = nearest_node(random_point)

            # Steer towards random point
            new_point = steer(nearest.position, random_point)

            # Check collision
            if not is_collision_free(nearest.position, new_point):
                continue

            # Create new node
            new_node = Node(new_point, nearest, nearest.cost + distance(nearest.position, new_point))
            nearest.children.append(new_node)
            tree.append(new_node)

            # Check if close to goal
            if distance(new_point, goal) < 2.0 and is_collision_free(new_point, goal):
                goal_node = Node(goal, new_node, new_node.cost + distance(new_point, goal))
                new_node.children.append(goal_node)
                tree.append(goal_node)
                break

        # Reconstruct path
        if goal_node is None:
            return []

        path = []
        current = goal_node
        while current:
            path.append(current.position)
            current = current.parent
        return path[::-1]


class CollisionAvoidance:
    """Artificial potential fields for collision avoidance"""

    def __init__(self, config: SwarmConfig):
        self.config = config
        self.repulsive_gain = 10.0
        self.attractive_gain = 1.0
        self.drone_radius = 0.5

    def calculate_force(self, drone_pos: np.ndarray, target_pos: np.ndarray,
                       neighbors: List[np.ndarray], obstacles: List[np.ndarray]) -> np.ndarray:
        """Calculate total force using potential fields"""

        force = np.zeros(3)

        # Attractive force to target
        attractive_force = self.attractive_gain * (target_pos - drone_pos)
        force += attractive_force

        # Repulsive forces from neighbors
        for neighbor_pos in neighbors:
            diff = drone_pos - neighbor_pos
            distance = np.linalg.norm(diff)

            if distance < self.config.collision_threshold and distance > 0:
                # Repulsive force inversely proportional to distance
                repulsive_force = self.repulsive_gain * (diff / distance) * (1/distance - 1/self.config.collision_threshold)
                force += repulsive_force

        # Repulsive forces from obstacles
        for obstacle in obstacles:
            obstacle_pos = obstacle[:3]  # Position part of obstacle array
            obstacle_radius = obstacle[3]  # Radius part of obstacle array
            diff = drone_pos - obstacle_pos
            distance = np.linalg.norm(diff)

            if distance < obstacle_radius + self.drone_radius:  # obstacle radius + drone radius
                repulsive_force = self.repulsive_gain * (diff / distance) / (distance ** 2)
                force += repulsive_force

        return force

    def avoid_collisions(self, swarm_positions: Dict[int, np.ndarray],
                        obstacles: List[np.ndarray]) -> Dict[int, np.ndarray]:
        """Calculate avoidance forces for all drones"""

        avoidance_forces = {}

        for drone_id, position in swarm_positions.items():
            neighbors = []
            for other_id, other_pos in swarm_positions.items():
                if other_id != drone_id:
                    neighbors.append(other_pos)

            force = self.calculate_force(position, position, neighbors, obstacles)
            avoidance_forces[drone_id] = force

        return avoidance_forces


class SwarmCoordinator:
    """Main coordinator for drone swarm operations"""

    def __init__(self, config: SwarmConfig):
        self.config = config
        self.drones: Dict[int, DroneState] = {}
        self.consensus_algorithms: Dict[int, ConsensusAlgorithm] = {}
        self.pathfinder = PathfindingAlgorithm((100, 100, 50))  # 100x100x50 meter space
        self.collision_avoidance = CollisionAvoidance(config)

        # Swarm state
        self.formation = config.formation
        self.target_positions: Dict[int, np.ndarray] = {}
        self.paths: Dict[int, List[np.ndarray]] = {}

        # Communication
        self.message_queue = asyncio.Queue()
        self.logger = logging.getLogger("SwarmCoordinator")

        # Initialize drones
        self._initialize_drones()

    def _initialize_drones(self):
        """Initialize drone states and consensus algorithms"""
        for i in range(self.config.num_drones):
            # Random initial positions
            position = np.array([
                np.random.uniform(10, 90),
                np.random.uniform(10, 90),
                np.random.uniform(5, 15)
            ])

            drone = DroneState(
                id=i,
                position=position,
                velocity=np.zeros(3),
                orientation=np.array([1, 0, 0, 0]),  # Identity quaternion
                battery_level=1.0,
                last_update=time.time()
            )

            self.drones[i] = drone
            self.consensus_algorithms[i] = ConsensusAlgorithm(i, self.config)

        self._calculate_formation()

    def _calculate_formation(self):
        """Calculate target positions for current formation"""
        center = np.array([50, 50, 10])  # Center of the space

        if self.formation == Formation.CIRCLE:
            radius = self.config.separation_distance * self.config.num_drones / (2 * np.pi)
            for i in range(self.config.num_drones):
                angle = 2 * np.pi * i / self.config.num_drones
                x = center[0] + radius * np.cos(angle)
                y = center[1] + radius * np.sin(angle)
                z = center[2] + i * 2  # Slight height variation
                self.target_positions[i] = np.array([x, y, z])

        elif self.formation == Formation.LINE:
            for i in range(self.config.num_drones):
                x = center[0] - (self.config.num_drones - 1) * self.config.separation_distance / 2 + i * self.config.separation_distance
                y = center[1]
                z = center[2]
                self.target_positions[i] = np.array([x, y, z])

        elif self.formation == Formation.GRID:
            grid_size = int(np.ceil(np.sqrt(self.config.num_drones)))
            for i in range(self.config.num_drones):
                row = i // grid_size
                col = i % grid_size
                x = center[0] + (col - grid_size/2) * self.config.separation_distance
                y = center[1] + (row - grid_size/2) * self.config.separation_distance
                z = center[2]
                self.target_positions[i] = np.array([x, y, z])

        elif self.formation == Formation.V_FORMATION:
            for i in range(self.config.num_drones):
                if self.config.num_drones % 2 == 0:
                    side = 1 if i < self.config.num_drones // 2 else -1
                    row = i % (self.config.num_drones // 2)
                else:
                    side = 1 if i <= self.config.num_drones // 2 else -1
                    row = min(i, self.config.num_drones - 1 - i)

                x = center[0] + row * self.config.separation_distance * 0.7
                y = center[1] + side * row * self.config.separation_distance * 0.5
                z = center[2]
                self.target_positions[i] = np.array([x, y, z])

        # Calculate paths to targets
        for drone_id, target_pos in self.target_positions.items():
            if drone_id in self.drones:
                current_pos = self.drones[drone_id].position
                with profile_pathfinding('astar'):
                    path = self.pathfinder.find_path_astar(current_pos, target_pos)
                if path:
                    self.paths[drone_id] = path

    def set_formation(self, formation: Formation):
        """Change swarm formation"""
        self.formation = formation
        self._calculate_formation()
        self.logger.info(f"Changed formation to {formation.value}")

    def add_obstacle(self, position: np.ndarray, radius: float):
        """Add obstacle to environment"""
        self.pathfinder.add_obstacle(position, radius)
        self.logger.info(f"Added obstacle at {position} with radius {radius}")

    @profile_function("coordinator.step")
    def step(self, dt: float) -> Dict[str, Any]:
        """Perform one simulation step"""
        current_time = time.time()

        # Run consensus algorithms
        consensus_messages = []
        leader_id = None

        with profile_consensus():
            for drone_id, consensus in self.consensus_algorithms.items():
                message = consensus.step(current_time)
                if message:
                    message['from'] = drone_id
                    consensus_messages.append(message)

                if consensus.is_leader():
                    leader_id = drone_id

        # Process consensus messages
        for message in consensus_messages:
            self._process_consensus_message(message)

        # Update drone positions
        swarm_positions = {id: drone.position for id, drone in self.drones.items()}
        obstacles = self.pathfinder.obstacles

        with profile_collision_detection():
            avoidance_forces = self.collision_avoidance.avoid_collisions(swarm_positions, obstacles)

        with profile_physics():
            for drone_id, drone in self.drones.items():
                if not drone.is_active:
                    continue

                # Get target position
                target_pos = self.target_positions.get(drone_id, drone.position)

                # Path following
                if drone_id in self.paths and self.paths[drone_id]:
                    next_waypoint = self.paths[drone_id][0]
                    if np.linalg.norm(drone.position - next_waypoint) < 1.0:
                        self.paths[drone_id].pop(0)
                    else:
                        target_pos = next_waypoint

                # Calculate desired velocity
                direction = target_pos - drone.position
                distance = np.linalg.norm(direction)

                if distance > 0.1:
                    desired_velocity = direction / distance * min(self.config.max_velocity, distance / dt)
                else:
                    desired_velocity = np.zeros(3)

                # Apply collision avoidance
                if drone_id in avoidance_forces:
                    desired_velocity += avoidance_forces[drone_id] * dt

                # Update velocity with smoothing
                smoothing_factor = 0.1
                drone.velocity = drone.velocity * (1 - smoothing_factor) + desired_velocity * smoothing_factor

                # Limit velocity
                speed = np.linalg.norm(drone.velocity)
                if speed > self.config.max_velocity:
                    drone.velocity = drone.velocity * (self.config.max_velocity / speed)

                # Update position
                drone.position += drone.velocity * dt

                # Update battery (simplified)
                drone.battery_level = max(0, drone.battery_level - speed * dt * 0.001)
                drone.last_update = current_time

                # Deactivate if battery too low
                if drone.battery_level < 0.1:
                    drone.is_active = False

        # Return current state
        return {
            'drones': {id: drone.to_dict() for id, drone in self.drones.items()},
            'leader_id': leader_id,
            'formation': self.formation.value,
            'target_positions': {id: pos.tolist() for id, pos in self.target_positions.items()},
            'timestamp': current_time
        }

    def _process_consensus_message(self, message: Dict):
        """Process consensus algorithm messages"""
        msg_type = message['type']
        from_id = message['from']

        if msg_type == 'heartbeat':
            for drone_id, consensus in self.consensus_algorithms.items():
                if drone_id != from_id:
                    consensus.receive_heartbeat(message['leader_id'], message['term'])

        elif msg_type == 'vote_request':
            # Send vote response
            for drone_id, consensus in self.consensus_algorithms.items():
                if drone_id != from_id:
                    vote_granted = consensus.request_vote(message['candidate_id'], message['term'])
                    if vote_granted:
                        # Notify candidate of vote
                        candidate_consensus = self.consensus_algorithms[message['candidate_id']]
                        candidate_consensus.receive_vote(drone_id)

    def get_swarm_state(self) -> Dict:
        """Get current swarm state"""
        return {
            'drones': {id: drone.to_dict() for id, drone in self.drones.items()},
            'config': {
                'num_drones': self.config.num_drones,
                'separation_distance': self.config.separation_distance,
                'max_velocity': self.config.max_velocity,
                'formation': self.formation.value
            },
            'obstacles': [obs.tolist() for obs in self.pathfinder.obstacles],
            'timestamp': time.time()
        }

    async def run_simulation(self, duration: float, dt: float = 0.1):
        """Run simulation for specified duration"""
        steps = int(duration / dt)
        start_time = time.time()

        for step in range(steps):
            state = self.step(dt)

            # Log progress
            if step % 100 == 0:
                elapsed = time.time() - start_time
                progress = (step / steps) * 100
                self.logger.info(f"Progress: {progress:.1f}% (elapsed: {elapsed:.1f}s)")

            # Small delay to prevent CPU hogging
            await asyncio.sleep(0.001)

        self.logger.info("Simulation completed")

    def reset(self):
        """Reset swarm to initial state"""
        self.drones.clear()
        self.consensus_algorithms.clear()
        self.target_positions.clear()
        self.paths.clear()
        self._initialize_drones()
        self.logger.info("Swarm reset to initial state")
