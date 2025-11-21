"""
Pathfinding algorithms for drone navigation
Implements A* and RRT* algorithms for 3D path planning
"""

import heapq
import logging
from typing import List, Optional, Tuple

import numpy as np

# Import profiling
from backend.profiling import profile_function, profile_pathfinding


class Node:
    """Node for pathfinding algorithms"""

    def __init__(self, position: np.ndarray, g_cost: float = 0,
                 h_cost: float = 0, parent: Optional['Node'] = None):
        self.position = position
        self.g_cost = g_cost  # Cost from start
        self.h_cost = h_cost  # Heuristic cost to goal
        self.f_cost = g_cost + h_cost  # Total cost
        self.parent = parent

    def __lt__(self, other):
        return self.f_cost < other.f_cost


class Pathfinding3D:
    """3D pathfinding with obstacle avoidance"""

    def __init__(self, bounds: Tuple[float, float, float], resolution: float = 1.0):
        self.bounds = bounds  # (width, height, depth)
        self.resolution = resolution
        self.obstacles: List[Tuple[np.ndarray, float]] = []  # (position, radius)
        self.logger = logging.getLogger(self.__class__.__name__)

    def add_obstacle(self, position: np.ndarray, radius: float):
        """Add spherical obstacle"""
        self.obstacles.append((position.copy(), radius))

    def remove_obstacle(self, position: np.ndarray, radius: float):
        """Remove obstacle at position"""
        self.obstacles = [
            (pos, r) for pos, r in self.obstacles
            if not np.allclose(pos, position, atol=radius)
        ]

    def clear_obstacles(self):
        """Remove all obstacles"""
        self.obstacles.clear()

    def is_valid_position(self, position: np.ndarray) -> bool:
        """Check if position is valid"""
        # Check bounds
        if not (0 <= position[0] <= self.bounds[0] and
                0 <= position[1] <= self.bounds[1] and
                0 <= position[2] <= self.bounds[2]):
            return False

        # Check obstacles
        for obs_pos, obs_radius in self.obstacles:
            if np.linalg.norm(position - obs_pos) <= obs_radius:
                return False

        return True

    def get_neighbors(self, position: np.ndarray) -> List[np.ndarray]:
        """Get valid neighboring positions"""
        neighbors = []
        step = self.resolution

        # 26-connected neighborhood in 3D
        for dx in [-step, 0, step]:
            for dy in [-step, 0, step]:
                for dz in [-step, 0, step]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue

                    neighbor = position + np.array([dx, dy, dz])
                    if self.is_valid_position(neighbor):
                        neighbors.append(neighbor)

        return neighbors

    def heuristic(self, a: np.ndarray, b: np.ndarray) -> float:
        """Euclidean distance heuristic"""
        return np.linalg.norm(a - b)

    @profile_function("pathfinding.astar")
    def astar(self, start: np.ndarray, goal: np.ndarray) -> Optional[List[np.ndarray]]:
        """A* pathfinding algorithm"""
        if not self.is_valid_position(start) or not self.is_valid_position(goal):
            return None

        open_set = []
        closed_set = set()

        start_node = Node(start, h_cost=self.heuristic(start, goal))
        heapq.heappush(open_set, start_node)

        while open_set:
            current = heapq.heappop(open_set)

            # Check if goal reached
            if np.linalg.norm(current.position - goal) < self.resolution:
                # Reconstruct path
                path = []
                node = current
                while node:
                    path.append(node.position.copy())
                    node = node.parent
                return path[::-1]  # Reverse path

            # Mark as visited
            closed_set.add(tuple(current.position))

            # Explore neighbors
            for neighbor_pos in self.get_neighbors(current.position):
                if tuple(neighbor_pos) in closed_set:
                    continue

                g_cost = current.g_cost + self.heuristic(current.position, neighbor_pos)

                # Check if neighbor is already in open set with lower cost
                existing = None
                for node in open_set:
                    if np.allclose(node.position, neighbor_pos):
                        existing = node
                        break

                if existing:
                    if g_cost < existing.g_cost:
                        existing.g_cost = g_cost
                        existing.f_cost = g_cost + existing.h_cost
                        existing.parent = current
                        # Re-heapify (inefficient but works)
                        heapq.heapify(open_set)
                else:
                    h_cost = self.heuristic(neighbor_pos, goal)
                    neighbor_node = Node(neighbor_pos, g_cost, h_cost, current)
                    heapq.heappush(open_set, neighbor_node)

        return None  # No path found

    @profile_function("pathfinding.rrt_star")
    def rrt_star(self, start: np.ndarray, goal: np.ndarray,
                  max_iterations: int = 1000, search_radius: float = 5.0) -> Optional[List[np.ndarray]]:
        """RRT* pathfinding algorithm"""

        class RRTNode:
            def __init__(self, position: np.ndarray, parent: Optional['RRTNode'] = None, cost: float = 0):
                self.position = position.copy()
                self.parent = parent
                self.cost = cost
                self.children: List[RRTNode] = []

        def distance(a: np.ndarray, b: np.ndarray) -> float:
            return np.linalg.norm(a - b)

        def steer(from_pos: np.ndarray, to_pos: np.ndarray, max_distance: float = 2.0) -> np.ndarray:
            direction = to_pos - from_pos
            dist = np.linalg.norm(direction)
            if dist > max_distance:
                direction = direction * (max_distance / dist)
            return from_pos + direction

        def nearest_node(point: np.ndarray, tree: List[RRTNode]) -> RRTNode:
            return min(tree, key=lambda node: distance(node.position, point))

        def near_nodes(point: np.ndarray, tree: List[RRTNode], radius: float) -> List[RRTNode]:
            return [node for node in tree if distance(node.position, point) <= radius]

        def collision_free(a: np.ndarray, b: np.ndarray, steps: int = 10) -> bool:
            """Check if path between two points is collision free"""
            for i in range(steps + 1):
                t = i / steps
                point = a + t * (b - a)
                if not self.is_valid_position(point):
                    return False
            return True

        if not self.is_valid_position(start) or not self.is_valid_position(goal):
            return None

        tree = [RRTNode(start)]
        goal_node = None

        for iteration in range(max_iterations):
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
            nearest = nearest_node(random_point, tree)

            # Steer towards random point
            new_point = steer(nearest.position, random_point)

            # Check collision
            if not collision_free(nearest.position, new_point):
                continue

            # Find nearby nodes for rewiring
            near_nodes_list = near_nodes(new_point, tree, search_radius)

            # Find best parent
            best_parent = nearest
            best_cost = nearest.cost + distance(nearest.position, new_point)

            for near_node in near_nodes_list:
                if collision_free(near_node.position, new_point):
                    cost = near_node.cost + distance(near_node.position, new_point)
                    if cost < best_cost:
                        best_parent = near_node
                        best_cost = cost

            # Create new node
            new_node = RRTNode(new_point, best_parent, best_cost)
            best_parent.children.append(new_node)
            tree.append(new_node)

            # Rewire nearby nodes
            for near_node in near_nodes_list:
                if near_node == best_parent:
                    continue

                if collision_free(new_point, near_node.position):
                    new_cost = new_node.cost + distance(new_point, near_node.position)
                    if new_cost < near_node.cost:
                        # Remove from old parent
                        if near_node.parent:
                            near_node.parent.children.remove(near_node)

                        # Add to new parent
                        near_node.parent = new_node
                        near_node.cost = new_cost
                        new_node.children.append(near_node)

            # Check if close to goal
            if distance(new_point, goal) < search_radius and collision_free(new_point, goal):
                goal_cost = new_node.cost + distance(new_point, goal)
                goal_node = RRTNode(goal, new_node, goal_cost)
                new_node.children.append(goal_node)
                tree.append(goal_node)
                self.logger.info(f"RRT* found path after {iteration + 1} iterations")
                break

        # Reconstruct path
        if goal_node is None:
            self.logger.warning("RRT* failed to find path")
            return None

        path = []
        current = goal_node
        while current:
            path.append(current.position.copy())
            current = current.parent
        path.reverse()

        # Smooth path
        smoothed_path = self.smooth_path(path)
        return smoothed_path

    def smooth_path(self, path: List[np.ndarray], max_iterations: int = 10) -> List[np.ndarray]:
        """Smooth path by removing unnecessary waypoints"""
        if len(path) <= 2:
            return path

        smoothed = path.copy()

        for _ in range(max_iterations):
            changed = False
            i = 0
            while i < len(smoothed) - 2:
                # Check if we can connect i directly to i+2
                if self.collision_free(smoothed[i], smoothed[i + 2]):
                    # Remove intermediate point
                    smoothed.pop(i + 1)
                    changed = True
                else:
                    i += 1

            if not changed:
                break

        return smoothed

    def collision_free(self, a: np.ndarray, b: np.ndarray, steps: int = 10) -> bool:
        """Check if path between two points is collision free"""
        for i in range(steps + 1):
            t = i / steps
            point = a + t * (b - a)
            if not self.is_valid_position(point):
                return False
        return True

    def find_path(self, start: np.ndarray, goal: np.ndarray,
                  algorithm: str = "astar") -> Optional[List[np.ndarray]]:
        """Find path using specified algorithm"""
        if algorithm == "astar":
            return self.astar(start, goal)
        elif algorithm == "rrt_star":
            return self.rrt_star(start, goal)
        else:
            raise ValueError(f"Unknown algorithm: {algorithm}")


class FormationPathPlanner:
    """Plans paths for drone formations"""

    def __init__(self, pathfinder: Pathfinding3D):
        self.pathfinder = pathfinder

    def plan_formation_paths(self, current_positions: dict,
                           target_positions: dict) -> dict:
        """Plan paths for multiple drones to reach formation positions"""
        paths = {}

        for drone_id in current_positions:
            if drone_id in target_positions:
                start = current_positions[drone_id]
                goal = target_positions[drone_id]

                # Try A* first, fallback to RRT*
                path = self.pathfinder.find_path(start, goal, "astar")
                if path is None:
                    path = self.pathfinder.find_path(start, goal, "rrt_star")

                if path:
                    paths[drone_id] = path
                else:
                    # Direct line if no path found
                    paths[drone_id] = [start, goal]

        return paths

    def optimize_formation_paths(self, paths: dict) -> dict:
        """Optimize paths to avoid conflicts and improve coordination"""
        # Simple optimization: ensure minimum separation
        optimized = {}

        for drone_id, path in paths.items():
            optimized_path = [path[0]]  # Start point

            for i in range(1, len(path)):
                current_point = path[i]
                # Check distance to other drones at this step
                conflict = False

                for other_id, other_path in paths.items():
                    if other_id != drone_id and i < len(other_path):
                        other_point = other_path[i]
                        if np.linalg.norm(current_point - other_point) < 2.0:
                            conflict = True
                            break

                if not conflict:
                    optimized_path.append(current_point)

            # Ensure we reach the goal
            if len(optimized_path) > 1:
                optimized_path[-1] = path[-1]

            optimized[drone_id] = optimized_path

        return optimized
