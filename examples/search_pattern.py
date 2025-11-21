#!/usr/bin/env python3
"""
Drone Swarm Search Pattern Example

This example demonstrates how to coordinate a drone swarm for
area search and surveillance missions using systematic patterns.
"""

import math
import os
import sys
import time

import matplotlib.pyplot as plt
import numpy as np

# Add backend to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend'))

from coordinator import Formation, SwarmConfig, SwarmCoordinator
from pathfinding import Pathfinding3D


class SearchPattern:
    """Base class for search patterns"""

    def __init__(self, bounds, num_drones):
        self.bounds = bounds  # (width, height, altitude)
        self.num_drones = num_drones
        self.waypoints = []

    def generate_pattern(self):
        """Generate search waypoints"""
        raise NotImplementedError

    def get_drone_waypoints(self, drone_id):
        """Get waypoints for specific drone"""
        if not self.waypoints:
            self.generate_pattern()

        # Distribute waypoints among drones
        drone_waypoints = []
        for i, waypoint in enumerate(self.waypoints):
            if i % self.num_drones == drone_id:
                drone_waypoints.append(waypoint)

        return drone_waypoints


class LawnMowerPattern(SearchPattern):
    """Classic lawn mower search pattern"""

    def __init__(self, bounds, num_drones, overlap=0.1):
        super().__init__(bounds, num_drones)
        self.overlap = overlap  # Overlap between adjacent lanes

    def generate_pattern(self):
        width, height, altitude = self.bounds
        lane_spacing = 5.0  # Distance between parallel lanes
        effective_width = width * (1 - self.overlap)

        # Calculate number of lanes needed
        num_lanes = int(math.ceil(height / lane_spacing))

        self.waypoints = []
        direction = 1  # 1 for right, -1 for left

        for lane in range(num_lanes):
            y_pos = lane * lane_spacing

            if direction == 1:
                # Left to right
                start_x, end_x = 0, effective_width
            else:
                # Right to left
                start_x, end_x = effective_width, 0

            # Add waypoints along this lane
            num_points = int(math.ceil(abs(end_x - start_x) / 2.0))
            for i in range(num_points):
                x_pos = start_x + (end_x - start_x) * i / (num_points - 1)
                self.waypoints.append([x_pos, y_pos, altitude])

            direction *= -1  # Reverse direction for next lane


class SpiralPattern(SearchPattern):
    """Spiral search pattern from center outward"""

    def __init__(self, bounds, num_drones, turns=3):
        super().__init__(bounds, num_drones)
        self.turns = turns

    def generate_pattern(self):
        width, height, altitude = self.bounds
        center_x, center_y = width / 2, height / 2

        # Spiral parameters
        max_radius = min(width, height) / 2 * 0.8
        points_per_turn = 20
        total_points = points_per_turn * self.turns

        self.waypoints = []

        for i in range(total_points):
            # Calculate angle and radius
            angle = (i / points_per_turn) * 2 * math.pi
            radius = (i / total_points) * max_radius

            # Convert to cartesian
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)

            # Keep within bounds
            x = max(0, min(width, x))
            y = max(0, min(height, y))

            self.waypoints.append([x, y, altitude])


def search_pattern_demo():
    """Demonstrate different search patterns"""

    print("Drone Swarm Search Pattern Demo")
    print("=" * 45)

    # Define search area
    search_bounds = (80, 60, 15)  # width, height, altitude
    num_drones = 6

    print(f"Search area: {search_bounds[0]}m x {search_bounds[1]}m")
    print(f"Number of drones: {num_drones}")

    # Create pathfinding environment
    pathfinder = Pathfinding3D(bounds=search_bounds)

    # Add some obstacles
    obstacles = [
        ([25, 20, 8], 3),
        ([55, 40, 10], 4),
        ([15, 45, 6], 2)
    ]

    for pos, radius in obstacles:
        pathfinder.add_obstacle(pos, radius)
        print(f"Added obstacle at {pos} with radius {radius}")

    # Test different search patterns
    patterns = [
        ("Lawn Mower Pattern", LawnMowerPattern(search_bounds, num_drones)),
        ("Spiral Pattern", SpiralPattern(search_bounds, num_drones, turns=4))
    ]

    for pattern_name, pattern in patterns:
        print(f"\nTesting {pattern_name}")

        # Generate pattern waypoints
        pattern.generate_pattern()
        print(f"  Generated {len(pattern.waypoints)} total waypoints")

        # Create visualization
        fig = plt.figure(figsize=(15, 10))

        # 3D view
        ax3d = fig.add_subplot(121, projection='3d')

        # Plot obstacles
        for pos, radius in obstacles:
            # Simple obstacle representation
            ax3d.scatter([pos[0]], [pos[1]], [pos[2]],
                        c='red', s=radius*50, alpha=0.6)

        # Plot search waypoints
        if pattern.waypoints:
            wp_x = [p[0] for p in pattern.waypoints]
            wp_y = [p[1] for p in pattern.waypoints]
            wp_z = [p[2] for p in pattern.waypoints]

            ax3d.plot(wp_x, wp_y, wp_z, 'b-', alpha=0.7, linewidth=2)
            ax3d.scatter(wp_x, wp_y, wp_z, c='blue', s=20, alpha=0.8)

        ax3d.set_xlabel('X (m)')
        ax3d.set_ylabel('Y (m)')
        ax3d.set_zlabel('Z (m)')
        ax3d.set_title(f'{pattern_name} - 3D View')
        ax3d.set_xlim([0, search_bounds[0]])
        ax3d.set_ylim([0, search_bounds[1]])
        ax3d.set_zlim([0, search_bounds[2]])

        # 2D top view
        ax2d = fig.add_subplot(122)

        # Plot search area boundary
        boundary = plt.Rectangle((0, 0), search_bounds[0], search_bounds[1],
                               fill=False, color='black', linewidth=2)
        ax2d.add_patch(boundary)

        # Plot obstacles (2D)
        for pos, radius in obstacles:
            obstacle = plt.Circle((pos[0], pos[1]), radius,
                                color='red', alpha=0.6)
            ax2d.add_patch(obstacle)

        # Plot waypoints (2D)
        if pattern.waypoints:
            wp_x = [p[0] for p in pattern.waypoints]
            wp_y = [p[1] for p in pattern.waypoints]

            # Connect waypoints with lines
            ax2d.plot(wp_x, wp_y, 'b-', alpha=0.7, linewidth=2)
            ax2d.scatter(wp_x, wp_y, c='blue', s=30, alpha=0.8, zorder=5)

            # Label start and end
            if wp_x and wp_y:
                ax2d.scatter(wp_x[0], wp_y[0], c='green', s=100,
                           marker='^', zorder=10, label='Start')
                ax2d.scatter(wp_x[-1], wp_y[-1], c='orange', s=100,
                           marker='v', zorder=10, label='End')

        ax2d.set_xlabel('X (m)')
        ax2d.set_ylabel('Y (m)')
        ax2d.set_title(f'{pattern_name} - Top View')
        ax2d.set_xlim([-5, search_bounds[0] + 5])
        ax2d.set_ylim([-5, search_bounds[1] + 5])
        ax2d.set_aspect('equal')
        ax2d.grid(True, alpha=0.3)
        ax2d.legend()

        plt.tight_layout()
        plt.show()

        # Simulate drone allocation
        print(f"  Drone waypoint distribution:")
        total_assigned = 0
        for drone_id in range(num_drones):
            drone_waypoints = pattern.get_drone_waypoints(drone_id)
            print(f"    Drone {drone_id}: {len(drone_waypoints)} waypoints")
            total_assigned += len(drone_waypoints)

        print(f"  Total waypoints assigned: {total_assigned}")


def coordinated_search_demo():
    """Demonstrate coordinated search with formation changes"""

    print("\nCoordinated Search with Formation Control")
    print("=" * 50)

    # Create swarm configuration
    config = SwarmConfig(
        num_drones=8,
        formation=Formation.LINE,
        separation_distance=4.0,
        max_velocity=1.5
    )

    coordinator = SwarmCoordinator(config)

    # Define search mission phases
    mission_phases = [
        {
            'name': 'Deployment',
            'formation': Formation.LINE,
            'duration': 30,  # steps
            'description': 'Deploy in line formation'
        },
        {
            'name': 'Area Search',
            'formation': Formation.GRID,
            'duration': 60,
            'description': 'Search in grid pattern'
        },
        {
            'name': 'Target Investigation',
            'formation': Formation.CIRCLE,
            'duration': 20,
            'description': 'Circle around suspected target'
        },
        {
            'name': 'Return to Base',
            'formation': Formation.V_FORMATION,
            'duration': 40,
            'description': 'Return in V-formation'
        }
    ]

    print(f"Starting coordinated search mission with {config.num_drones} drones")

    for phase in mission_phases:
        print(f"\nPhase: {phase['name']}")
        print(f"   {phase['description']}")
        print(f"   Formation: {phase['formation'].value}")

        # Change formation
        coordinator.set_formation(phase['formation'])

        # Simulate phase
        for step in range(phase['duration']):
            state = coordinator.step(dt=0.1)

            if step % 10 == 0:
                active_drones = len([d for d in state['drones'].values() if d['is_active']])
                print(f"   Step {step}: {active_drones}/{config.num_drones} drones active, "
                      f"Leader: {state['leader_id']}")

    print("\nCoordinated search mission completed!")


if __name__ == "__main__":
    try:
        search_pattern_demo()
        coordinated_search_demo()
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure you're running this from the examples directory")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
