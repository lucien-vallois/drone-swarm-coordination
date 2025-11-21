#!/usr/bin/env python3
"""
Drone Swarm Formation Flight Example

This example demonstrates how to create and control a drone swarm
flying in different formations using the coordination simulator.
"""

import os
import sys
import time

import matplotlib.pyplot as plt

# Add backend to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend'))

from coordinator import Formation, SwarmConfig, SwarmCoordinator
from pathfinding import Pathfinding3D


def formation_flight_demo():
    """Demonstrate different formation flights"""

    print("Drone Swarm Formation Flight Demo")
    print("=" * 50)

    # Create swarm configuration
    config = SwarmConfig(
        num_drones=8,
        formation=Formation.CIRCLE,
        separation_distance=5.0,
        max_velocity=2.0,
        communication_range=15.0
    )

    # Create coordinator
    coordinator = SwarmCoordinator(config)

    print(f"Created swarm with {config.num_drones} drones")
    print(f"Initial formation: {config.formation.value}")

    # Initialize matplotlib for visualization
    plt.ion()
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    formations = [
        ("Circle Formation", Formation.CIRCLE),
        ("Line Formation", Formation.LINE),
        ("Grid Formation", Formation.GRID),
        ("V-Formation", Formation.V_FORMATION)
    ]

    for formation_name, formation_type in formations:
        print(f"\nSwitching to {formation_name}")

        # Change formation
        coordinator.set_formation(formation_type)

        # Simulate flight for a few steps
        positions = []
        for step in range(50):  # 5 seconds at 10Hz
            state = coordinator.step(dt=0.1)
            current_positions = [
                drone_data['position']
                for drone_data in state['drones'].values()
            ]
            positions.append(current_positions)

            if step % 10 == 0:
                print(f"  Step {step}: Leader={state['leader_id']}, "
                      f"Active drones={len([d for d in state['drones'].values() if d['is_active']])}")

        # Visualize final formation
        ax.clear()
        final_positions = positions[-1]

        # Plot drones
        x_coords = [pos[0] for pos in final_positions]
        y_coords = [pos[1] for pos in final_positions]
        z_coords = [pos[2] for pos in final_positions]

        ax.scatter(x_coords, y_coords, z_coords, c='green', s=100, alpha=0.8)

        # Add labels
        for i, (x, y, z) in enumerate(zip(x_coords, y_coords, z_coords)):
            ax.text(x, y, z, f'D{i}', fontsize=8)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'{formation_name} - Final Positions')
        ax.set_xlim([-20, 20])
        ax.set_ylim([-20, 20])
        ax.set_zlim([0, 15])

        plt.draw()
        plt.pause(2.0)  # Show formation for 2 seconds

    plt.ioff()
    plt.show()

    print("\nFormation flight demo completed!")
    print(f"Simulated {len(formations)} different formations")
    print(f"Total simulation time: {len(formations) * 5} seconds")


def pathfinding_formation_demo():
    """Demonstrate coordinated pathfinding for formations"""

    print("\nCoordinated Pathfinding Demo")
    print("=" * 40)

    # Create pathfinder
    pathfinder = Pathfinding3D(bounds=(100, 100, 50))

    # Add some obstacles
    obstacles = [
        ([30, 30, 10], 5),
        ([70, 70, 15], 3),
        ([50, 20, 8], 4)
    ]

    for pos, radius in obstacles:
        pathfinder.add_obstacle(pos, radius)
        print(f"Added obstacle at {pos} with radius {radius}")

    # Define formation waypoints
    waypoints = [
        [0, 0, 5],    # Start
        [40, 40, 10], # Waypoint 1
        [80, 20, 8],  # Waypoint 2
        [90, 90, 15]  # Goal
    ]

    print(f"Planning path through {len(waypoints)} waypoints")

    # Calculate path
    full_path = []
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        goal = waypoints[i + 1]

        segment_path = pathfinder.astar(start, goal)
        if segment_path:
            full_path.extend(segment_path[:-1])  # Avoid duplicating waypoints
        else:
            print(f"Could not find path from {start} to {goal}")
            return

    full_path.append(waypoints[-1])  # Add final waypoint

    print(f"Found path with {len(full_path)} waypoints")

    # Visualize path
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot obstacles
    for pos, radius in obstacles:
        # Create sphere for obstacle
        u, v = [], []
        for i in range(21):
            for j in range(21):
                u.append(i * 2 * 3.14159 / 20)
                v.append(j * 3.14159 / 10)
        x = pos[0] + radius * [i[0] * i[1] for i in zip([1]*441, u)]
        y = pos[1] + radius * [i[0] * i[1] for i in zip([1]*441, v)]
        z = pos[2] + radius * [1]*441
        ax.scatter(x, y, z, c='red', alpha=0.1)

    # Plot path
    if full_path:
        path_x = [p[0] for p in full_path]
        path_y = [p[1] for p in full_path]
        path_z = [p[2] for p in full_path]
        ax.plot(path_x, path_y, path_z, 'b-', linewidth=2, alpha=0.8)

        # Plot waypoints
        for i, wp in enumerate(waypoints):
            ax.scatter(wp[0], wp[1], wp[2], c='green', s=100, marker='^')
            ax.text(wp[0], wp[1], wp[2], f'WP{i}', fontsize=10)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Formation Pathfinding with Obstacles')
    ax.set_xlim([0, 100])
    ax.set_ylim([0, 100])
    ax.set_zlim([0, 50])

    plt.show()


if __name__ == "__main__":
    try:
        formation_flight_demo()
        pathfinding_formation_demo()
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except ImportError as e:
        print(f"Import error: {e}")
        print("Make sure you're running this from the examples directory")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
