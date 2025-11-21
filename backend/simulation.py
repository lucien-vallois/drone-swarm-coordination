#!/usr/bin/env python3
"""
Drone Swarm Coordination Simulation
Main simulation script with WebSocket server for real-time visualization
"""

import asyncio
import json
import logging
import signal
import sys
import threading
import time
from typing import Any, Dict

import websockets
from backend.coordinator import Formation, SwarmConfig, SwarmCoordinator


class SwarmSimulation:
    """Main swarm simulation with WebSocket interface"""

    def __init__(self, config: SwarmConfig):
        self.config = config
        self.coordinator = SwarmCoordinator(config)

        # Simulation state
        self.running = False
        self.simulation_thread = None
        self.websocket_clients = set()
        self.loop = None  # Event loop for async operations in thread

        # Performance monitoring
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0

        self.logger = logging.getLogger("Simulation")

    def start_simulation(self):
        """Start the simulation loop"""
        if self.running:
            return

        self.running = True
        self.simulation_thread = threading.Thread(target=self._simulation_loop)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()
        self.logger.info("Simulation started")

    def stop_simulation(self):
        """Stop the simulation"""
        self.running = False
        if self.simulation_thread:
            self.simulation_thread.join()
        self.logger.info("Simulation stopped")

    def _simulation_loop(self):
        """Main simulation loop"""
        dt = 0.1  # 10Hz simulation
        
        # Create event loop for this thread
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        while self.running:
            start_time = time.time()

            try:
                # Step coordinator (consensus and pathfinding)
                coordinator_state = self.coordinator.step(dt)

                # Prepare state for clients
                state = self._prepare_state(coordinator_state)

                # Broadcast to WebSocket clients
                self.loop.run_until_complete(self._broadcast_state(state))

                # Update performance metrics
                self.frame_count += 1
                elapsed = time.time() - self.start_time
                if elapsed > 0:
                    self.fps = self.frame_count / elapsed

            except Exception as e:
                self.logger.error(f"Simulation error: {e}")

            # Maintain frame rate
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
        
        # Clean up event loop
        if self.loop:
            self.loop.close()
            self.loop = None

    def _prepare_state(self, coordinator_state: Dict) -> Dict[str, Any]:
        """Prepare simulation state for broadcasting"""

        state = {
            'timestamp': time.time(),
            'frame': self.frame_count,
            'fps': self.fps,
            'coordinator': coordinator_state,
            'environment': {
                'bounds': [100, 100, 50],
                'obstacles': coordinator_state.get('obstacles', [])
            }
        }

        return state

    async def _broadcast_state(self, state: Dict):
        """Broadcast state to all connected WebSocket clients"""

        if not self.websocket_clients:
            return

        message = json.dumps(state)

        # Remove disconnected clients
        disconnected = set()
        for client in self.websocket_clients:
            try:
                await client.send(message)
            except Exception:
                disconnected.add(client)

        self.websocket_clients -= disconnected

    def add_websocket_client(self, client):
        """Add a WebSocket client"""
        self.websocket_clients.add(client)
        self.logger.info(f"WebSocket client connected. Total clients: {len(self.websocket_clients)}")

    def remove_websocket_client(self, client):
        """Remove a WebSocket client"""
        self.websocket_clients.discard(client)
        self.logger.info(f"WebSocket client disconnected. Total clients: {len(self.websocket_clients)}")

    def set_formation(self, formation: str):
        """Change swarm formation"""
        try:
            formation_enum = Formation[formation.upper()]
            self.coordinator.set_formation(formation_enum)
        except KeyError:
            self.logger.error(f"Unknown formation: {formation}")

    def add_obstacle(self, x: float, y: float, z: float, radius: float):
        """Add obstacle to environment"""
        position = [x, y, z]
        self.coordinator.add_obstacle(position, radius)
        self.logger.info(f"Added obstacle at {position} with radius {radius}")

    def reset_simulation(self):
        """Reset simulation to initial state"""
        self.coordinator.reset()
        self.frame_count = 0
        self.start_time = time.time()
        self.logger.info("Simulation reset")


class WebSocketServer:
    """WebSocket server for real-time communication"""

    def __init__(self, simulation: SwarmSimulation, host: str = "localhost", port: int = 8765):
        self.simulation = simulation
        self.host = host
        self.port = port
        self.server = None

    async def handle_client(self, websocket, path: str):
        """Handle WebSocket client connection"""

        self.simulation.add_websocket_client(websocket)

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self._handle_message(websocket, data)
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({"error": "Invalid JSON"}))
                except Exception as e:
                    await websocket.send(json.dumps({"error": str(e)}))
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.simulation.remove_websocket_client(websocket)

    async def _handle_message(self, websocket, data: Dict):
        """Handle incoming message from client"""

        message_type = data.get('type', '')

        if message_type == 'command':
            command = data.get('command', '')
            params = data.get('params', {})

            if command == 'start':
                self.simulation.start_simulation()
                await websocket.send(json.dumps({"status": "started"}))

            elif command == 'stop':
                self.simulation.stop_simulation()
                await websocket.send(json.dumps({"status": "stopped"}))

            elif command == 'reset':
                self.simulation.reset_simulation()
                await websocket.send(json.dumps({"status": "reset"}))

            elif command == 'set_formation':
                formation = params.get('formation', 'circle')
                self.simulation.set_formation(formation)
                await websocket.send(json.dumps({"status": "formation_set", "formation": formation}))

            elif command == 'add_obstacle':
                x = params.get('x', 50)
                y = params.get('y', 50)
                z = params.get('z', 10)
                radius = params.get('radius', 5)
                self.simulation.add_obstacle(x, y, z, radius)
                await websocket.send(json.dumps({"status": "obstacle_added"}))

            else:
                await websocket.send(json.dumps({"error": f"Unknown command: {command}"}))

        elif message_type == 'request':
            request = data.get('request', '')

            if request == 'state':
                state = self.simulation._prepare_state(self.simulation.coordinator.get_swarm_state())
                await websocket.send(json.dumps({"type": "state", "data": state}))

            elif request == 'config':
                config = {
                    'num_drones': self.simulation.config.num_drones,
                    'formation': self.simulation.config.formation.value,
                    'max_velocity': self.simulation.config.max_velocity,
                    'separation_distance': self.simulation.config.separation_distance
                }
                await websocket.send(json.dumps({"type": "config", "data": config}))

    async def start(self):
        """Start the WebSocket server"""

        self.server = await websockets.serve(
            self.handle_client,
            self.host,
            self.port
        )

        logging.info(f"WebSocket server started on ws://{self.host}:{self.port}")

        # Keep server running
        await self.server.wait_closed()

    def stop(self):
        """Stop the WebSocket server"""
        if self.server:
            self.server.close()


async def main():
    """Main function"""

    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Drone Swarm Coordination Simulation')
    parser.add_argument('--num-drones', type=int, default=5, help='Number of drones')
    parser.add_argument('--host', type=str, default='localhost', help='WebSocket server host')
    parser.add_argument('--port', type=int, default=8765, help='WebSocket server port')
    parser.add_argument('--formation', type=str, default='circle',
                       choices=['circle', 'line', 'grid', 'v_formation'],
                       help='Initial formation')

    args = parser.parse_args()

    # Create swarm configuration
    config = SwarmConfig(
        num_drones=args.num_drones,
        formation=Formation[args.formation.upper()]
    )

    # Create simulation
    simulation = SwarmSimulation(config)

    # Create WebSocket server
    server = WebSocketServer(simulation, args.host, args.port)

    # Setup signal handlers
    def signal_handler(signum, frame):
        logging.info("Shutting down...")
        simulation.stop_simulation()
        server.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Add some obstacles for demonstration
    simulation.add_obstacle(30, 30, 10, 3)
    simulation.add_obstacle(70, 70, 15, 4)

    # Start simulation
    simulation.start_simulation()

    # Start WebSocket server
    try:
        await server.start()
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)


if __name__ == "__main__":
    asyncio.run(main())
