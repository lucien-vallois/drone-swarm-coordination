#!/usr/bin/env python3
"""
Exemplo de extensão modular do sistema de coordenação de drones
Demonstra como implementar componentes customizados usando interfaces
"""

import os
import sys
import time
from typing import Any, Dict, List, Optional

import numpy as np

# Adicionar o diretório backend ao path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.coordinator import SwarmConfig
from backend.factories import create_custom_factory
from backend.interfaces import (
    ConsensusState,
    Formation,
    ICollisionAvoidance,
    IConsensusAlgorithm,
    IPathfindingAlgorithm,
)


class CustomConsensusAlgorithm(IConsensusAlgorithm):
    """
    Implementação customizada de consenso baseada em leader election simples.

    Esta implementação demonstra como criar algoritmos de consenso alternativos
    que podem ser facilmente integrados ao sistema.
    """

    def __init__(self, drone_id: int, config: Dict[str, Any]):
        self._drone_id = drone_id
        self._config = config
        self._state = ConsensusState.FOLLOWER
        self._leader_id = None
        self._election_timeout = np.random.uniform(1.0, 3.0)
        self._last_heartbeat = time.time()
        self._votes_received = 0

    @property
    def drone_id(self) -> int:
        return self._drone_id

    @property
    def state(self) -> ConsensusState:
        return self._state

    def step(self, current_time: float) -> Optional[Dict[str, Any]]:
        if self._state == ConsensusState.LEADER:
            # Enviar heartbeat periodicamente
            if current_time - self._last_heartbeat >= self._config.get('heartbeat_interval', 1.0):
                self._last_heartbeat = current_time
                return {
                    'type': 'heartbeat',
                    'leader_id': self._drone_id,
                    'timestamp': current_time
                }
        elif current_time - self._last_heartbeat >= self._election_timeout:
            # Iniciar eleição
            self._state = ConsensusState.CANDIDATE
            self._votes_received = 1  # Voto próprio
            return {
                'type': 'election',
                'candidate_id': self._drone_id,
                'timestamp': current_time
            }
        return None

    def receive_heartbeat(self, leader_id: int, term: int) -> None:
        self._state = ConsensusState.FOLLOWER
        self._leader_id = leader_id
        self._last_heartbeat = time.time()

    def request_vote(self, candidate_id: int, term: int) -> bool:
        # Estratégia simples: sempre votar no primeiro candidato
        if self._state == ConsensusState.FOLLOWER:
            return True
        return False

    def receive_vote(self, voter_id: int) -> None:
        self._votes_received += 1
        majority = (self._config.get('num_drones', 5) // 2) + 1
        if self._votes_received >= majority and self._state == ConsensusState.CANDIDATE:
            self._state = ConsensusState.LEADER
            self._last_heartbeat = time.time()

    def is_leader(self) -> bool:
        return self._state == ConsensusState.LEADER

    def get_state_info(self) -> Dict[str, Any]:
        return {
            'drone_id': self._drone_id,
            'state': self._state.value,
            'leader_id': self._leader_id,
            'votes_received': self._votes_received
        }


class CustomPathfindingAlgorithm(IPathfindingAlgorithm):
    """
    Implementação customizada de pathfinding usando busca em grade.

    Demonstra como criar algoritmos de pathfinding alternativos
    com diferentes estratégias de busca.
    """

    def __init__(self, bounds: tuple, resolution: float = 2.0):
        self.bounds = bounds
        self.resolution = resolution
        self.obstacles: List[tuple] = []

        # Criar grade para busca
        self.grid_width = int(bounds[0] / resolution)
        self.grid_height = int(bounds[1] / resolution)
        self.grid_depth = int(bounds[2] / resolution)

    def find_path(self, start: np.ndarray, goal: np.ndarray,
                  **kwargs) -> Optional[List[np.ndarray]]:
        """
        Implementação simplificada de busca em grade.
        Usa BFS para encontrar caminho válido.
        """
        if not self.is_valid_position(start) or not self.is_valid_position(goal):
            return None

        # Converter para coordenadas de grade
        start_grid = self._world_to_grid(start)
        goal_grid = self._world_to_grid(goal)

        # BFS
        from collections import deque
        queue = deque([start_grid])
        came_from = {start_grid: None}
        visited = set([start_grid])

        while queue:
            current = queue.popleft()

            if current == goal_grid:
                # Reconstruir caminho
                path = []
                while current:
                    path.append(self._grid_to_world(current))
                    current = came_from[current]
                return path[::-1]  # Inverter

            # Explorar vizinhos
            for neighbor in self._get_grid_neighbors(current):
                if neighbor not in visited and self._is_grid_valid(neighbor):
                    visited.add(neighbor)
                    queue.append(neighbor)
                    came_from[neighbor] = current

        return None

    def add_obstacle(self, position: np.ndarray, radius: float) -> None:
        self.obstacles.append((position, radius))

    def clear_obstacles(self) -> None:
        self.obstacles.clear()

    def is_valid_position(self, position: np.ndarray) -> bool:
        # Verificar limites
        if not (0 <= position[0] <= self.bounds[0] and
                0 <= position[1] <= self.bounds[1] and
                0 <= position[2] <= self.bounds[2]):
            return False

        # Verificar obstáculos
        for obs_pos, obs_radius in self.obstacles:
            if np.linalg.norm(position - obs_pos) <= obs_radius:
                return False

        return True

    def get_bounds(self) -> tuple:
        return self.bounds

    def optimize_path(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """Otimização simples: remover pontos colineares."""
        if len(path) <= 2:
            return path

        optimized = [path[0]]
        for i in range(1, len(path) - 1):
            # Verificar se podemos pular este ponto
            if not self._can_skip_point(path[i-1], path[i], path[i+1]):
                optimized.append(path[i])
        optimized.append(path[-1])

        return optimized

    def _world_to_grid(self, position: np.ndarray) -> tuple:
        """Converte coordenadas do mundo para coordenadas de grade."""
        x = int(position[0] / self.resolution)
        y = int(position[1] / self.resolution)
        z = int(position[2] / self.resolution)
        return (x, y, z)

    def _grid_to_world(self, grid_pos: tuple) -> np.ndarray:
        """Converte coordenadas de grade para coordenadas do mundo."""
        x = grid_pos[0] * self.resolution + self.resolution / 2
        y = grid_pos[1] * self.resolution + self.resolution / 2
        z = grid_pos[2] * self.resolution + self.resolution / 2
        return np.array([x, y, z])

    def _get_grid_neighbors(self, grid_pos: tuple) -> List[tuple]:
        """Retorna vizinhos em 6 direções (sem diagonais)."""
        x, y, z = grid_pos
        neighbors = []
        for dx, dy, dz in [(-1,0,0), (1,0,0), (0,-1,0), (0,1,0), (0,0,-1), (0,0,1)]:
            neighbors.append((x+dx, y+dy, z+dz))
        return neighbors

    def _is_grid_valid(self, grid_pos: tuple) -> bool:
        """Verifica se posição de grade é válida."""
        x, y, z = grid_pos
        if not (0 <= x < self.grid_width and
                0 <= y < self.grid_height and
                0 <= z < self.grid_depth):
            return False

        # Verificar obstáculos na posição do mundo
        world_pos = self._grid_to_world(grid_pos)
        return self.is_valid_position(world_pos)

    def _can_skip_point(self, prev: np.ndarray, current: np.ndarray,
                       next_pos: np.ndarray) -> bool:
        """Verifica se um ponto intermediário pode ser removido."""
        # Verificação simples: se segmento direto é válido
        direction = next_pos - prev
        distance = np.linalg.norm(direction)
        if distance < 1.0:
            return True

        # Amostrar pontos ao longo do segmento
        steps = int(distance / self.resolution)
        for i in range(steps):
            t = (i + 1) / (steps + 1)
            point = prev + t * direction
            if not self.is_valid_position(point):
                return False

        return True


class CustomCollisionAvoidance(ICollisionAvoidance):
    """
    Implementação customizada de avoidance usando velocity obstacles.

    Demonstra como implementar estratégias alternativas de avoidance
    baseadas em velocity obstacles (VO) ao invés de campos de potencial.
    """

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.time_horizon = config.get('time_horizon', 2.0)  # segundos
        self.velocity_obstacles: Dict[int, List] = {}

    def calculate_forces(self, drone_pos: np.ndarray, target_pos: np.ndarray,
                        neighbors: List[np.ndarray],
                        obstacles: List[np.ndarray]) -> np.ndarray:
        """
        Calcula força baseada em velocity obstacles.

        Esta é uma implementação simplificada que demonstra o conceito.
        """
        desired_velocity = np.zeros(3)

        if np.linalg.norm(target_pos - drone_pos) > 0.1:
            direction = target_pos - drone_pos
            desired_velocity = direction / np.linalg.norm(direction) * self.config.get('max_velocity', 2.0)

        # VO-based avoidance (simplificado)
        avoidance_force = np.zeros(3)

        for neighbor_pos in neighbors:
            # Calcular velocity obstacle
            relative_pos = neighbor_pos - drone_pos
            distance = np.linalg.norm(relative_pos)

            if distance < self.config.get('collision_threshold', 2.0) and distance > 0:
                # Força repulsiva baseada em VO
                avoidance_dir = -relative_pos / distance
                avoidance_force += avoidance_dir * (10.0 / distance)

        return desired_velocity + avoidance_force

    def avoid_collisions(self, swarm_positions: Dict[int, np.ndarray],
                        obstacles: List[np.ndarray]) -> Dict[int, np.ndarray]:
        """Calcula avoidance para todos os drones usando VO."""
        avoidance_forces = {}

        for drone_id, position in swarm_positions.items():
            neighbors = []
            for other_id, other_pos in swarm_positions.items():
                if other_id != drone_id:
                    neighbors.append(other_pos)

            # Usar posição atual como alvo temporário para foco em avoidance
            force = self.calculate_forces(position, position, neighbors, obstacles)
            avoidance_forces[drone_id] = force

        return avoidance_forces

    def detect_collisions(self, positions: Dict[int, np.ndarray],
                         threshold: float = 2.0) -> List[tuple]:
        """Detecta colisões baseado em proximidade."""
        collisions = []
        drone_ids = list(positions.keys())

        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                pos1 = positions[drone_ids[i]]
                pos2 = positions[drone_ids[j]]
                distance = np.linalg.norm(pos1 - pos2)
                if distance < threshold:
                    collisions.append((drone_ids[i], drone_ids[j]))

        return collisions

    def get_parameters(self) -> Dict[str, Any]:
        return {
            'time_horizon': self.time_horizon,
            'collision_threshold': self.config.get('collision_threshold', 2.0)
        }

    def set_parameters(self, **params) -> None:
        for key, value in params.items():
            if key == 'time_horizon':
                self.time_horizon = value
            elif key in self.config:
                self.config[key] = value


def demonstrate_modular_extension():
    """
    Demonstra como usar componentes customizados no sistema modular.
    """
    print("=== Demonstração de Extensão Modular ===\n")

    # Criar fábrica customizada com componentes customizados
    custom_factory = create_custom_factory(
        consensus_algorithm=CustomConsensusAlgorithm,
        pathfinding_algorithm=CustomPathfindingAlgorithm,
        collision_avoidance=CustomCollisionAvoidance
    )

    # Configuração do enxame
    config = SwarmConfig(
        num_drones=3,
        formation=Formation.CIRCLE
    )

    print("1. Criando componentes customizados...")
    print("   - ConsensusAlgorithm: CustomConsensusAlgorithm")
    print("   - PathfindingAlgorithm: CustomPathfindingAlgorithm")
    print("   - CollisionAvoidance: CustomCollisionAvoidance")

    # Criar instâncias dos componentes
    consensus = custom_factory.create_consensus_algorithm(0, {'swarm_config': config})
    pathfinder = custom_factory.create_pathfinding_algorithm({'bounds': (50, 50, 20)})
    collision_avoidance = custom_factory.create_collision_avoidance({'swarm_config': config})

    print("\n2. Testando componentes...")

    # Testar consensus
    print(f"   Consensus inicial - State: {consensus.state.value}")
    message = consensus.step(time.time())
    if message:
        print(f"   Consensus message: {message['type']}")

    # Testar pathfinding
    start = np.array([0, 0, 0])
    goal = np.array([40, 40, 10])
    path = pathfinder.find_path(start, goal)
    print(f"   Pathfinding: Found path with {len(path) if path else 0} waypoints")

    # Testar collision avoidance
    positions = {0: np.array([0, 0, 0]), 1: np.array([5, 0, 0])}
    forces = collision_avoidance.avoid_collisions(positions, [])
    print(f"   Collision avoidance: Calculated forces for {len(forces)} drones")

    print("\n3. Benefícios da abordagem modular:")
    print("   + Componentes facilmente intercambiáveis")
    print("   + Testabilidade isolada de cada componente")
    print("   + Extensibilidade sem modificar código existente")
    print("   + Reutilização de componentes em diferentes contextos")

    print("\n=== Demonstração Concluída ===")


if __name__ == "__main__":
    demonstrate_modular_extension()
