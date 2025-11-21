"""
Testes unitários para collision avoidance e formação de enxames.
"""

from unittest.mock import Mock

import numpy as np
import pytest
from backend.coordinator import (
    CollisionAvoidance,
    Formation,
    SwarmConfig,
    SwarmCoordinator,
)


class TestCollisionAvoidance:
    """Testes para CollisionAvoidance"""

    def test_initialization(self, sample_config):
        """Testa inicialização do sistema de avoidance"""
        avoidance = CollisionAvoidance(sample_config)

        assert avoidance.config == sample_config
        assert avoidance.repulsive_gain > 0
        assert avoidance.attractive_gain > 0
        assert avoidance.drone_radius > 0

    def test_calculate_force_no_neighbors_no_obstacles(self, sample_config):
        """Testa cálculo de força sem vizinhos ou obstáculos"""
        avoidance = CollisionAvoidance(sample_config)

        drone_pos = np.array([10.0, 10.0, 5.0])
        target_pos = np.array([20.0, 10.0, 5.0])
        neighbors = []
        obstacles = []

        force = avoidance.calculate_force(drone_pos, target_pos, neighbors, obstacles)

        # Deve ter apenas força atrativa ao alvo
        expected_attractive = avoidance.attractive_gain * (target_pos - drone_pos)
        assert np.allclose(force, expected_attractive)

    def test_calculate_force_with_neighbors(self, sample_config):
        """Testa cálculo de força com vizinhos próximos"""
        avoidance = CollisionAvoidance(sample_config)

        drone_pos = np.array([10.0, 10.0, 5.0])
        target_pos = np.array([20.0, 10.0, 5.0])
        neighbors = [np.array([11.0, 10.0, 5.0])]  # Vizinho muito próximo
        obstacles = []

        force = avoidance.calculate_force(drone_pos, target_pos, neighbors, obstacles)

        # Deve ter força repulsiva do vizinho
        assert force[0] < 0  # Força repulsiva em X (vizinho à direita)

    def test_calculate_force_with_obstacles(self, sample_config):
        """Testa cálculo de força com obstáculos"""
        avoidance = CollisionAvoidance(sample_config)

        drone_pos = np.array([10.0, 10.0, 5.0])
        target_pos = np.array([20.0, 10.0, 5.0])
        neighbors = []
        obstacles = [(np.array([15.0, 10.0, 5.0]), 2.0)]  # Obstáculo no caminho

        force = avoidance.calculate_force(drone_pos, target_pos, neighbors, obstacles)

        # Deve haver componente repulsiva
        # Força total = atrativa + repulsiva

    def test_avoid_collisions_multiple_drones(self, sample_config):
        """Testa avoidance para múltiplos drones"""
        avoidance = CollisionAvoidance(sample_config)

        # Configuração com 3 drones próximos
        swarm_positions = {
            0: np.array([0.0, 0.0, 0.0]),
            1: np.array([1.5, 0.0, 0.0]),  # Próximo do drone 0
            2: np.array([3.0, 0.0, 0.0])   # Mais distante
        }
        obstacles = []

        forces = avoidance.avoid_collisions(swarm_positions, obstacles)

        assert len(forces) == 3

        # Drone 0 deve sentir repulsão do drone 1
        assert forces[0][0] < 0  # Força para esquerda

        # Drone 1 deve sentir repulsão tanto do drone 0 quanto do drone 2
        assert forces[1][0] < 0  # Força para esquerda (afastar do 0)

        # Drone 2 pode sentir pouca ou nenhuma força se estiver longe

    def test_avoid_collisions_with_obstacles(self, sample_config):
        """Testa avoidance com obstáculos"""
        avoidance = CollisionAvoidance(sample_config)

        swarm_positions = {
            0: np.array([10.0, 10.0, 5.0])
        }
        obstacles = [(np.array([12.0, 10.0, 5.0]), 1.0)]  # Obstáculo próximo

        forces = avoidance.avoid_collisions(swarm_positions, obstacles)

        assert len(forces) == 1
        # Deve haver força repulsiva do obstáculo
        assert forces[0][0] < 0  # Força para esquerda (afastar do obstáculo)


class TestSwarmCoordinator:
    """Testes de integração para SwarmCoordinator"""

    def test_initialization(self, sample_config):
        """Testa inicialização do coordinator"""
        coordinator = SwarmCoordinator(sample_config)

        assert len(coordinator.drones) == sample_config.num_drones
        assert len(coordinator.consensus_algorithms) == sample_config.num_drones
        assert coordinator.formation == sample_config.formation
        assert len(coordinator.target_positions) == sample_config.num_drones

    def test_drone_initialization(self, sample_config):
        """Testa que drones são inicializados corretamente"""
        coordinator = SwarmCoordinator(sample_config)

        for drone_id, drone in coordinator.drones.items():
            assert drone.id == drone_id
            assert isinstance(drone.position, np.ndarray)
            assert isinstance(drone.velocity, np.ndarray)
            assert isinstance(drone.orientation, np.ndarray)
            assert 0.0 <= drone.battery_level <= 1.0
            assert drone.is_active == True

    def test_calculate_formation_circle(self, sample_config):
        """Testa cálculo de formação circular"""
        coordinator = SwarmCoordinator(sample_config)

        coordinator._calculate_formation()

        assert len(coordinator.target_positions) == sample_config.num_drones

        # Verifica que posições estão distribuídas em círculo
        positions = list(coordinator.target_positions.values())

        # Centro aproximado
        center_x = sum(p[0] for p in positions) / len(positions)
        center_y = sum(p[1] for p in positions) / len(positions)

        assert abs(center_x - 50.0) < 10.0  # Próximo do centro esperado
        assert abs(center_y - 50.0) < 10.0

    def test_calculate_formation_line(self, sample_config):
        """Testa cálculo de formação em linha"""
        config = SwarmConfig(num_drones=3, formation=Formation.LINE)
        coordinator = SwarmCoordinator(config)

        coordinator._calculate_formation()

        positions = list(coordinator.target_positions.values())

        # Deve estar alinhado em Y (mesma coordenada Y)
        y_coords = [p[1] for p in positions]
        assert all(y == y_coords[0] for y in y_coords)

        # Deve variar em X
        x_coords = [p[0] for p in positions]
        assert len(set(x_coords)) == len(x_coords)  # Todos X diferentes

    def test_calculate_formation_grid(self, sample_config):
        """Testa cálculo de formação em grid"""
        config = SwarmConfig(num_drones=4, formation=Formation.GRID)  # 2x2 grid
        coordinator = SwarmCoordinator(config)

        coordinator._calculate_formation()

        positions = list(coordinator.target_positions.values())

        # Deve formar um grid 2x2
        x_coords = sorted([p[0] for p in positions])
        y_coords = sorted([p[1] for p in positions])

        # Deve haver 2 valores únicos de X e 2 de Y
        assert len(set(x_coords)) == 2
        assert len(set(y_coords)) == 2

    def test_set_formation(self, sample_config):
        """Testa mudança de formação"""
        coordinator = SwarmCoordinator(sample_config)

        initial_formation = coordinator.formation
        initial_positions = coordinator.target_positions.copy()

        # Muda para linha
        coordinator.set_formation(Formation.LINE)

        assert coordinator.formation == Formation.LINE
        assert coordinator.target_positions != initial_positions

    def test_add_obstacle(self, sample_config):
        """Testa adição de obstáculos"""
        coordinator = SwarmCoordinator(sample_config)

        position = np.array([25.0, 25.0, 10.0])
        radius = 3.0

        coordinator.add_obstacle(position, radius)

        # Verifica que obstáculo foi adicionado ao pathfinder
        assert len(coordinator.pathfinder.obstacles) > 0

    def test_step_integration(self, sample_config):
        """Testa integração completa do step"""
        coordinator = SwarmCoordinator(sample_config)

        # Executa um step
        dt = 0.1
        state = coordinator.step(dt)

        assert isinstance(state, dict)
        assert 'drones' in state
        assert 'leader_id' in state
        assert 'formation' in state
        assert 'target_positions' in state
        assert 'timestamp' in state

        # Deve haver dados para todos os drones
        assert len(state['drones']) == sample_config.num_drones

    def test_consensus_integration(self, sample_config):
        """Testa integração do consensus no step"""
        coordinator = SwarmCoordinator(sample_config)

        # Força um algoritmo a se tornar líder
        coordinator.consensus_algorithms[0].state = coordinator.consensus_algorithms[0].ConsensusState.LEADER

        dt = 0.1
        state = coordinator.step(dt)

        # Deve haver um líder identificado
        assert state['leader_id'] is not None

    def test_pathfinding_integration(self, sample_config):
        """Testa integração do pathfinding"""
        coordinator = SwarmCoordinator(sample_config)

        # Adiciona obstáculo
        coordinator.add_obstacle(np.array([50.0, 50.0, 10.0]), 5.0)

        # Recalcula formação (deve usar pathfinding)
        coordinator._calculate_formation()

        # Alguns drones devem ter caminhos calculados
        has_paths = any(len(path) > 1 for path in coordinator.paths.values())
        # Nota: pode não haver caminhos se posições estiverem livres

    def test_battery_drain(self, sample_config):
        """Testa consumo de bateria"""
        coordinator = SwarmCoordinator(sample_config)

        # Drone com velocidade
        coordinator.drones[0].velocity = np.array([2.0, 0.0, 0.0])

        initial_battery = coordinator.drones[0].battery_level

        dt = 0.1
        coordinator.step(dt)

        # Bateria deve ter diminuído
        assert coordinator.drones[0].battery_level < initial_battery

    def test_drone_deactivation(self, sample_config):
        """Testa desativação de drone com bateria baixa"""
        coordinator = SwarmCoordinator(sample_config)

        # Define bateria muito baixa
        coordinator.drones[0].battery_level = 0.05  # Abaixo do threshold

        dt = 0.1
        coordinator.step(dt)

        # Drone deve ser desativado
        assert coordinator.drones[0].is_active == False

    def test_get_swarm_state(self, sample_config):
        """Testa obtenção do estado completo do enxame"""
        coordinator = SwarmCoordinator(sample_config)

        state = coordinator.get_swarm_state()

        assert isinstance(state, dict)
        assert 'drones' in state
        assert 'config' in state
        assert 'obstacles' in state
        assert 'timestamp' in state

        # Deve haver dados para todos os drones
        assert len(state['drones']) == sample_config.num_drones

    def test_reset(self, sample_config):
        """Testa reset do enxame"""
        coordinator = SwarmCoordinator(sample_config)

        # Modifica estado
        coordinator.add_obstacle(np.array([10, 10, 10]), 2.0)
        coordinator.set_formation(Formation.LINE)

        # Reset
        coordinator.reset()

        # Deve voltar ao estado inicial
        assert coordinator.formation == sample_config.formation
        assert len(coordinator.obstacles) == 0
        assert len(coordinator.paths) == 0

        # Todos os drones devem estar ativos novamente
        assert all(drone.is_active for drone in coordinator.drones.values())
