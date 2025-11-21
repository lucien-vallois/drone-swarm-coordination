"""
Testes para casos extremos e edge cases do SwarmCoordinator.
"""

import numpy as np
import pytest
from backend.coordinator import (
    DroneState,
    Formation,
    SwarmConfig,
    SwarmCoordinator,
)


class TestSwarmCoordinatorEdgeCases:
    """Testes para casos extremos do SwarmCoordinator"""

    def test_single_drone_swarm(self):
        """Testa enxame com apenas um drone"""
        config = SwarmConfig(num_drones=1)
        coordinator = SwarmCoordinator(config)

        assert len(coordinator.drones) == 1
        assert len(coordinator.consensus_algorithms) == 1

        # Deve funcionar normalmente
        state = coordinator.step(0.1)
        assert state['leader_id'] is not None  # Único drone é líder

    def test_large_swarm(self):
        """Testa enxame grande"""
        config = SwarmConfig(num_drones=20)
        coordinator = SwarmCoordinator(config)

        assert len(coordinator.drones) == 20

        # Deve funcionar sem erros
        state = coordinator.step(0.1)
        assert len(state['drones']) == 20

    def test_zero_dt_step(self):
        """Testa step com dt zero"""
        config = SwarmConfig(num_drones=3)
        coordinator = SwarmCoordinator(config)

        initial_positions = {
            id: drone.position.copy()
            for id, drone in coordinator.drones.items()
        }

        state = coordinator.step(0.0)

        # Posições não devem mudar com dt=0
        for id, drone in coordinator.drones.items():
            assert np.allclose(drone.position, initial_positions[id])

    def test_very_large_dt_step(self):
        """Testa step com dt muito grande"""
        config = SwarmConfig(num_drones=3)
        coordinator = SwarmCoordinator(config)

        # Não deve causar crash
        state = coordinator.step(10.0)
        assert state is not None

    def test_formation_v_formation_odd_drones(self):
        """Testa formação V com número ímpar de drones"""
        config = SwarmConfig(num_drones=5, formation=Formation.V_FORMATION)
        coordinator = SwarmCoordinator(config)

        coordinator._calculate_formation()

        assert len(coordinator.target_positions) == 5
        # Deve ter posições válidas
        for pos in coordinator.target_positions.values():
            assert len(pos) == 3
            assert all(np.isfinite(pos))

    def test_formation_v_formation_even_drones(self):
        """Testa formação V com número par de drones"""
        config = SwarmConfig(num_drones=4, formation=Formation.V_FORMATION)
        coordinator = SwarmCoordinator(config)

        coordinator._calculate_formation()

        assert len(coordinator.target_positions) == 4

    def test_formation_grid_square(self):
        """Testa formação grid com número quadrado perfeito"""
        config = SwarmConfig(num_drones=9, formation=Formation.GRID)  # 3x3
        coordinator = SwarmCoordinator(config)

        coordinator._calculate_formation()

        assert len(coordinator.target_positions) == 9

    def test_formation_grid_non_square(self):
        """Testa formação grid com número não quadrado"""
        config = SwarmConfig(num_drones=7, formation=Formation.GRID)
        coordinator = SwarmCoordinator(config)

        coordinator._calculate_formation()

        assert len(coordinator.target_positions) == 7

    def test_multiple_obstacles_same_position(self):
        """Testa adição de múltiplos obstáculos na mesma posição"""
        config = SwarmConfig(num_drones=3)
        coordinator = SwarmCoordinator(config)

        position = np.array([50.0, 50.0, 10.0])
        coordinator.add_obstacle(position, 2.0)
        coordinator.add_obstacle(position, 3.0)

        # Ambos devem ser adicionados
        assert len(coordinator.pathfinder.obstacles) >= 2

    def test_obstacle_outside_bounds(self):
        """Testa adição de obstáculo fora dos limites"""
        config = SwarmConfig(num_drones=3)
        coordinator = SwarmCoordinator(config)

        # Obstáculo fora dos limites (bounds são 100x100x50)
        position = np.array([150.0, 150.0, 60.0])
        coordinator.add_obstacle(position, 5.0)

        # Deve ser adicionado, mas pathfinding pode não funcionar bem
        assert len(coordinator.pathfinder.obstacles) > 0

    def test_all_drones_inactive(self):
        """Testa comportamento quando todos os drones estão inativos"""
        config = SwarmConfig(num_drones=3)
        coordinator = SwarmCoordinator(config)

        # Desativa todos os drones
        for drone in coordinator.drones.values():
            drone.is_active = False

        # Step deve funcionar sem crash
        state = coordinator.step(0.1)
        assert state is not None

    def test_consensus_no_leader(self):
        """Testa comportamento quando não há líder"""
        config = SwarmConfig(num_drones=3)
        coordinator = SwarmCoordinator(config)

        # Garante que não há líder
        from backend.coordinator import ConsensusState
        for consensus in coordinator.consensus_algorithms.values():
            consensus.state = ConsensusState.FOLLOWER

        state = coordinator.step(0.1)

        # Pode não haver líder inicialmente
        # Mas sistema deve continuar funcionando

    def test_pathfinding_no_path(self):
        """Testa quando pathfinding não encontra caminho"""
        config = SwarmConfig(num_drones=3)
        coordinator = SwarmCoordinator(config)

        # Cerca completamente uma posição alvo
        target_pos = np.array([50.0, 50.0, 10.0])
        for i in range(8):
            angle = 2 * np.pi * i / 8
            obs_pos = target_pos + np.array([
                3.0 * np.cos(angle),
                3.0 * np.sin(angle),
                0.0
            ])
            coordinator.add_obstacle(obs_pos, 2.0)

        # Recalcula formação
        coordinator._calculate_formation()

        # Alguns caminhos podem ser None, mas não deve crashar
        assert coordinator.target_positions is not None

    def test_battery_depletion_sequence(self):
        """Testa sequência de depleção de bateria"""
        config = SwarmConfig(num_drones=1)
        coordinator = SwarmCoordinator(config)

        drone = coordinator.drones[0]
        drone.battery_level = 0.2
        drone.velocity = np.array([5.0, 0.0, 0.0])  # Alta velocidade

        # Executa múltiplos steps
        for _ in range(10):
            coordinator.step(0.1)

        # Bateria deve ter diminuído
        assert drone.battery_level < 0.2

    def test_reset_preserves_config(self):
        """Testa que reset preserva configuração"""
        config = SwarmConfig(num_drones=5, formation=Formation.LINE)
        coordinator = SwarmCoordinator(config)

        # Modifica estado
        coordinator.add_obstacle(np.array([10, 10, 10]), 2.0)
        coordinator.set_formation(Formation.CIRCLE)

        # Reset
        coordinator.reset()

        # Configuração deve ser preservada
        assert coordinator.config.num_drones == 5
        # Formação volta para a inicial
        assert coordinator.formation == Formation.LINE

    def test_rapid_formation_changes(self):
        """Testa mudanças rápidas de formação"""
        config = SwarmConfig(num_drones=3)
        coordinator = SwarmCoordinator(config)

        formations = [Formation.CIRCLE, Formation.LINE, Formation.GRID, Formation.V_FORMATION]

        for formation in formations:
            coordinator.set_formation(formation)
            assert coordinator.formation == formation

    def test_consensus_message_processing(self):
        """Testa processamento de mensagens de consenso"""
        config = SwarmConfig(num_drones=3)
        coordinator = SwarmCoordinator(config)

        # Simula mensagem de heartbeat
        message = {
            'type': 'heartbeat',
            'from': 0,
            'leader_id': 0,
            'term': 1
        }

        coordinator._process_consensus_message(message)

        # Outros drones devem receber o heartbeat
        for i in range(1, 3):
            consensus = coordinator.consensus_algorithms[i]
            assert consensus.current_term >= 1

    def test_consensus_vote_request(self):
        """Testa processamento de vote request"""
        config = SwarmConfig(num_drones=3)
        coordinator = SwarmCoordinator(config)

        # Simula vote request
        message = {
            'type': 'vote_request',
            'from': 0,
            'candidate_id': 0,
            'term': 1
        }

        coordinator._process_consensus_message(message)

        # Candidato deve receber votos se concedidos

    def test_drone_state_to_dict(self):
        """Testa conversão de DroneState para dicionário"""
        drone = DroneState(
            id=0,
            position=np.array([1.0, 2.0, 3.0]),
            velocity=np.array([0.1, 0.2, 0.3]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            battery_level=0.8,
            is_active=True
        )

        drone_dict = drone.to_dict()

        assert isinstance(drone_dict, dict)
        assert drone_dict['id'] == 0
        assert drone_dict['position'] == [1.0, 2.0, 3.0]
        assert drone_dict['battery_level'] == 0.8
        assert drone_dict['is_active'] is True

    def test_swarm_config_defaults(self):
        """Testa valores padrão de SwarmConfig"""
        config = SwarmConfig()

        assert config.num_drones == 5
        assert config.separation_distance == 5.0
        assert config.max_velocity == 2.0
        assert config.formation == Formation.CIRCLE

