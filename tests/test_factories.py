"""
Testes unitários para o sistema de fábricas de componentes.
"""

import numpy as np
import pytest
from backend.coordinator import SwarmConfig
from backend.factories import (
    DefaultComponentFactory,
    IdealCommunicationManager,
    ModularSwarmCoordinator,
    create_custom_factory,
)
from backend.interfaces import IComponentFactory
from backend.pathfinding import Pathfinding3D


class TestDefaultComponentFactory:
    """Testes para DefaultComponentFactory"""

    def test_initialization(self):
        """Testa inicialização da fábrica"""
        factory = DefaultComponentFactory()

        assert isinstance(factory, IComponentFactory)
        assert 'raft' in factory._consensus_implementations
        assert 'astar_3d' in factory._pathfinding_implementations

    def test_create_consensus_algorithm(self, sample_config):
        """Testa criação de algoritmo de consenso"""
        factory = DefaultComponentFactory()

        consensus = factory.create_consensus_algorithm(0, {
            'algorithm': 'raft',
            'swarm_config': sample_config
        })

        assert consensus is not None
        assert consensus.drone_id == 0

    def test_create_consensus_algorithm_invalid(self, sample_config):
        """Testa criação com algoritmo inválido"""
        factory = DefaultComponentFactory()

        with pytest.raises(ValueError):
            factory.create_consensus_algorithm(0, {
                'algorithm': 'invalid',
                'swarm_config': sample_config
            })

    def test_create_pathfinding_algorithm(self):
        """Testa criação de algoritmo de pathfinding"""
        factory = DefaultComponentFactory()

        pathfinder = factory.create_pathfinding_algorithm({
            'algorithm': 'astar_3d',
            'bounds': (100, 100, 50),
            'resolution': 1.0
        })

        assert isinstance(pathfinder, Pathfinding3D)
        assert pathfinder.bounds == (100, 100, 50)

    def test_create_pathfinding_algorithm_defaults(self):
        """Testa criação com valores padrão"""
        factory = DefaultComponentFactory()

        pathfinder = factory.create_pathfinding_algorithm({})

        assert isinstance(pathfinder, Pathfinding3D)

    def test_create_collision_avoidance(self, sample_config):
        """Testa criação de sistema de avoidance"""
        factory = DefaultComponentFactory()

        avoidance = factory.create_collision_avoidance({
            'algorithm': 'potential_fields',
            'swarm_config': sample_config
        })

        assert avoidance is not None
        assert avoidance.config == sample_config

    def test_create_physics_engine(self):
        """Testa criação de motor de física"""
        factory = DefaultComponentFactory()

        physics = factory.create_physics_engine({
            'engine': 'rigid_body',
            'num_drones': 5
        })

        assert physics is not None
        assert len(physics.drones) == 5

    def test_create_formation_planner(self):
        """Testa criação de planejador de formação"""
        factory = DefaultComponentFactory()
        pathfinder = Pathfinding3D((100, 100, 50))

        planner = factory.create_formation_planner({
            'pathfinder': pathfinder
        })

        assert planner is not None
        assert planner.pathfinder == pathfinder

    def test_create_formation_planner_no_pathfinder(self):
        """Testa criação sem pathfinder (deve falhar)"""
        factory = DefaultComponentFactory()

        with pytest.raises(ValueError):
            factory.create_formation_planner({})

    def test_create_environment(self):
        """Testa criação de ambiente"""
        factory = DefaultComponentFactory()

        env = factory.create_environment({
            'bounds': (100, 100, 50)
        })

        assert env is not None
        assert env.bounds == (100, 100, 50)


class TestIdealCommunicationManager:
    """Testes para IdealCommunicationManager"""

    def test_initialization(self):
        """Testa inicialização do gerenciador"""
        manager = IdealCommunicationManager({'num_drones': 5})

        assert manager.config['num_drones'] == 5
        assert isinstance(manager.message_queues, dict)

    def test_send_message(self):
        """Testa envio de mensagem"""
        manager = IdealCommunicationManager({'num_drones': 5})

        manager.send_message(0, 1, {'type': 'test'})

        assert 1 in manager.message_queues
        assert len(manager.message_queues[1]) == 1
        assert manager.message_queues[1][0]['from'] == 0

    def test_broadcast_message(self):
        """Testa broadcast de mensagem"""
        manager = IdealCommunicationManager({'num_drones': 3})

        manager.broadcast_message(0, {'type': 'broadcast'})

        # Deve ter mensagens para todos exceto o remetente
        assert len(manager.message_queues) == 2
        assert 1 in manager.message_queues
        assert 2 in manager.message_queues

    def test_receive_messages(self):
        """Testa recebimento de mensagens"""
        manager = IdealCommunicationManager({'num_drones': 5})

        manager.send_message(0, 1, {'type': 'test1'})
        manager.send_message(0, 1, {'type': 'test2'})

        messages = manager.receive_messages(1)

        assert len(messages) == 2
        assert messages[0]['content']['type'] == 'test1'

        # Fila deve estar vazia após recebimento
        assert len(manager.message_queues[1]) == 0

    def test_receive_messages_empty(self):
        """Testa recebimento quando não há mensagens"""
        manager = IdealCommunicationManager({'num_drones': 5})

        messages = manager.receive_messages(1)

        assert len(messages) == 0

    def test_get_network_state(self):
        """Testa obtenção do estado da rede"""
        manager = IdealCommunicationManager({'num_drones': 5})

        state = manager.get_network_state()

        assert state['latency'] == 0.0
        assert state['packet_loss'] == 0.0
        assert state['bandwidth'] == float('inf')

    def test_simulate_network_conditions(self):
        """Testa simulação de condições de rede (ideal não aplica)"""
        manager = IdealCommunicationManager({'num_drones': 5})

        # Não deve causar erro
        manager.simulate_network_conditions(latency=0.1, packet_loss=0.05)


class TestModularSwarmCoordinator:
    """Testes para ModularSwarmCoordinator"""

    def test_initialization(self, sample_config):
        """Testa inicialização do coordenador modular"""
        coordinator = ModularSwarmCoordinator(sample_config)

        assert coordinator.config == sample_config
        assert coordinator.pathfinder is not None
        assert coordinator.collision_avoidance is not None
        assert len(coordinator.consensus_algorithms) == sample_config.num_drones

    def test_initialization_with_custom_factory(self, sample_config):
        """Testa inicialização com fábrica customizada"""
        factory = DefaultComponentFactory()
        coordinator = ModularSwarmCoordinator(sample_config, factory)

        assert coordinator.factory == factory

    def test_step(self, sample_config):
        """Testa execução de step"""
        coordinator = ModularSwarmCoordinator(sample_config)

        state = coordinator.step(0.1)

        assert isinstance(state, dict)
        assert 'timestamp' in state
        assert 'formation' in state

    def test_calculate_formation(self, sample_config):
        """Testa cálculo de formação"""
        coordinator = ModularSwarmCoordinator(sample_config)

        coordinator._calculate_formation()

        assert len(coordinator.target_positions) == sample_config.num_drones


class TestCreateCustomFactory:
    """Testes para create_custom_factory"""

    def test_create_custom_factory(self):
        """Testa criação de fábrica customizada"""
        # Define classe customizada mock
        class CustomPathfinder:
            def __init__(self, bounds, resolution):
                self.bounds = bounds
                self.resolution = resolution

        factory = create_custom_factory(
            pathfinding_algorithm=CustomPathfinder
        )

        assert isinstance(factory, DefaultComponentFactory)
        # Verifica que mapeamento customizado foi adicionado
        assert 'custom' in factory._pathfinding_implementations







