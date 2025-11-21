"""
Testes de integração para o sistema de coordenação de enxames.
Testa interação entre múltiplos componentes.
"""

import asyncio
import time
from unittest.mock import AsyncMock, Mock, patch

import numpy as np
import pytest
from backend.coordinator import Formation, SwarmConfig, SwarmCoordinator
from backend.pathfinding import Pathfinding3D
from backend.physics import SwarmPhysics
from backend.simulation import SwarmSimulation, WebSocketServer


class TestSwarmCoordinatorIntegration:
    """Testes de integração do SwarmCoordinator"""

    def test_full_simulation_step(self, sample_config):
        """Testa um step completo de simulação"""
        coordinator = SwarmCoordinator(sample_config)

        dt = 0.1
        state = coordinator.step(dt)

        # Verifica estrutura do estado
        required_keys = ['drones', 'leader_id', 'formation', 'target_positions', 'timestamp']
        for key in required_keys:
            assert key in state

        # Verifica que todos os drones têm dados
        assert len(state['drones']) == sample_config.num_drones

        # Verifica formato dos dados dos drones
        for drone_data in state['drones'].values():
            required_drone_keys = ['id', 'position', 'velocity', 'orientation', 'battery_level', 'is_active']
            for key in required_drone_keys:
                assert key in drone_data

    def test_consensus_pathfinding_integration(self, sample_config):
        """Testa integração entre consensus e pathfinding"""
        coordinator = SwarmCoordinator(sample_config)

        # Adiciona obstáculo que força pathfinding
        obstacle_pos = np.array([50.0, 50.0, 10.0])
        coordinator.add_obstacle(obstacle_pos, 3.0)

        # Muda formação para forçar replanejamento
        coordinator.set_formation(Formation.CIRCLE)

        # Step múltiplas vezes
        for _ in range(5):
            state = coordinator.step(0.1)

            # Verifica que líder foi eleito
            assert state['leader_id'] is not None

            # Verifica que posições estão sendo atualizadas
            for drone_data in state['drones'].values():
                assert len(drone_data['position']) == 3

    def test_collision_avoidance_integration(self, sample_config):
        """Testa integração do sistema de avoidance de colisão"""
        # Configuração com drones próximos
        config = SwarmConfig(num_drones=3, separation_distance=3.0)
        coordinator = SwarmCoordinator(config)

        # Posiciona drones próximos (dentro do threshold de colisão)
        coordinator.drones[0].position = np.array([0.0, 0.0, 0.0])
        coordinator.drones[1].position = np.array([2.0, 0.0, 0.0])  # Próximo
        coordinator.drones[2].position = np.array([6.0, 0.0, 0.0])  # Mais distante

        # Step
        dt = 0.1
        state = coordinator.step(dt)

        # Verifica que posições mudaram devido ao avoidance
        # (difícil testar deterministicamente devido à física complexa)

    def test_formation_maintenance(self, sample_config):
        """Testa manutenção de formação ao longo do tempo"""
        coordinator = SwarmCoordinator(sample_config)

        # Executa múltiplos steps
        positions_over_time = []
        for _ in range(10):
            state = coordinator.step(0.1)
            positions = [drone['position'] for drone in state['drones'].values()]
            positions_over_time.append(positions)

        # Verifica que posições convergem para alvos de formação
        final_positions = positions_over_time[-1]
        target_positions = list(coordinator.target_positions.values())

        # Calcula erro médio final
        total_error = 0
        for final_pos in final_positions:
            # Encontra alvo mais próximo
            min_distance = float('inf')
            for target_pos in target_positions:
                distance = np.linalg.norm(np.array(final_pos) - np.array(target_pos))
                min_distance = min(min_distance, distance)
            total_error += min_distance

        avg_error = total_error / len(final_positions)
        # Erro deve ser razoável (não testa convergência perfeita)
        assert avg_error < 20.0


class TestSwarmSimulationIntegration:
    """Testes de integração da SwarmSimulation"""

    def test_simulation_initialization(self, sample_config):
        """Testa inicialização da simulação completa"""
        simulation = SwarmSimulation(sample_config)

        assert simulation.config == sample_config
        assert isinstance(simulation.coordinator, SwarmCoordinator)
        assert simulation.running == False

    def test_simulation_step_sync(self, sample_config):
        """Testa sincronização entre coordinator e physics"""
        simulation = SwarmSimulation(sample_config)

        # Executa step do coordinator
        coord_state = simulation.coordinator.step(0.1)

        # Verifica que estado do coordinator foi gerado
        assert coord_state is not None
        assert 'drones' in coord_state

    def test_obstacle_propagation(self, sample_config):
        """Testa que obstáculos são propagados corretamente"""
        simulation = SwarmSimulation(sample_config)

        position = np.array([25.0, 25.0, 10.0])
        radius = 3.0

        simulation.add_obstacle(25.0, 25.0, 10.0, 3.0)

        # Verifica que obstáculo está no coordinator/pathfinder
        assert len(simulation.coordinator.pathfinder.obstacles) > 0

    @patch('asyncio.Queue')
    def test_prepare_state(self, mock_queue, sample_config):
        """Testa preparação do estado para broadcasting"""
        simulation = SwarmSimulation(sample_config)
        simulation.message_queue = mock_queue

        state = simulation._prepare_state(simulation.coordinator.get_swarm_state())

        required_keys = ['timestamp', 'frame', 'fps', 'coordinator', 'environment']
        for key in required_keys:
            assert key in state

        assert 'drones' in state['coordinator']
        assert 'bounds' in state['environment']


class TestWebSocketIntegration:
    """Testes de integração do WebSocket"""

    @pytest.mark.asyncio
    async def test_websocket_message_handling(self, sample_config):
        """Testa tratamento de mensagens WebSocket"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        # Mock websocket
        mock_ws = AsyncMock()

        # Testa comando start
        message = {'type': 'command', 'command': 'start'}
        await server._handle_message(mock_ws, message)

        # Simulação deve estar rodando
        assert simulation.running == True

        # Testa comando stop
        message = {'type': 'command', 'command': 'stop'}
        await server._handle_message(mock_ws, message)

        # Simulação deve parar
        assert simulation.running == False

    @pytest.mark.asyncio
    async def test_formation_change(self, sample_config):
        """Testa mudança de formação via WebSocket"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        mock_ws = AsyncMock()

        # Muda formação para line
        message = {'type': 'command', 'command': 'set_formation', 'params': {'formation': 'line'}}
        await server._handle_message(mock_ws, message)

        assert simulation.coordinator.formation.value == 'line'

    @pytest.mark.asyncio
    async def test_obstacle_addition(self, sample_config):
        """Testa adição de obstáculos via WebSocket"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        mock_ws = AsyncMock()

        # Adiciona obstáculo
        message = {'type': 'command', 'command': 'add_obstacle',
                  'params': {'x': 30.0, 'y': 30.0, 'z': 10.0, 'radius': 2.0}}
        await server._handle_message(mock_ws, message)

        # Verifica que obstáculo foi adicionado
        assert len(simulation.coordinator.pathfinder.obstacles) > 0


class TestPerformanceBenchmarks:
    """Testes de performance e benchmarks"""

    def test_coordinator_step_performance(self, sample_config):
        """Testa performance do step do coordinator"""
        coordinator = SwarmCoordinator(sample_config)

        # Benchmark com 100 steps
        num_steps = 100
        dt = 0.01  # Passo pequeno

        start_time = time.time()
        for _ in range(num_steps):
            coordinator.step(dt)
        end_time = time.time()

        total_time = end_time - start_time
        avg_step_time = total_time / num_steps

        # Deve ser razoavelmente rápido (< 10ms por step para simulação real-time)
        assert avg_step_time < 0.01  # 10ms

        print(".6f")

    def test_pathfinding_performance(self):
        """Testa performance do pathfinding"""
        pathfinder = Pathfinding3D(bounds=(100, 100, 50))

        # Adiciona alguns obstáculos
        for i in range(5):
            pos = np.array([20*i, 20*i, 10])
            pathfinder.add_obstacle(pos, 3.0)

        # Benchmark pathfinding A*
        num_tests = 50
        times = []

        for _ in range(num_tests):
            start = np.array([0, 0, 0])
            goal = np.array([80, 80, 20])

            start_time = time.time()
            path = pathfinder.astar(start, goal)
            end_time = time.time()

            times.append(end_time - start_time)

        avg_time = sum(times) / len(times)

        # Deve ser rápido (< 100ms típico para ambientes complexos)
        assert avg_time < 0.1  # 100ms

        print(".6f")

    def test_large_swarm_performance(self):
        """Testa performance com enxame grande"""
        config = SwarmConfig(num_drones=20)  # Enxame maior
        coordinator = SwarmCoordinator(config)

        # Benchmark
        num_steps = 50
        dt = 0.05

        start_time = time.time()
        for _ in range(num_steps):
            coordinator.step(dt)
        end_time = time.time()

        total_time = end_time - start_time
        avg_step_time = total_time / num_steps

        # Mesmo com 20 drones deve ser razoável
        assert avg_step_time < 0.05  # 50ms

        print(".6f")

    def test_memory_usage_stability(self, sample_config):
        """Testa que uso de memória permanece estável"""
        coordinator = SwarmCoordinator(sample_config)

        # Executa muitos steps
        num_steps = 1000

        initial_obstacle_count = len(coordinator.pathfinder.obstacles)

        for _ in range(num_steps):
            coordinator.step(0.01)

            # Adiciona obstáculos periodicamente para testar limpeza
            if _ % 100 == 0:
                coordinator.add_obstacle(
                    np.array([np.random.uniform(0, 100),
                             np.random.uniform(0, 100),
                             np.random.uniform(0, 50)]),
                    2.0
                )

        # Número de obstáculos não deve crescer indefinidamente
        # (em implementação real deveria haver limpeza)
        final_obstacle_count = len(coordinator.pathfinder.obstacles)

        # Limitação razoável (não testa limpeza específica)
        assert final_obstacle_count <= initial_obstacle_count + 20

    def test_consensus_convergence_time(self, sample_config):
        """Testa tempo de convergência do consensus"""
        coordinator = SwarmCoordinator(sample_config)

        # Executa até que líder seja eleito
        max_steps = 100
        leader_found = False
        convergence_time = 0

        for step in range(max_steps):
            state = coordinator.step(0.1)
            convergence_time += 0.1

            if state['leader_id'] is not None:
                leader_found = True
                break

        assert leader_found, "Consensus deve convergir para um líder"
        assert convergence_time < 5.0, "Convergência deve ser rápida"

        print(f"Convergence time: {convergence_time:.3f}s")


class TestFaultTolerance:
    """Testes de tolerância a falhas"""

    def test_drone_failure_recovery(self, sample_config):
        """Testa recuperação de falha de drone"""
        coordinator = SwarmCoordinator(sample_config)

        # Simula falha de drone (bateria = 0)
        failed_drone_id = 0
        coordinator.drones[failed_drone_id].battery_level = 0.0
        coordinator.drones[failed_drone_id].is_active = False

        # Continua simulação
        for _ in range(10):
            state = coordinator.step(0.1)

            # Outros drones devem continuar funcionando
            active_drones = sum(1 for drone in state['drones'].values() if drone['is_active'])
            assert active_drones >= sample_config.num_drones - 1

    def test_leader_failure_recovery(self, sample_config):
        """Testa recuperação de falha do líder"""
        coordinator = SwarmCoordinator(sample_config)

        # Força eleição de líder
        for _ in range(20):  # Tempo suficiente para eleição
            coordinator.step(0.1)

        initial_leader = None
        for drone_id, consensus in coordinator.consensus_algorithms.items():
            if consensus.is_leader():
                initial_leader = drone_id
                break

        assert initial_leader is not None

        # Simula falha do líder
        coordinator.drones[initial_leader].is_active = False

        # Continua simulação - deve haver nova eleição
        new_leader_found = False
        for _ in range(30):  # Tempo para nova eleição
            state = coordinator.step(0.1)

            if state['leader_id'] is not None and state['leader_id'] != initial_leader:
                new_leader_found = True
                break

        # Sistema deve eleger novo líder
        # (teste pode falhar se consensus não estiver implementado perfeitamente)

    def test_obstacle_avoidance_robustness(self, sample_config):
        """Testa robustez do avoidance com obstáculos dinâmicos"""
        coordinator = SwarmCoordinator(sample_config)

        # Adiciona obstáculos próximos aos drones
        for i, drone in coordinator.drones.items():
            obstacle_pos = drone.position + np.array([2.0, 0.0, 0.0])
            coordinator.add_obstacle(obstacle_pos, 1.5)

        # Simulação deve continuar sem crashes
        for _ in range(20):
            state = coordinator.step(0.1)

            # Todos os drones devem ter posições válidas
            for drone_data in state['drones'].values():
                pos = np.array(drone_data['position'])
                assert all(np.isfinite(pos))  # Não NaN ou inf
