"""
Testes unitários para o módulo de simulação (SwarmSimulation e WebSocketServer).
"""

import asyncio
import json
import time
from unittest.mock import AsyncMock, Mock, patch

import numpy as np
import pytest
from backend.coordinator import Formation, SwarmConfig
from backend.simulation import SwarmSimulation, WebSocketServer


class TestSwarmSimulation:
    """Testes para SwarmSimulation"""

    def test_initialization(self, sample_config):
        """Testa inicialização da simulação"""
        simulation = SwarmSimulation(sample_config)

        assert simulation.config == sample_config
        assert simulation.running is False
        assert len(simulation.websocket_clients) == 0
        assert simulation.frame_count == 0

    def test_start_simulation(self, sample_config):
        """Testa início da simulação"""
        simulation = SwarmSimulation(sample_config)

        simulation.start_simulation()

        assert simulation.running is True
        assert simulation.simulation_thread is not None
        assert simulation.simulation_thread.is_alive()

        # Para a simulação
        simulation.stop_simulation()

    def test_stop_simulation(self, sample_config):
        """Testa parada da simulação"""
        simulation = SwarmSimulation(sample_config)

        simulation.start_simulation()
        time.sleep(0.1)  # Aguarda thread iniciar

        simulation.stop_simulation()

        assert simulation.running is False

    def test_add_remove_websocket_client(self, sample_config):
        """Testa adição e remoção de clientes WebSocket"""
        simulation = SwarmSimulation(sample_config)
        mock_client = Mock()

        simulation.add_websocket_client(mock_client)
        assert len(simulation.websocket_clients) == 1
        assert mock_client in simulation.websocket_clients

        simulation.remove_websocket_client(mock_client)
        assert len(simulation.websocket_clients) == 0

    def test_set_formation(self, sample_config):
        """Testa mudança de formação"""
        simulation = SwarmSimulation(sample_config)

        simulation.set_formation('line')
        assert simulation.coordinator.formation == Formation.LINE

        simulation.set_formation('circle')
        assert simulation.coordinator.formation == Formation.CIRCLE

    def test_set_formation_invalid(self, sample_config):
        """Testa mudança para formação inválida"""
        simulation = SwarmSimulation(sample_config)

        # Não deve causar crash, apenas logar erro
        simulation.set_formation('invalid_formation')

    def test_add_obstacle(self, sample_config):
        """Testa adição de obstáculo"""
        simulation = SwarmSimulation(sample_config)

        initial_count = len(simulation.coordinator.pathfinder.obstacles)

        simulation.add_obstacle(30.0, 40.0, 10.0, 3.0)

        assert len(simulation.coordinator.pathfinder.obstacles) == initial_count + 1

    def test_reset_simulation(self, sample_config):
        """Testa reset da simulação"""
        simulation = SwarmSimulation(sample_config)

        # Modifica estado
        simulation.add_obstacle(10, 10, 10, 2.0)
        simulation.frame_count = 100

        simulation.reset_simulation()

        assert simulation.frame_count == 0
        # Obstáculos podem ou não ser resetados dependendo da implementação

    def test_prepare_state(self, sample_config):
        """Testa preparação do estado para broadcast"""
        simulation = SwarmSimulation(sample_config)

        # Executa um step do coordinator
        coord_state = simulation.coordinator.step(0.1)

        state = simulation._prepare_state(coord_state)

        assert isinstance(state, dict)
        assert 'timestamp' in state
        assert 'frame' in state
        assert 'fps' in state
        assert 'coordinator' in state
        assert 'environment' in state

    @pytest.mark.asyncio
    async def test_broadcast_state(self, sample_config):
        """Testa broadcast de estado para clientes"""
        simulation = SwarmSimulation(sample_config)

        # Adiciona cliente mock
        mock_client = AsyncMock()
        simulation.add_websocket_client(mock_client)

        state = {
            'timestamp': time.time(),
            'frame': 1,
            'fps': 60.0,
            'coordinator': {},
            'environment': {}
        }

        await simulation._broadcast_state(state)

        # Verifica que mensagem foi enviada
        mock_client.send.assert_called_once()
        sent_message = json.loads(mock_client.send.call_args[0][0])
        assert sent_message['frame'] == 1

    @pytest.mark.asyncio
    async def test_broadcast_state_disconnected_client(self, sample_config):
        """Testa broadcast com cliente desconectado"""
        simulation = SwarmSimulation(sample_config)

        # Cliente que falha ao enviar
        mock_client = AsyncMock()
        mock_client.send.side_effect = Exception("Connection closed")
        simulation.add_websocket_client(mock_client)

        state = {'timestamp': time.time(), 'frame': 1, 'fps': 60.0}

        # Não deve causar erro
        await simulation._broadcast_state(state)

        # Cliente deve ser removido
        assert mock_client not in simulation.websocket_clients


class TestWebSocketServer:
    """Testes para WebSocketServer"""

    def test_initialization(self, sample_config):
        """Testa inicialização do servidor WebSocket"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation, host="localhost", port=8765)

        assert server.simulation == simulation
        assert server.host == "localhost"
        assert server.port == 8765

    @pytest.mark.asyncio
    async def test_handle_client_connection(self, sample_config):
        """Testa tratamento de conexão de cliente"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        mock_websocket = AsyncMock()
        mock_websocket.__aiter__ = Mock(return_value=iter([]))

        # Simula conexão e desconexão imediata
        await server.handle_client(mock_websocket, "/")

        # Cliente deve ter sido adicionado e removido
        assert len(simulation.websocket_clients) == 0

    @pytest.mark.asyncio
    async def test_handle_message_start_command(self, sample_config):
        """Testa comando start"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        mock_ws = AsyncMock()
        message = {'type': 'command', 'command': 'start'}

        await server._handle_message(mock_ws, message)

        assert simulation.running is True
        mock_ws.send.assert_called_once()
        response = json.loads(mock_ws.send.call_args[0][0])
        assert response['status'] == 'started'

        simulation.stop_simulation()

    @pytest.mark.asyncio
    async def test_handle_message_stop_command(self, sample_config):
        """Testa comando stop"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        simulation.start_simulation()

        mock_ws = AsyncMock()
        message = {'type': 'command', 'command': 'stop'}

        await server._handle_message(mock_ws, message)

        assert simulation.running is False
        response = json.loads(mock_ws.send.call_args[0][0])
        assert response['status'] == 'stopped'

    @pytest.mark.asyncio
    async def test_handle_message_reset_command(self, sample_config):
        """Testa comando reset"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        mock_ws = AsyncMock()
        message = {'type': 'command', 'command': 'reset'}

        await server._handle_message(mock_ws, message)

        response = json.loads(mock_ws.send.call_args[0][0])
        assert response['status'] == 'reset'

    @pytest.mark.asyncio
    async def test_handle_message_set_formation(self, sample_config):
        """Testa comando set_formation"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        mock_ws = AsyncMock()
        message = {
            'type': 'command',
            'command': 'set_formation',
            'params': {'formation': 'line'}
        }

        await server._handle_message(mock_ws, message)

        assert simulation.coordinator.formation == Formation.LINE
        response = json.loads(mock_ws.send.call_args[0][0])
        assert response['status'] == 'formation_set'

    @pytest.mark.asyncio
    async def test_handle_message_add_obstacle(self, sample_config):
        """Testa comando add_obstacle"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        initial_count = len(simulation.coordinator.pathfinder.obstacles)

        mock_ws = AsyncMock()
        message = {
            'type': 'command',
            'command': 'add_obstacle',
            'params': {'x': 30.0, 'y': 40.0, 'z': 10.0, 'radius': 3.0}
        }

        await server._handle_message(mock_ws, message)

        assert len(simulation.coordinator.pathfinder.obstacles) == initial_count + 1
        response = json.loads(mock_ws.send.call_args[0][0])
        assert response['status'] == 'obstacle_added'

    @pytest.mark.asyncio
    async def test_handle_message_invalid_command(self, sample_config):
        """Testa comando inválido"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        mock_ws = AsyncMock()
        message = {
            'type': 'command',
            'command': 'invalid_command'
        }

        await server._handle_message(mock_ws, message)

        response = json.loads(mock_ws.send.call_args[0][0])
        assert 'error' in response

    @pytest.mark.asyncio
    async def test_handle_message_request_state(self, sample_config):
        """Testa request de estado"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        mock_ws = AsyncMock()
        message = {'type': 'request', 'request': 'state'}

        await server._handle_message(mock_ws, message)

        mock_ws.send.assert_called_once()
        response = json.loads(mock_ws.send.call_args[0][0])
        assert response['type'] == 'state'
        assert 'data' in response

    @pytest.mark.asyncio
    async def test_handle_message_request_config(self, sample_config):
        """Testa request de configuração"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        mock_ws = AsyncMock()
        message = {'type': 'request', 'request': 'config'}

        await server._handle_message(mock_ws, message)

        response = json.loads(mock_ws.send.call_args[0][0])
        assert response['type'] == 'config'
        assert 'data' in response
        assert response['data']['num_drones'] == sample_config.num_drones

    @pytest.mark.asyncio
    async def test_handle_message_invalid_json(self, sample_config):
        """Testa tratamento de JSON inválido"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        mock_ws = AsyncMock()
        mock_websocket = AsyncMock()
        mock_websocket.__aiter__ = Mock(return_value=iter(['invalid json']))

        # Simula erro de JSON
        try:
            await server.handle_client(mock_websocket, "/")
        except Exception:
            pass  # Esperado que falhe

    @pytest.mark.asyncio
    async def test_handle_message_exception(self, sample_config):
        """Testa tratamento de exceções"""
        simulation = SwarmSimulation(sample_config)
        server = WebSocketServer(simulation)

        mock_ws = AsyncMock()
        # Mensagem que causa erro
        message = {'type': 'command', 'command': 'start'}

        # Simula erro no processamento
        with patch.object(simulation, 'start_simulation', side_effect=Exception("Test error")):
            await server._handle_message(mock_ws, message)

            # Deve enviar mensagem de erro
            response = json.loads(mock_ws.send.call_args[0][0])
            assert 'error' in response







