"""
Testes unitários para o algoritmo de consenso (Raft-based).
"""

import time
from unittest.mock import Mock, patch

import numpy as np
import pytest
from backend.coordinator import ConsensusAlgorithm, ConsensusState, SwarmConfig


class TestConsensusAlgorithm:
    """Testes para ConsensusAlgorithm"""

    def test_initialization(self, sample_config):
        """Testa inicialização do algoritmo de consenso"""
        consensus = ConsensusAlgorithm(0, sample_config)

        assert consensus.drone_id == 0
        assert consensus.state == ConsensusState.FOLLOWER
        assert consensus.current_term == 0
        assert consensus.voted_for is None
        assert len(consensus.peers) == 0

    def test_receive_heartbeat_higher_term(self, sample_config):
        """Testa recebimento de heartbeat com termo maior"""
        consensus = ConsensusAlgorithm(0, sample_config)

        # Recebe heartbeat de líder com termo maior
        consensus.receive_heartbeat(1, 5)

        assert consensus.current_term == 5
        assert consensus.state == ConsensusState.FOLLOWER
        assert consensus.voted_for == 1

    def test_receive_heartbeat_lower_term(self, sample_config):
        """Testa recebimento de heartbeat com termo menor (ignorar)"""
        consensus = ConsensusAlgorithm(0, sample_config)
        consensus.current_term = 10

        # Recebe heartbeat com termo menor
        consensus.receive_heartbeat(1, 5)

        # Estado não deve mudar
        assert consensus.current_term == 10
        assert consensus.state == ConsensusState.FOLLOWER

    def test_start_election(self, sample_config):
        """Testa início de eleição"""
        consensus = ConsensusAlgorithm(0, sample_config)

        consensus.start_election()

        assert consensus.current_term == 1
        assert consensus.state == ConsensusState.CANDIDATE
        assert consensus.voted_for == 0  # Voto para si mesmo
        assert consensus.votes_received == 1

    def test_request_vote_first_time(self, sample_config):
        """Testa pedido de voto pela primeira vez"""
        consensus = ConsensusAlgorithm(0, sample_config)

        # Pedido de voto válido
        vote_granted = consensus.request_vote(1, 1)

        assert vote_granted is True
        assert consensus.voted_for == 1

    def test_request_vote_already_voted_same_term(self, sample_config):
        """Testa pedido de voto quando já votou no mesmo termo"""
        consensus = ConsensusAlgorithm(0, sample_config)
        consensus.voted_for = 1
        consensus.current_term = 1  # Mesmo termo

        # Pedido de voto de candidato diferente no mesmo termo (deve ser negado)
        vote_granted = consensus.request_vote(2, 1)

        assert vote_granted is False
        assert consensus.voted_for == 1  # Mantém voto anterior

    def test_request_vote_higher_term(self, sample_config):
        """Testa pedido de voto com termo maior"""
        consensus = ConsensusAlgorithm(0, sample_config)
        consensus.current_term = 5
        consensus.voted_for = 1

        # Pedido com termo maior
        vote_granted = consensus.request_vote(2, 10)

        assert vote_granted is True
        assert consensus.current_term == 10
        assert consensus.state == ConsensusState.FOLLOWER
        assert consensus.voted_for == 2

    def test_receive_vote_become_leader(self, sample_config):
        """Testa recebimento de votos suficiente para se tornar líder"""
        config = SwarmConfig(num_drones=3)  # 3 drones, maioria = 2
        consensus = ConsensusAlgorithm(0, config)

        # Inicia eleição
        consensus.start_election()
        assert consensus.state == ConsensusState.CANDIDATE

        # Recebe voto suficiente
        consensus.receive_vote(1)  # Recebe 2 votos totais (próprio + 1)

        assert consensus.state == ConsensusState.LEADER

    def test_receive_vote_insufficient_majority(self, sample_config):
        """Testa recebimento de votos insuficiente"""
        config = SwarmConfig(num_drones=5)  # 5 drones, maioria = 3
        consensus = ConsensusAlgorithm(0, config)

        # Inicia eleição
        consensus.start_election()
        consensus.receive_vote(1)  # Só 2 votos, precisa de 3

        assert consensus.state == ConsensusState.CANDIDATE

    def test_step_leader_sends_heartbeat(self, sample_config):
        """Testa que líder envia heartbeats periodicamente"""
        consensus = ConsensusAlgorithm(0, sample_config)
        consensus.state = ConsensusState.LEADER

        # Simula tempo inicial
        with patch('time.time', return_value=0.0):
            consensus.last_heartbeat = 0.0

            # Step com tempo suficiente para heartbeat
            with patch('time.time', return_value=1.5):  # > heartbeat_interval
                message = consensus.step(1.5)

        assert message is not None
        assert message['type'] == 'heartbeat'
        assert message['term'] == 0
        assert message['leader_id'] == 0

    def test_step_follower_timeout_starts_election(self, sample_config):
        """Testa timeout do follower iniciando eleição"""
        consensus = ConsensusAlgorithm(0, sample_config)
        consensus.state = ConsensusState.FOLLOWER

        # Simula timeout
        with patch('time.time', return_value=0.0):
            consensus.last_heartbeat = 0.0
            consensus.election_timeout = 1.0  # Curto para teste

            # Step após timeout
            with patch('time.time', return_value=2.0):  # > election_timeout
                message = consensus.step(2.0)

        assert message is not None
        assert message['type'] == 'vote_request'
        assert message['term'] == 1
        assert message['candidate_id'] == 0
        assert consensus.state == ConsensusState.CANDIDATE

    def test_is_leader(self, sample_config):
        """Testa verificação se é líder"""
        consensus = ConsensusAlgorithm(0, sample_config)

        assert consensus.is_leader() is False

        consensus.state = ConsensusState.LEADER
        assert consensus.is_leader() is True

    def test_multiple_terms(self, sample_config):
        """Testa múltiplos termos de eleição"""
        consensus = ConsensusAlgorithm(0, sample_config)

        # Primeira eleição
        consensus.start_election()
        assert consensus.current_term == 1

        # Recebe heartbeat com termo maior
        consensus.receive_heartbeat(1, 5)
        assert consensus.current_term == 5

        # Nova eleição
        consensus.start_election()
        assert consensus.current_term == 6
