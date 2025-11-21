"""
Interfaces abstratas para o sistema de coordenação de enxames de drones
Define contratos para extensibilidade e modularidade
"""

from abc import ABC, abstractmethod
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


class ConsensusState(Enum):
    """Estados possíveis do algoritmo de consenso"""
    FOLLOWER = "follower"
    CANDIDATE = "candidate"
    LEADER = "leader"


class Formation(Enum):
    """Formações suportadas pelo enxame"""
    CIRCLE = "circle"
    LINE = "line"
    GRID = "grid"
    V_FORMATION = "v_formation"
    RANDOM = "random"


class IConsensusAlgorithm(ABC):
    """
    Interface para algoritmos de consenso distribuído.

    Esta interface define o contrato para implementações de consenso como Raft,
    Paxos, ou outros algoritmos distribuídos personalizados.
    """

    @property
    @abstractmethod
    def drone_id(self) -> int:
        """ID único do drone neste algoritmo."""
        pass

    @property
    @abstractmethod
    def state(self) -> ConsensusState:
        """Estado atual do algoritmo."""
        pass

    @abstractmethod
    def step(self, current_time: float) -> Optional[Dict[str, Any]]:
        """
        Executa um passo do algoritmo de consenso.

        Args:
            current_time: Tempo atual da simulação.

        Returns:
            Mensagem a ser enviada ou None se nenhuma ação necessária.
        """
        pass

    @abstractmethod
    def receive_heartbeat(self, leader_id: int, term: int) -> None:
        """
        Processa heartbeat recebido do líder.

        Args:
            leader_id: ID do líder enviando heartbeat.
            term: Termo atual do líder.
        """
        pass

    @abstractmethod
    def request_vote(self, candidate_id: int, term: int) -> bool:
        """
        Processa pedido de voto de candidato.

        Args:
            candidate_id: ID do candidato solicitando voto.
            term: Termo da eleição.

        Returns:
            True se voto concedido, False caso contrário.
        """
        pass

    @abstractmethod
    def receive_vote(self, voter_id: int) -> None:
        """
        Recebe voto de um eleitor.

        Args:
            voter_id: ID do drone que votou.
        """
        pass

    @abstractmethod
    def is_leader(self) -> bool:
        """Retorna True se este drone for líder."""
        pass

    @abstractmethod
    def get_state_info(self) -> Dict[str, Any]:
        """Retorna informações completas do estado do algoritmo."""
        pass


class IPathfindingAlgorithm(ABC):
    """
    Interface para algoritmos de planejamento de caminhos.

    Define contrato para implementações de pathfinding como A*, RRT*,
    ou algoritmos personalizados para navegação em 3D.
    """

    @abstractmethod
    def find_path(self, start: np.ndarray, goal: np.ndarray,
                  **kwargs) -> Optional[List[np.ndarray]]:
        """
        Encontra caminho do ponto inicial ao objetivo.

        Args:
            start: Posição inicial [x, y, z].
            goal: Posição objetivo [x, y, z].
            **kwargs: Parâmetros específicos do algoritmo.

        Returns:
            Lista de waypoints ou None se caminho não encontrado.
        """
        pass

    @abstractmethod
    def add_obstacle(self, position: np.ndarray, radius: float) -> None:
        """
        Adiciona obstáculo ao ambiente.

        Args:
            position: Centro do obstáculo [x, y, z].
            radius: Raio do obstáculo.
        """
        pass

    @abstractmethod
    def clear_obstacles(self) -> None:
        """Remove todos os obstáculos."""
        pass

    @abstractmethod
    def is_valid_position(self, position: np.ndarray) -> bool:
        """
        Verifica se posição é válida (dentro dos limites e sem colisão).

        Args:
            position: Posição a verificar.

        Returns:
            True se posição válida.
        """
        pass

    @abstractmethod
    def get_bounds(self) -> Tuple[float, float, float]:
        """Retorna limites do espaço (largura, altura, profundidade)."""
        pass

    @abstractmethod
    def optimize_path(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """
        Otimiza caminho existente (suavização, redução de waypoints).

        Args:
            path: Caminho original.

        Returns:
            Caminho otimizado.
        """
        pass


class ICollisionAvoidance(ABC):
    """
    Interface para algoritmos de avoidance de colisão.

    Define contrato para implementações de avoidance como campos de potencial,
    velocity obstacles, ou outros métodos de navegação multi-agente.
    """

    @abstractmethod
    def calculate_forces(self, drone_pos: np.ndarray, target_pos: np.ndarray,
                        neighbors: List[np.ndarray],
                        obstacles: List[np.ndarray]) -> np.ndarray:
        """
        Calcula forças resultantes para um drone.

        Args:
            drone_pos: Posição atual do drone.
            target_pos: Posição alvo do drone.
            neighbors: Posições dos drones vizinhos.
            obstacles: Lista de obstáculos [pos, radius].

        Returns:
            Vetor força total [fx, fy, fz].
        """
        pass

    @abstractmethod
    def avoid_collisions(self, swarm_positions: Dict[int, np.ndarray],
                        obstacles: List[np.ndarray]) -> Dict[int, np.ndarray]:
        """
        Calcula forças de avoidance para todo o enxame.

        Args:
            swarm_positions: Posições de todos os drones.
            obstacles: Obstáculos no ambiente.

        Returns:
            Forças de avoidance por drone.
        """
        pass

    @abstractmethod
    def detect_collisions(self, positions: Dict[int, np.ndarray],
                         threshold: float = 2.0) -> List[Tuple[int, int]]:
        """
        Detecta colisões entre drones.

        Args:
            positions: Posições de todos os drones.
            threshold: Distância mínima para considerar colisão.

        Returns:
            Lista de pares de drones em colisão.
        """
        pass

    @abstractmethod
    def get_parameters(self) -> Dict[str, Any]:
        """Retorna parâmetros atuais do algoritmo."""
        pass

    @abstractmethod
    def set_parameters(self, **params) -> None:
        """Atualiza parâmetros do algoritmo."""
        pass


class IPhysicsEngine(ABC):
    """
    Interface para motores de física.

    Define contrato para simulações físicas, incluindo dinâmica de corpos rígidos,
    aerodinâmica, e interações ambientais.
    """

    @abstractmethod
    def step(self, dt: float, **kwargs) -> None:
        """
        Executa um passo da simulação física.

        Args:
            dt: Intervalo de tempo.
            **kwargs: Parâmetros específicos (vento, etc.).
        """
        pass

    @abstractmethod
    def set_state(self, drone_id: int, **state) -> None:
        """
        Define estado de um drone específico.

        Args:
            drone_id: ID do drone.
            **state: Estado a definir (posição, velocidade, etc.).
        """
        pass

    @abstractmethod
    def get_state(self, drone_id: int) -> Dict[str, Any]:
        """
        Retorna estado completo de um drone.

        Args:
            drone_id: ID do drone.

        Returns:
            Estado físico do drone.
        """
        pass

    @abstractmethod
    def get_swarm_state(self) -> Dict[int, Dict[str, Any]]:
        """Retorna estado de todos os drones."""
        pass

    @abstractmethod
    def add_force(self, drone_id: int, force: np.ndarray) -> None:
        """
        Adiciona força externa a um drone.

        Args:
            drone_id: ID do drone.
            force: Vetor força [fx, fy, fz].
        """
        pass

    @abstractmethod
    def reset(self) -> None:
        """Reseta simulação para estado inicial."""
        pass


class IFormationPlanner(ABC):
    """
    Interface para planejadores de formação.

    Define contrato para algoritmos que geram e mantêm formações
    específicas do enxame.
    """

    @abstractmethod
    def calculate_formation(self, formation_type: Formation,
                           num_drones: int, center: np.ndarray,
                           **params) -> Dict[int, np.ndarray]:
        """
        Calcula posições alvo para formação específica.

        Args:
            formation_type: Tipo de formação.
            num_drones: Número de drones.
            center: Centro da formação.
            **params: Parâmetros específicos da formação.

        Returns:
            Posições alvo por drone.
        """
        pass

    @abstractmethod
    def transition_formation(self, current_positions: Dict[int, np.ndarray],
                           target_positions: Dict[int, np.ndarray],
                           time_factor: float) -> Dict[int, np.ndarray]:
        """
        Calcula posições intermediárias para transição suave.

        Args:
            current_positions: Posições atuais.
            target_positions: Posições alvo.
            time_factor: Fator de interpolação [0, 1].

        Returns:
            Posições interpoladas.
        """
        pass

    @abstractmethod
    def optimize_formation(self, positions: Dict[int, np.ndarray],
                          constraints: Dict[str, Any]) -> Dict[int, np.ndarray]:
        """
        Otimiza formação considerando restrições.

        Args:
            positions: Posições atuais da formação.
            constraints: Restrições a considerar.

        Returns:
            Posições otimizadas.
        """
        pass

    @abstractmethod
    def validate_formation(self, positions: Dict[int, np.ndarray],
                          formation_type: Formation) -> bool:
        """
        Valida se posições formam formação correta.

        Args:
            positions: Posições a validar.
            formation_type: Tipo esperado.

        Returns:
            True se formação válida.
        """
        pass


class ICommunicationManager(ABC):
    """
    Interface para gerenciamento de comunicações.

    Define contrato para protocolos de comunicação entre drones,
    incluindo latência, perda de pacotes, e topologias de rede.
    """

    @abstractmethod
    def send_message(self, from_id: int, to_id: int, message: Dict[str, Any]) -> None:
        """
        Envia mensagem de um drone para outro.

        Args:
            from_id: ID do remetente.
            to_id: ID do destinatário.
            message: Conteúdo da mensagem.
        """
        pass

    @abstractmethod
    def broadcast_message(self, from_id: int, message: Dict[str, Any]) -> None:
        """
        Broadcast mensagem para todos os drones.

        Args:
            from_id: ID do remetente.
            message: Conteúdo da mensagem.
        """
        pass

    @abstractmethod
    def receive_messages(self, drone_id: int) -> List[Dict[str, Any]]:
        """
        Recebe mensagens pendentes para um drone.

        Args:
            drone_id: ID do drone.

        Returns:
            Lista de mensagens recebidas.
        """
        pass

    @abstractmethod
    def get_network_state(self) -> Dict[str, Any]:
        """Retorna estado atual da rede de comunicação."""
        pass

    @abstractmethod
    def simulate_network_conditions(self, latency: float = 0.0,
                                  packet_loss: float = 0.0) -> None:
        """
        Simula condições de rede realistas.

        Args:
            latency: Latência média em segundos.
            packet_loss: Taxa de perda de pacotes [0, 1].
        """
        pass


class ISwarmCoordinator(ABC):
    """
    Interface principal para coordenadores de enxame.

    Define contrato para sistemas de coordenação que integram
    consenso, pathfinding, física, e comunicação.
    """

    @abstractmethod
    def step(self, dt: float) -> Dict[str, Any]:
        """
        Executa um passo completo da simulação.

        Args:
            dt: Intervalo de tempo.

        Returns:
            Estado atual do enxame.
        """
        pass

    @abstractmethod
    def set_formation(self, formation: Formation) -> None:
        """
        Altera formação do enxame.

        Args:
            formation: Nova formação.
        """
        pass

    @abstractmethod
    def add_obstacle(self, position: np.ndarray, radius: float) -> None:
        """
        Adiciona obstáculo ao ambiente.

        Args:
            position: Posição do obstáculo.
            radius: Raio do obstáculo.
        """
        pass

    @abstractmethod
    def get_swarm_state(self) -> Dict[str, Any]:
        """Retorna estado completo do enxame."""
        pass

    @abstractmethod
    def reset(self) -> None:
        """Reseta enxame para estado inicial."""
        pass

    @abstractmethod
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Retorna métricas de performance do enxame."""
        pass


class IEnvironment(ABC):
    """
    Interface para representação do ambiente.

    Define contrato para modelos ambientais incluindo terreno,
    obstáculos, condições climáticas, e restrições espaciais.
    """

    @abstractmethod
    def is_position_valid(self, position: np.ndarray) -> bool:
        """
        Verifica se posição é válida no ambiente.

        Args:
            position: Posição a verificar.

        Returns:
            True se posição válida.
        """
        pass

    @abstractmethod
    def get_environmental_forces(self, position: np.ndarray) -> np.ndarray:
        """
        Calcula forças ambientais em uma posição.

        Args:
            position: Posição para calcular forças.

        Returns:
            Vetor de forças ambientais [fx, fy, fz].
        """
        pass

    @abstractmethod
    def update_environment(self, dt: float) -> None:
        """
        Atualiza estado dinâmico do ambiente.

        Args:
            dt: Intervalo de tempo.
        """
        pass

    @abstractmethod
    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Retorna limites do ambiente (min_corner, max_corner)."""
        pass

    @abstractmethod
    def add_dynamic_obstacle(self, obstacle_id: str,
                           position: np.ndarray, velocity: np.ndarray,
                           radius: float) -> None:
        """
        Adiciona obstáculo dinâmico ao ambiente.

        Args:
            obstacle_id: ID único do obstáculo.
            position: Posição inicial.
            velocity: Velocidade do obstáculo.
            radius: Raio do obstáculo.
        """
        pass

    @abstractmethod
    def get_environment_state(self) -> Dict[str, Any]:
        """Retorna estado completo do ambiente."""
        pass


# Factory interfaces para criação de componentes
class IComponentFactory(ABC):
    """
    Interface para fábricas de componentes.

    Permite criação extensível de componentes do sistema
    através de configuração declarativa.
    """

    @abstractmethod
    def create_consensus_algorithm(self, drone_id: int, config: Dict[str, Any]) -> IConsensusAlgorithm:
        """Cria algoritmo de consenso."""
        pass

    @abstractmethod
    def create_pathfinding_algorithm(self, config: Dict[str, Any]) -> IPathfindingAlgorithm:
        """Cria algoritmo de pathfinding."""
        pass

    @abstractmethod
    def create_collision_avoidance(self, config: Dict[str, Any]) -> ICollisionAvoidance:
        """Cria sistema de avoidance de colisão."""
        pass

    @abstractmethod
    def create_physics_engine(self, config: Dict[str, Any]) -> IPhysicsEngine:
        """Cria motor de física."""
        pass

    @abstractmethod
    def create_formation_planner(self, config: Dict[str, Any]) -> IFormationPlanner:
        """Cria planejador de formação."""
        pass

    @abstractmethod
    def create_communication_manager(self, config: Dict[str, Any]) -> ICommunicationManager:
        """Cria gerenciador de comunicação."""
        pass

    @abstractmethod
    def create_environment(self, config: Dict[str, Any]) -> IEnvironment:
        """Cria representação do ambiente."""
        pass
