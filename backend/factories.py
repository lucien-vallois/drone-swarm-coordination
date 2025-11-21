"""
Fábricas concretas para criação de componentes do sistema
Implementa a interface IComponentFactory para criação extensível
"""

import importlib
import time
from typing import Any, Dict, List, Type

import numpy as np

from backend.coordinator import (
    CollisionAvoidance,
    ConsensusAlgorithm,
    Formation,
    SwarmConfig,
    SwarmCoordinator,
)
from backend.interfaces import (
    ICollisionAvoidance,
    ICommunicationManager,
    IComponentFactory,
    IConsensusAlgorithm,
    IEnvironment,
    IFormationPlanner,
    IPathfindingAlgorithm,
    IPhysicsEngine,
)
from backend.pathfinding import FormationPathPlanner, Pathfinding3D
from backend.physics import DronePhysics, Environment, SwarmPhysics


class DefaultComponentFactory(IComponentFactory):
    """
    Fábrica padrão que cria implementações padrão dos componentes.

    Esta fábrica pode ser estendida ou substituída para criar
    versões customizadas dos componentes.
    """

    def __init__(self):
        # Mapeamento de tipos para classes concretas
        self._consensus_implementations = {
            'raft': ConsensusAlgorithm,
            # Pode ser extendido com outras implementações
        }

        self._pathfinding_implementations = {
            'astar_3d': Pathfinding3D,
            # Pode ser extendido com outras implementações
        }

        self._collision_implementations = {
            'potential_fields': CollisionAvoidance,
            # Pode ser extendido com outras implementações
        }

        self._physics_implementations = {
            'rigid_body': SwarmPhysics,
            # Pode ser extendido com outras implementações
        }

    def create_consensus_algorithm(self, drone_id: int, config: Dict[str, Any]) -> IConsensusAlgorithm:
        """
        Cria algoritmo de consenso baseado na configuração.

        Configuração esperada:
        {
            'algorithm': 'raft',  # ou outro tipo
            'swarm_config': SwarmConfig,
            ... outros parâmetros
        }
        """
        algorithm_type = config.get('algorithm', 'raft')
        swarm_config = config.get('swarm_config')

        if algorithm_type not in self._consensus_implementations:
            raise ValueError(f"Unknown consensus algorithm: {algorithm_type}")

        impl_class = self._consensus_implementations[algorithm_type]
        return impl_class(drone_id, swarm_config)

    def create_pathfinding_algorithm(self, config: Dict[str, Any]) -> IPathfindingAlgorithm:
        """
        Cria algoritmo de pathfinding baseado na configuração.

        Configuração esperada:
        {
            'algorithm': 'astar_3d',
            'bounds': (100, 100, 50),
            'resolution': 1.0,
            ... outros parâmetros
        }
        """
        algorithm_type = config.get('algorithm', 'astar_3d')
        bounds = config.get('bounds', (100, 100, 50))
        resolution = config.get('resolution', 1.0)

        if algorithm_type not in self._pathfinding_implementations:
            raise ValueError(f"Unknown pathfinding algorithm: {algorithm_type}")

        impl_class = self._pathfinding_implementations[algorithm_type]
        return impl_class(bounds, resolution)

    def create_collision_avoidance(self, config: Dict[str, Any]) -> ICollisionAvoidance:
        """
        Cria sistema de avoidance de colisão.

        Configuração esperada:
        {
            'algorithm': 'potential_fields',
            'swarm_config': SwarmConfig,
            ... outros parâmetros
        }
        """
        algorithm_type = config.get('algorithm', 'potential_fields')
        swarm_config = config.get('swarm_config')

        if algorithm_type not in self._collision_implementations:
            raise ValueError(f"Unknown collision avoidance algorithm: {algorithm_type}")

        impl_class = self._collision_implementations[algorithm_type]
        return impl_class(swarm_config)

    def create_physics_engine(self, config: Dict[str, Any]) -> IPhysicsEngine:
        """
        Cria motor de física.

        Configuração esperada:
        {
            'engine': 'rigid_body',
            'num_drones': 5,
            ... outros parâmetros
        }
        """
        engine_type = config.get('engine', 'rigid_body')
        num_drones = config.get('num_drones', 5)

        if engine_type not in self._physics_implementations:
            raise ValueError(f"Unknown physics engine: {engine_type}")

        impl_class = self._physics_implementations[engine_type]
        return impl_class(num_drones)

    def create_formation_planner(self, config: Dict[str, Any]) -> IFormationPlanner:
        """
        Cria planejador de formação.

        Configuração esperada:
        {
            'pathfinder': Pathfinding3D instance,
            ... outros parâmetros
        }
        """
        pathfinder = config.get('pathfinder')
        if pathfinder is None:
            raise ValueError("Formation planner requires a pathfinder instance")

        return FormationPathPlanner(pathfinder)

    def create_communication_manager(self, config: Dict[str, Any]) -> ICommunicationManager:
        """
        Cria gerenciador de comunicação.

        Configuração esperada:
        {
            'protocol': 'websocket',
            'latency': 0.0,
            'packet_loss': 0.0,
            ... outros parâmetros
        }
        """
        # Implementação básica - pode ser extendida
        protocol = config.get('protocol', 'ideal')  # Comunicação ideal por padrão

        if protocol == 'ideal':
            return IdealCommunicationManager(config)
        else:
            raise ValueError(f"Unknown communication protocol: {protocol}")

    def create_environment(self, config: Dict[str, Any]) -> IEnvironment:
        """
        Cria representação do ambiente.

        Configuração esperada:
        {
            'bounds': (100, 100, 50),
            'obstacles': [],
            ... outros parâmetros
        }
        """
        bounds = config.get('bounds', (100, 100, 50))
        return Environment(bounds)


class ModularSwarmCoordinator:
    """
    Coordenador de enxame modular que usa interfaces para extensibilidade.

    Este coordenador permite trocar componentes individuais sem modificar
    o código principal, facilitando extensões e testes.
    """

    def __init__(self, config: SwarmConfig, factory: IComponentFactory = None):
        self.config = config
        self.factory = factory or DefaultComponentFactory()

        # Componentes modulares
        self.consensus_algorithms: Dict[int, IConsensusAlgorithm] = {}
        self.pathfinder: IPathfindingAlgorithm = None
        self.collision_avoidance: ICollisionAvoidance = None
        self.formation_planner: IFormationPlanner = None
        self.physics_engine: IPhysicsEngine = None
        self.communication_manager: ICommunicationManager = None
        self.environment: IEnvironment = None

        # Estado interno
        self.formation = config.formation
        self.target_positions: Dict[int, np.ndarray] = {}
        self.paths: Dict[int, List[np.ndarray]] = {}

        # Inicialização
        self._initialize_components()

    def _initialize_components(self):
        """Inicializa todos os componentes usando a fábrica."""
        # Criar pathfinder
        self.pathfinder = self.factory.create_pathfinding_algorithm({
            'bounds': (100, 100, 50),
            'resolution': 1.0
        })

        # Criar collision avoidance
        self.collision_avoidance = self.factory.create_collision_avoidance({
            'swarm_config': self.config
        })

        # Criar formation planner
        self.formation_planner = self.factory.create_formation_planner({
            'pathfinder': self.pathfinder
        })

        # Criar physics engine
        self.physics_engine = self.factory.create_physics_engine({
            'num_drones': self.config.num_drones
        })

        # Criar communication manager
        self.communication_manager = self.factory.create_communication_manager({
            'protocol': 'ideal'
        })

        # Criar environment
        self.environment = self.factory.create_environment({
            'bounds': (100, 100, 50)
        })

        # Criar consensus algorithms para cada drone
        for i in range(self.config.num_drones):
            self.consensus_algorithms[i] = self.factory.create_consensus_algorithm(i, {
                'swarm_config': self.config
            })

        # Calcular formação inicial
        self._calculate_formation()

    def _calculate_formation(self):
        """Calcula formação usando o formation planner."""
        center = np.array([50, 50, 10])
        self.target_positions = self.formation_planner.calculate_formation(
            self.formation, self.config.num_drones, center
        )

        # Planejar caminhos
        current_positions = {}
        for drone_id in range(self.config.num_drones):
            # Posições simplificadas para inicialização
            current_positions[drone_id] = np.array([50, 50, 10])

        self.paths = self.formation_planner.plan_formation_paths(
            current_positions, self.target_positions
        )

    def step(self, dt: float) -> Dict[str, Any]:
        """Executa um passo de simulação usando componentes modulares."""
        # Implementação similar ao SwarmCoordinator original,
        # mas usando interfaces para maior flexibilidade
        # ... (implementação completa seria similar ao coordinator.py)

        # Por brevidade, retornando estrutura básica
        return {
            'timestamp': time.time(),
            'formation': self.formation.value,
            'target_positions': {id: pos.tolist() for id, pos in self.target_positions.items()}
        }


class IdealCommunicationManager(ICommunicationManager):
    """
    Gerenciador de comunicação ideal (sem latência ou perda de pacotes).

    Serve como implementação de referência e pode ser extendido
    com modelos de comunicação mais realistas.
    """

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.message_queues: Dict[int, List[Dict[str, Any]]] = {}

    def send_message(self, from_id: int, to_id: int, message: Dict[str, Any]) -> None:
        """Envia mensagem diretamente para fila do destinatário."""
        if to_id not in self.message_queues:
            self.message_queues[to_id] = []
        self.message_queues[to_id].append({
            'from': from_id,
            'timestamp': time.time(),
            'content': message
        })

    def broadcast_message(self, from_id: int, message: Dict[str, Any]) -> None:
        """Broadcast para todos os drones."""
        for drone_id in range(self.config.get('num_drones', 5)):
            if drone_id != from_id:
                self.send_message(from_id, drone_id, message)

    def receive_messages(self, drone_id: int) -> List[Dict[str, Any]]:
        """Retorna mensagens pendentes para o drone."""
        if drone_id not in self.message_queues:
            return []
        messages = self.message_queues[drone_id][:]
        self.message_queues[drone_id].clear()
        return messages

    def get_network_state(self) -> Dict[str, Any]:
        """Retorna estado da rede (ideal = perfeito)."""
        return {
            'latency': 0.0,
            'packet_loss': 0.0,
            'bandwidth': float('inf'),
            'connections': len(self.message_queues)
        }

    def simulate_network_conditions(self, latency: float = 0.0,
                                  packet_loss: float = 0.0) -> None:
        """Configura condições de rede (não aplicável para comunicação ideal)."""
        pass


# Função utilitária para criar configurador personalizado
def create_custom_factory(**component_mappings) -> IComponentFactory:
    """
    Cria fábrica customizada com mapeamentos específicos de componentes.

    Exemplo:
        factory = create_custom_factory(
            consensus_algorithm=MyCustomConsensus,
            pathfinding_algorithm=MyCustomPathfinder
        )
    """
    factory = DefaultComponentFactory()

    # Atualizar mapeamentos conforme especificado
    for component_type, implementation in component_mappings.items():
        if hasattr(factory, f'_{component_type}_implementations'):
            attr_name = f'_{component_type}_implementations'
            getattr(factory, attr_name)['custom'] = implementation

    return factory
