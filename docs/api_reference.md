# Drone Swarm Coordination API Reference

Esta documentação fornece especificações completas das APIs internas do simulador de coordenação de enxames de drones, incluindo todos os métodos, parâmetros, retornos e comportamentos.

## Índice

- [Coordinator Module](#coordinator-module)
  - [SwarmCoordinator](#swarmcoordinator)
  - [ConsensusAlgorithm](#consensusalgorithm)
  - [CollisionAvoidance](#collisionavoidance)
- [Pathfinding Module](#pathfinding-module)
  - [Pathfinding3D](#pathfinding3d)
  - [FormationPathPlanner](#formationpathplanner)
- [Physics Module](#physics-module)
  - [DronePhysics](#dronephysics)
  - [SwarmPhysics](#swarmphysics)
  - [WindField](#windfield)
- [Simulation Module](#simulation-module)
  - [SwarmSimulation](#swarmsimulation)
  - [WebSocketServer](#websocketserver)

---

## Coordinator Module

### SwarmCoordinator

Classe principal que coordena todas as operações do enxame, incluindo consenso, pathfinding e formação.

#### Inicialização

```python
def __init__(self, config: SwarmConfig):
    """
    Inicializa o coordenador do enxame.

    Args:
        config (SwarmConfig): Configuração do enxame incluindo número de drones,
                             distâncias de separação, velocidade máxima, etc.

    Raises:
        ValueError: Se a configuração for inválida (número negativo de drones, etc.)
    """
```

#### Métodos Principais

```python
def step(self, dt: float) -> Dict[str, Any]:
    """
    Executa um passo de simulação completo.

    Args:
        dt (float): Tempo delta em segundos para integração física.

    Returns:
        Dict[str, Any]: Estado atual do enxame contendo:
            - 'drones': Lista de estados de drones
            - 'leader_id': ID do líder atual ou None
            - 'formation': Formação atual como string
            - 'target_positions': Posições alvo para formação
            - 'timestamp': Timestamp da simulação

    Raises:
        RuntimeError: Se o enxame não foi inicializado corretamente
    """
```

```python
def set_formation(self, formation: Formation):
    """
    Altera a formação do enxame.

    Args:
        formation (Formation): Nova formação (CIRCLE, LINE, GRID, V_FORMATION).

    Effects:
        - Recalcula todas as posições alvo
        - Planejamento de caminhos para transição suave
        - Atualiza estado interno do enxame
    """
```

```python
def add_obstacle(self, position: np.ndarray, radius: float):
    """
    Adiciona obstáculo ao ambiente.

    Args:
        position (np.ndarray): Posição [x, y, z] do obstáculo.
        radius (float): Raio do obstáculo em metros.

    Effects:
        - Adiciona obstáculo ao pathfinder
        - Força replanejamento de caminhos afetados
        - Atualiza forças de colisão
    """
```

#### Métodos Internos

```python
def _initialize_drones(self):
    """
    Inicializa estados e algoritmos de consenso para todos os drones.

    Internal:
        - Cria instâncias DroneState para cada drone
        - Inicializa ConsensusAlgorithm para cada drone
        - Define posições iniciais aleatórias
        - Calcula formação inicial
    """
```

```python
def _calculate_formation(self):
    """
    Calcula posições alvo para a formação atual.

    Internal:
        - Usa algoritmos específicos para cada tipo de formação
        - Gera caminhos do pathfinder para cada drone
        - Atualiza target_positions e paths
    """
```

```python
def _process_consensus_message(self, message: Dict):
    """
    Processa mensagens do algoritmo de consenso.

    Args:
        message (Dict): Mensagem contendo tipo, ID do emissor e dados específicos.

    Internal:
        - Roteia heartbeats para todos os algoritmos de consenso
        - Coordena eleições e votos
        - Atualiza estados de liderança
    """
```

### ConsensusAlgorithm

Implementa algoritmo de consenso baseado em Raft para coordenação do enxame.

#### Inicialização

```python
def __init__(self, drone_id: int, config: SwarmConfig):
    """
    Inicializa algoritmo de consenso para um drone específico.

    Args:
        drone_id (int): ID único do drone neste algoritmo.
        config (SwarmConfig): Configuração do enxame.

    Attributes:
        - state: Estado atual (FOLLOWER, CANDIDATE, LEADER)
        - current_term: Termo atual da eleição
        - voted_for: ID do candidato votado neste termo
        - votes_received: Número de votos recebidos (apenas candidato)
        - election_timeout: Timeout para iniciar eleição
        - last_heartbeat: Timestamp do último heartbeat
        - peers: Dict de peers ativos
    """
```

#### Métodos Principais

```python
def step(self, current_time: float) -> Optional[Dict]:
    """
    Executa um passo do algoritmo de consenso.

    Args:
        current_time (float): Tempo atual da simulação.

    Returns:
        Optional[Dict]: Mensagem a ser enviada ou None se nenhuma ação necessária.
                        Tipos possíveis: 'heartbeat', 'vote_request'

    Behavior:
        - Líder: Envia heartbeats periodicamente
        - Seguidor/Candidato: Verifica timeout de eleição
        - Inicia eleição se necessário
    """
```

```python
def receive_heartbeat(self, leader_id: int, term: int):
    """
    Processa heartbeat recebido do líder.

    Args:
        leader_id (int): ID do líder enviando o heartbeat.
        term (int): Termo do líder.

    Effects:
        - Atualiza estado para FOLLOWER se termo for maior
        - Define voted_for como leader_id
        - Reseta last_heartbeat
        - Converte candidato para seguidor se necessário
    """
```

```python
def request_vote(self, candidate_id: int, term: int) -> bool:
    """
    Processa pedido de voto de um candidato.

    Args:
        candidate_id (int): ID do candidato solicitando voto.
        term (int): Termo do candidato.

    Returns:
        bool: True se voto for concedido, False caso contrário.

    Conditions for granting vote:
        - Termo do candidato > termo atual
        - Ainda não votou neste termo OU já votou neste candidato
        - Candidato não é o próprio drone
    """
```

```python
def receive_vote(self, voter_id: int):
    """
    Recebe voto de um eleitor.

    Args:
        voter_id (int): ID do drone que votou neste candidato.

    Effects:
        - Incrementa votes_received
        - Verifica se alcançou maioria
        - Promove para LEADER se maioria atingida
        - Envia heartbeat inicial como líder
    """
```

#### Métodos de Estado

```python
def is_leader(self) -> bool:
    """Retorna True se este drone for o líder atual."""
    return self.state == ConsensusState.LEADER

def get_state(self) -> Dict:
    """Retorna estado completo do algoritmo de consenso."""
    return {
        'state': self.state.value,
        'term': self.current_term,
        'voted_for': self.voted_for,
        'votes_received': self.votes_received
    }
```

### CollisionAvoidance

Implementa campos de potencial artificial para evitar colisões.

#### Inicialização

```python
def __init__(self, config: SwarmConfig):
    """
    Inicializa sistema de avoidance de colisão.

    Args:
        config (SwarmConfig): Configuração do enxame.

    Attributes:
        - repulsive_gain: Ganho das forças repulsivas (10.0)
        - attractive_gain: Ganho das forças atrativas (1.0)
        - drone_radius: Raio efetivo do drone (0.5m)
    """
```

#### Métodos Principais

```python
def calculate_force(self, drone_pos: np.ndarray, target_pos: np.ndarray,
                   neighbors: List[np.ndarray], obstacles: List[np.ndarray]) -> np.ndarray:
    """
    Calcula força total resultante dos campos de potencial.

    Args:
        drone_pos (np.ndarray): Posição atual do drone [x, y, z].
        target_pos (np.ndarray): Posição alvo do drone [x, y, z].
        neighbors (List[np.ndarray]): Posições dos drones vizinhos.
        obstacles (List[np.ndarray]): Lista de obstáculos [pos, radius].

    Returns:
        np.ndarray: Vetor de força total [fx, fy, fz] em N.

    Force components:
        - Attractive: Para o alvo (proporcional à distância)
        - Repulsive (neighbors): Inversamente proporcional à distância
        - Repulsive (obstacles): Campos repulsivos esféricos
    """
```

```python
def avoid_collisions(self, swarm_positions: Dict[int, np.ndarray],
                    obstacles: List[np.ndarray]) -> Dict[int, np.ndarray]:
    """
    Calcula forças de avoidance para todos os drones do enxame.

    Args:
        swarm_positions (Dict[int, np.ndarray]): Posições de todos os drones.
        obstacles (List[np.ndarray]): Lista de obstáculos no ambiente.

    Returns:
        Dict[int, np.ndarray]: Forças de avoidance para cada drone.

    Note:
        Usa posições atuais como alvos temporários para focar apenas em avoidance.
    """
```

#### Constantes Internas

```python
REPULSIVE_GAIN = 10.0          # Ganho de forças repulsivas
ATTRACTIVE_GAIN = 1.0          # Ganho de forças atrativas
DRONE_RADIUS = 0.5            # Raio efetivo do drone (m)
COLLISION_THRESHOLD = 2.0      # Distância de influência (m)
```

---

## Pathfinding Module

### Pathfinding3D

Classe principal para planejamento de caminhos em 3D com avoidance de obstáculos.

#### Inicialização

```python
def __init__(self, bounds: Tuple[float, float, float], resolution: float = 1.0):
    """
    Inicializa pathfinder 3D.

    Args:
        bounds (Tuple[float, float, float]): Limites do espaço (largura, altura, profundidade).
        resolution (float): Resolução da grade de busca (metros).

    Attributes:
        - obstacles: Lista de obstáculos (posição, raio)
        - bounds: Limites do espaço de busca
        - resolution: Resolução da discretização
    """
```

#### Métodos de Obstáculos

```python
def add_obstacle(self, position: np.ndarray, radius: float):
    """
    Adiciona obstáculo esférico ao ambiente.

    Args:
        position (np.ndarray): Centro do obstáculo [x, y, z].
        radius (float): Raio do obstáculo.

    Effects:
        - Adiciona à lista de obstáculos
        - Afeta futuras chamadas de pathfinding
    """
```

```python
def clear_obstacles(self):
    """Remove todos os obstáculos do ambiente."""
    self.obstacles.clear()
```

#### Validação de Posições

```python
def is_valid_position(self, position: np.ndarray) -> bool:
    """
    Verifica se uma posição é válida (dentro dos limites e sem colisão).

    Args:
        position (np.ndarray): Posição a verificar [x, y, z].

    Returns:
        bool: True se posição for válida.

    Validation checks:
        - Dentro dos bounds do espaço
        - Não colide com obstáculos (distância > raio)
    """
```

#### Algoritmos de Pathfinding

```python
def astar(self, start: np.ndarray, goal: np.ndarray) -> Optional[List[np.ndarray]]:
    """
    Encontra caminho ótimo usando algoritmo A*.

    Args:
        start (np.ndarray): Posição inicial [x, y, z].
        goal (np.ndarray): Posição alvo [x, y, z].

    Returns:
        Optional[List[np.ndarray]]: Lista de waypoints ou None se não encontrar caminho.

    Algorithm details:
        - Usa heurística Euclidiana (admissível)
        - Explora 26 vizinhos em 3D
        - Garante otimalidade do caminho encontrado
        - Complexidade: O(b^d) onde b=ramificação, d=dimensões
    """
```

```python
def rrt_star(self, start: np.ndarray, goal: np.ndarray,
             max_iterations: int = 1000, search_radius: float = 5.0) -> Optional[List[np.ndarray]]:
    """
    Encontra caminho usando RRT* (assintoticamente ótimo).

    Args:
        start (np.ndarray): Posição inicial [x, y, z].
        goal (np.ndarray): Posição alvo [x, y, z].
        max_iterations (int): Máximo de iterações de amostragem.
        search_radius (float): Raio para rewiring de nós próximos.

    Returns:
        Optional[List[np.ndarray]]: Lista de waypoints ou None.

    Algorithm features:
        - Probabilistically complete
        - Assintoticamente ótimo
        - Efetivo em espaços de alta dimensão
        - Suporte a rewiring para otimização incremental
    """
```

#### Utilitários

```python
def smooth_path(self, path: List[np.ndarray], max_iterations: int = 10) -> List[np.ndarray]:
    """
    Suaviza caminho removendo waypoints desnecessários.

    Args:
        path (List[np.ndarray]): Caminho original.
        max_iterations (int): Máximo de iterações de suavização.

    Returns:
        List[np.ndarray]: Caminho suavizado.

    Algorithm:
        - Remove waypoints quando possível conectar pontos não adjacentes
        - Verifica colisões em segmentos direto
        - Itera até convergência ou max_iterations
    """
```

```python
def collision_free(self, a: np.ndarray, b: np.ndarray, steps: int = 10) -> bool:
    """
    Verifica se segmento de reta entre dois pontos é livre de colisão.

    Args:
        a (np.ndarray): Ponto inicial.
        b (np.ndarray): Ponto final.
        steps (int): Número de pontos de verificação ao longo do segmento.

    Returns:
        bool: True se segmento for livre de colisão.
    """
```

### FormationPathPlanner

Planejador de caminhos especializado para formações de enxame.

#### Inicialização

```python
def __init__(self, pathfinder: Pathfinding3D):
    """
    Inicializa planejador de formações.

    Args:
        pathfinder (Pathfinding3D): Instância do pathfinder subjacente.
    """
```

#### Métodos Principais

```python
def plan_formation_paths(self, current_positions: Dict[int, np.ndarray],
                        target_positions: Dict[int, np.ndarray]) -> Dict[int, List[np.ndarray]]:
    """
    Planeja caminhos para transição de formação.

    Args:
        current_positions (Dict[int, np.ndarray]): Posições atuais dos drones.
        target_positions (Dict[int, np.ndarray]): Posições alvo da formação.

    Returns:
        Dict[int, List[np.ndarray]]: Caminhos planejados para cada drone.

    Strategy:
        - Tenta A* primeiro (ótimo mas pode falhar)
        - Fallback para RRT* (mais robusto)
        - Linha direta como último recurso
    """
```

```python
def optimize_formation_paths(self, paths: Dict[int, List[np.ndarray]]) -> Dict[int, List[np.ndarray]]:
    """
    Otimiza caminhos para reduzir conflitos entre drones.

    Args:
        paths (Dict[int, List[np.ndarray]]): Caminhos iniciais.

    Returns:
        Dict[int, List[np.ndarray]]: Caminhos otimizados.

    Optimization:
        - Verifica separação mínima entre caminhos simultâneos
        - Remove waypoints que causam conflitos
        - Mantém conectividade para posições alvo
    """
```

---

## Physics Module

### DronePhysics

Simulação física detalhada para drone individual.

#### Inicialização

```python
def __init__(self, mass: float = 1.0, max_thrust: float = 20.0,
             drag_coefficient: float = 0.1, wind_speed: float = 0.0):
    """
    Inicializa física do drone.

    Args:
        mass (float): Massa do drone (kg).
        max_thrust (float): Empuxo máximo (N).
        drag_coefficient (float): Coeficiente de arrasto.
        wind_speed (float): Velocidade base do vento (m/s).

    Physical properties:
        - gravity: [0, 0, -9.81] m/s²
        - air_density: 1.225 kg/m³
        - rotor_area: 0.1 m² (área total dos rotores)
    """
```

#### Controle de Estado

```python
def set_position(self, position: np.ndarray):
    """Define posição do drone [x, y, z]."""

def set_velocity(self, velocity: np.ndarray):
    """Define velocidade do drone [vx, vy, vz]."""

def set_orientation(self, orientation: np.ndarray):
    """Define orientação como quaternion [w, x, y, z] (normalizado automaticamente)."""

def set_thrust(self, thrust: float):
    """Define empuxo normalizado [0, 1]."""

def set_target_orientation(self, orientation: np.ndarray):
    """Define orientação alvo para controle de atitude."""
```

#### Simulação Física

```python
def step(self, dt: float, wind_vector: np.ndarray = None):
    """
    Executa um passo da simulação física.

    Args:
        dt (float): Intervalo de tempo (s).
        wind_vector (np.ndarray): Vetor de vento local [vx, vy, vz].

    Integration steps:
        1. Calcula forças (empuxo, gravidade, arrasto, sustentação)
        2. Calcula torques para controle de orientação
        3. Integra aceleração linear e angular
        4. Atualiza posição e orientação
        5. Aplica amortecimento
    """
```

#### Cálculos Internos

```python
def _calculate_forces(self, wind_vector: np.ndarray) -> np.ndarray:
    """
    Calcula todas as forças atuando no drone.

    Returns:
        np.ndarray: Vetor força total [fx, fy, fz].

    Force components:
        - Thrust: thrust * max_thrust na direção local z
        - Gravity: mass * gravity
        - Drag: Proporcional ao quadrado da velocidade relativa
        - Lift: Força de sustentação aerodinâmica
    """
```

```python
def _calculate_torques(self) -> np.ndarray:
    """
    Calcula torques para controle de atitude.

    Returns:
        np.ndarray: Vetor torque [tx, ty, tz].

    Control:
        - Controle proporcional simples para orientação alvo
        - Converte erro de quaternion para torque
    """
```

#### Utilitários Quaternions

```python
def _rotate_vector_by_quaternion(self, vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    """Rotaciona vetor usando quaternion."""

def _quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiplica dois quaternions."""

def _quaternion_conjugate(self, q: np.ndarray) -> np.ndarray:
    """Retorna conjugado do quaternion."""

def _rotate_quaternion(self, axis: np.ndarray, angle: float):
    """Rotaciona quaternion ao redor de eixo por ângulo."""
```

### SwarmPhysics

Coordenação física para múltiplos drones.

#### Inicialização

```python
def __init__(self, num_drones: int = 5):
    """
    Inicializa física do enxame.

    Args:
        num_drones (int): Número de drones no enxame.

    Creates:
        - Instâncias DronePhysics para cada drone
        - Campo de vento dinâmico
        - Posições iniciais aleatórias
    """
```

#### Controle de Enxame

```python
def step(self, dt: float):
    """
    Executa passo físico para todos os drones.

    Args:
        dt (float): Intervalo de tempo.

    Process:
        - Atualiza campo de vento
        - Calcula vento local para cada drone
        - Executa step físico individual
    """
```

```python
def set_drone_thrust(self, drone_id: int, thrust: float):
    """Define empuxo para drone específico."""

def set_drone_target(self, drone_id: int, target_pos: np.ndarray):
    """Define posição alvo para controle de drone (simplificado)."""
```

#### Estado e Monitoramento

```python
def get_swarm_state(self) -> Dict[int, Dict]:
    """Retorna estado físico de todos os drones."""

def check_collisions(self) -> List[Tuple[int, int]]:
    """
    Verifica colisões entre drones.

    Returns:
        List[Tuple[int, int]]: Pares de drones em colisão.
    """
```

### WindField

Simulação de campo de vento dinâmico.

#### Inicialização

```python
def __init__(self, base_speed: float = 2.0, turbulence: float = 0.5):
    """
    Inicializa campo de vento.

    Args:
        base_speed (float): Velocidade base do vento (m/s).
        turbulence (float): Intensidade da turbulência.

    Attributes:
        - time: Tempo da simulação para variação temporal
        - grid_resolution: Resolução da grade (5m)
    """
```

#### Simulação de Vento

```python
def update(self, dt: float):
    """Atualiza estado temporal do campo de vento."""

def get_wind_at_position(self, position: np.ndarray) -> np.ndarray:
    """
    Calcula vetor de vento em posição específica.

    Args:
        position (np.ndarray): Posição [x, y, z].

    Returns:
        np.ndarray: Vetor vento [vx, vy, vz].

    Components:
        - Base wind: Variação temporal suave
        - Turbulence: Ruído baseado em posição e tempo
    """
```

---

## Simulation Module

### SwarmSimulation

Classe principal de simulação com interface WebSocket.

#### Inicialização

```python
def __init__(self, config: SwarmConfig):
    """
    Inicializa simulação do enxame.

    Args:
        config (SwarmConfig): Configuração do enxame.

    Attributes:
        - coordinator: Instância SwarmCoordinator
        - running: Flag de controle da simulação
        - simulation_thread: Thread de simulação
        - websocket_clients: Conjunto de clientes conectados
        - frame_count: Contador de frames para métricas
        - fps: Frames por segundo calculados
    """
```

#### Controle de Simulação

```python
def start_simulation(self):
    """Inicia loop de simulação em thread separada."""

def stop_simulation(self):
    """Para simulação e aguarda término da thread."""

def _simulation_loop(self):
    """
    Loop principal de simulação.

    Process:
        - Executa step do coordinator
        - Prepara estado para broadcast
        - Envia para clientes WebSocket
        - Mantém timing de 10Hz
    """
```

#### Gerenciamento de Clientes

```python
def add_websocket_client(self, client):
    """Adiciona cliente WebSocket."""

def remove_websocket_client(self, client):
    """Remove cliente WebSocket."""

def _broadcast_state(self, state: Dict):
    """Broadcast estado para todos os clientes WebSocket."""
```

#### Controle de Simulação

```python
def set_formation(self, formation: str):
    """Altera formação do enxame."""

def add_obstacle(self, x: float, y: float, z: float, radius: float):
    """Adiciona obstáculo ao ambiente."""

def reset_simulation(self):
    """Reseta simulação para estado inicial."""
```

### WebSocketServer

Servidor WebSocket para comunicação em tempo real.

#### Inicialização

```python
def __init__(self, simulation: SwarmSimulation, host: str = "localhost", port: int = 8765):
    """
    Inicializa servidor WebSocket.

    Args:
        simulation (SwarmSimulation): Instância da simulação.
        host (str): Host do servidor.
        port (int): Porta do servidor.
    """
```

#### Gerenciamento de Conexões

```python
async def handle_client(self, websocket, path: str):
    """
    Trata conexão de cliente WebSocket.

    Process:
        - Adiciona cliente à simulação
        - Processa mensagens recebidas
        - Remove cliente ao desconectar
    """
```

```python
async def _handle_message(self, websocket, data: Dict):
    """
    Processa mensagem recebida do cliente.

    Message types:
        - 'command': start, stop, reset, set_formation, add_obstacle
        - 'request': state, config
    """
```

#### Lifecycle

```python
async def start(self):
    """Inicia servidor WebSocket."""

def stop(self):
    """Para servidor WebSocket."""
```

---

## Tipos de Dados

### Configurações

```python
@dataclass
class SwarmConfig:
    """Configuração completa do enxame."""
    num_drones: int = 5
    separation_distance: float = 5.0      # Distância de separação (m)
    max_velocity: float = 2.0             # Velocidade máxima (m/s)
    communication_range: float = 20.0     # Alcance de comunicação (m)
    collision_threshold: float = 2.0      # Threshold de colisão (m)
    consensus_timeout: float = 5.0        # Timeout de consenso (s)
    heartbeat_interval: float = 1.0       # Intervalo de heartbeat (s)
    formation: Formation = Formation.CIRCLE
```

### Estados

```python
@dataclass
class DroneState:
    """Estado completo de um drone."""
    id: int
    position: np.ndarray        # [x, y, z] metros
    velocity: np.ndarray        # [vx, vy, vz] m/s
    orientation: np.ndarray     # Quaternion [w, x, y, z]
    battery_level: float        # 0.0 a 1.0
    is_active: bool = True
    last_update: float = 0.0    # Timestamp
```

### Formações

```python
class Formation(Enum):
    """Tipos de formação suportados."""
    CIRCLE = "circle"
    LINE = "line"
    GRID = "grid"
    V_FORMATION = "v_formation"
    RANDOM = "random"
```

### Estados de Consenso

```python
class ConsensusState(Enum):
    """Estados do algoritmo de consenso Raft."""
    FOLLOWER = "follower"
    CANDIDATE = "candidate"
    LEADER = "leader"
```

---

## Constantes Globais

### Física
```python
GRAVITY = np.array([0, 0, -9.81])  # Aceleração da gravidade (m/s²)
AIR_DENSITY = 1.225                 # Densidade do ar (kg/m³)
EARTH_RADIUS = 6371000              # Raio da Terra (m)
```

### Algoritmos
```python
DEFAULT_RESOLUTION = 1.0            # Resolução de pathfinding (m)
DEFAULT_SEARCH_RADIUS = 5.0         # Raio de busca RRT* (m)
MAX_ITERATIONS_RRT = 1000           # Máximo iterações RRT*
HEARTBEAT_INTERVAL = 1.0            # Intervalo de heartbeat (s)
ELECTION_TIMEOUT_RANGE = (1.0, 2.0) # Range de timeout de eleição (s)
```

### Performance
```python
SIMULATION_DT = 0.1                 # Passo de simulação (s) - 10Hz
PHYSICS_DT = 0.01                   # Passo de física (s) - 100Hz
MAX_SWARM_SIZE = 50                 # Máximo drones suportados
TARGET_FPS = 60                     # FPS alvo
```

---

## Tratamento de Erros

### Exceções Personalizadas

```python
class SwarmError(Exception):
    """Erro base para operações do enxame."""
    pass

class ConsensusError(SwarmError):
    """Erro no algoritmo de consenso."""
    pass

class PathfindingError(SwarmError):
    """Erro no planejamento de caminhos."""
    pass

class PhysicsError(SwarmError):
    """Erro na simulação física."""
    pass
```

### Logging

O sistema usa logging estruturado com níveis:

- `DEBUG`: Detalhes internos de algoritmos
- `INFO`: Eventos importantes (eleições, formações)
- `WARNING`: Condições não ideais
- `ERROR`: Falhas que afetam operação
- `CRITICAL`: Falhas críticas do sistema

---

## Extensibilidade

### Interfaces Abstratas

Para facilitar extensões futuras, considere implementar:

```python
from abc import ABC, abstractmethod

class PathfindingInterface(ABC):
    @abstractmethod
    def find_path(self, start: np.ndarray, goal: np.ndarray) -> Optional[List[np.ndarray]]:
        pass

class ConsensusInterface(ABC):
    @abstractmethod
    def step(self, current_time: float) -> Optional[Dict]:
        pass

class PhysicsInterface(ABC):
    @abstractmethod
    def step(self, dt: float):
        pass
```

### Padrões de Design

- **Strategy Pattern**: Para algoritmos intercambiáveis (pathfinding, consenso)
- **Observer Pattern**: Para notificações de eventos do enxame
- **Factory Pattern**: Para criação de componentes especializados
- **Command Pattern**: Para operações reversíveis (ex: undo de formações)

---

*Esta documentação é mantida automaticamente. Última atualização: Outubro 2025*
