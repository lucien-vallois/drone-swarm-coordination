# Drone Swarm Coordination Simulator

Maintained by [Lucien Vallois](https://github.com/lucien-vallois)

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![Three.js](https://img.shields.io/badge/Three.js-r128-orange.svg)](https://threejs.org/)
[![WebSocket](https://img.shields.io/badge/WebSocket-Real--time-green.svg)](https://developer.mozilla.org/en-US/docs/Web/API/WebSocket)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![CI](https://github.com/lucien-vallois/drone-swarm-coordination/workflows/CI/badge.svg)](https://github.com/lucien-vallois/drone-swarm-coordination/actions)
[![codecov](https://codecov.io/gh/lucien-vallois/drone-swarm-coordination/branch/main/graph/badge.svg)](https://codecov.io/gh/lucien-vallois/drone-swarm-coordination)

**Real-time 3D simulation of coordinated drone swarms** with advanced algorithms for autonomous multi-agent systems. Perfect demonstration of distributed consensus, pathfinding, and collision avoidance in robotics.

## Demo

![Drone Swarm Demo](https://raw.githubusercontent.com/lucien-vallois/drone-swarm-coordination/main/assets/demo.gif)

*10 drones autonomously coordinating in formation flight, avoiding obstacles in real-time*

**Generate your own demo GIF:**
```bash
python scripts/generate_demo.py --num-drones 10 --duration 15 --output assets/demo.gif
```

## Features

- **Consensus Algorithms**: Raft-based leader election for swarm coordination
- **Pathfinding**: A* and RRT* algorithms for 3D navigation with obstacle avoidance
- **Collision Avoidance**: Artificial potential fields for safe multi-drone operation
- **Real-time Physics**: Rigid body dynamics with aerodynamic forces and wind simulation
- **3D Visualization**: Interactive Three.js-based visualization with WebSocket streaming
- **Formation Control**: Dynamic formation changes (circle, line, grid, V-formation)
- **WebSocket API**: Real-time communication for remote control and monitoring

## Algorithms Implemented

### Consensus (Raft-inspired)
- **Leader Election**: Automatic leader selection for coordinated decisions
- **Heartbeat Protocol**: Maintain swarm synchronization
- **Fault Tolerance**: Graceful handling of drone failures

### Pathfinding
- **A* Algorithm**: Optimal pathfinding with admissible heuristics
- **RRT* Algorithm**: Sampling-based planning for complex environments
- **Formation Paths**: Coordinated path planning for entire swarm

### Collision Avoidance
- **Potential Fields**: Repulsive forces from obstacles and other drones
- **Multi-agent Coordination**: Decentralized collision-free navigation
- **Dynamic Obstacles**: Real-time obstacle addition and avoidance

##  Quick Start

Get the drone swarm running in **under 2 minutes**!

### Prerequisites
```bash
# Install dependencies
pip install -r requirements.txt

# For development (includes testing tools)
pip install -r requirements-dev.txt
```

### Launch Simulation
```bash
# Terminal 1: Start Backend Server
cd backend
python simulation.py --num-drones 8 --formation circle

# Terminal 2: Serve Frontend
cd frontend
python -m http.server 8000

# Open http://localhost:8000 in your browser
```

### Interactive Controls
- **Start/Stop**: Control simulation playback
- **Formation**: Circle → Line → Grid → V-Formation
- **Add Obstacles**: Dynamic obstacle placement
- **Reset**: Return to initial state
- **Mouse**: Orbit camera | **Scroll**: Zoom in/out

### Real-time Metrics
Monitor swarm performance with live statistics:
- Drone positions and velocities
- Leader election status
- Collision detection alerts
- FPS and frame timing

## Architecture

### Backend Components

#### Coordinator (`coordinator.py`)
- **SwarmCoordinator**: Main coordination logic
- **ConsensusAlgorithm**: Raft-based consensus implementation
- **SwarmConfig**: Configuration management

#### Pathfinding (`pathfinding.py`)
- **Pathfinding3D**: 3D pathfinding algorithms
- **FormationPathPlanner**: Multi-agent path coordination

#### Physics (`physics.py`)
- **DronePhysics**: Individual drone physics simulation
- **SwarmPhysics**: Multi-drone physics coordination
- **WindField**: Dynamic wind simulation

#### WebSocket Server (`simulation.py`)
- **SwarmSimulation**: Main simulation orchestrator
- **WebSocketServer**: Real-time communication server

### Frontend (Three.js)
- **3D Visualization**: Real-time drone positions and trajectories
- **Interactive Controls**: Formation changes and obstacle placement
- **Performance Monitoring**: FPS and swarm statistics display

## API Reference

### Complete API Documentation

Para documentação completa da API interna incluindo todas as classes, métodos, parâmetros e comportamentos, consulte:

- **[API Reference Completa](docs/api_reference.md)**: Especificações detalhadas de todos os métodos internos dos módulos principais
- **[Interfaces Modulares](backend/interfaces.py)**: Contratos abstratos para extensibilidade
- **[Exemplo de Extensão Modular](examples/modular_extension_example.py)**: Demonstração prática de como estender componentes

### WebSocket Messages

#### Commands
```json
{
  "type": "command",
  "command": "start|stop|reset|set_formation|add_obstacle",
  "params": {
    "formation": "circle|line|grid|v_formation",
    "x": 50, "y": 50, "z": 10, "radius": 3
  }
}
```

#### Requests
```json
{
  "type": "request",
  "request": "state|config"
}
```

#### Responses
```json
{
  "type": "state",
  "data": {
    "timestamp": 1234567890.123,
    "frame": 1000,
    "fps": 59.7,
    "coordinator": {...},
    "physics": {...},
    "environment": {...}
  }
}
```

## Usage Examples

### Basic Simulation
```python
from coordinator import SwarmCoordinator, SwarmConfig, Formation

# Create configuration
config = SwarmConfig(num_drones=8, formation=Formation.CIRCLE)

# Create coordinator
coordinator = SwarmCoordinator(config)

# Run simulation step
state = coordinator.step(dt=0.1)
print(f"Drones: {len(state['drones'])}, Leader: {state['leader_id']}")
```

### Pathfinding
```python
from pathfinding import Pathfinding3D

# Create pathfinder
pathfinder = Pathfinding3D(bounds=(100, 100, 50))
pathfinder.add_obstacle([50, 50, 10], 5)

# Find path
start = [0, 0, 5]
goal = [80, 80, 15]
path = pathfinder.astar(start, goal)
```

### Physics Simulation
```python
from physics import DronePhysics

# Create drone
drone = DronePhysics(mass=1.0, max_thrust=20.0)

# Set control inputs
drone.set_position([10, 10, 5])
drone.set_thrust(0.8)

# Step physics
drone.step(dt=0.1, wind_vector=[2, 0, 0])
```

## Architecture & Technologies

### Core Technologies
- **Backend**: Python 3.8+ with asyncio for high-performance simulation
- **Frontend**: Three.js WebGL for smooth 3D rendering and real-time visualization
- **Communication**: WebSocket protocol for bi-directional real-time data streaming
- **Physics**: Custom rigid-body dynamics with aerodynamic modeling
- **Algorithms**: NumPy-accelerated mathematical computations

### System Architecture

```
┌─────────────────┐    WebSocket    ┌─────────────────┐
│   Frontend      │◄──────────────►│   Backend       │
│   (Three.js)    │   Real-time     │   (Python)      │
│                 │   Streaming     │                 │
│ • 3D Rendering  │                 │ • Coordinator   │
│ • UI Controls   │                 │ • Pathfinding   │
│ • Visualization │                 │ • Physics       │
└─────────────────┘                 └─────────────────┘
                                        │
                                        ▼
                               ┌─────────────────┐
                               │  Simulation     │
                               │  Engine         │
                               │                 │
                               │ • Consensus     │
                               │ • Collision     │
                               │ • Formation     │
                               └─────────────────┘
```

### Algorithm Complexity
| Component | Time Complexity | Space Complexity | Performance |
|-----------|----------------|------------------|-------------|
| **Consensus** | O(n) | O(n) | <1s convergence |
| **Pathfinding** | O(b^d) | O(b^d) | <100ms complex |
| **Physics** | O(n) | O(n) | 60 FPS |
| **Collision** | O(n²) | O(n) | Real-time |

## Performance Benchmarks

- **Real-time**: 60 FPS simulation with WebSocket streaming
- **Scalable**: Tested with up to 50 drones simultaneously
- **Efficient**: Pathfinding completes in <100ms for complex environments
- **Low Latency**: WebSocket communication <10ms round-trip
- **Memory**: <50MB for 20-drone swarm simulation
- **CPU**: Optimized for real-time performance on standard hardware

### Advanced Performance Monitoring

O sistema inclui profiling detalhado para identificação de gargalos:

- **[Sistema de Profiling](backend/profiling.py)**: Monitoramento automático de tempo, memória e CPU
- **Métricas em Tempo Real**: Acompanhamento de performance por componente
- **Relatórios de Gargalos**: Identificação automática de funções críticas
- **Profiling Detalhado**: Integração com cProfile para análise profunda

#### Métricas Monitoradas
- Tempo de execução por função/módulo
- Uso de memória e CPU
- Latência de comunicação
- Taxa de colisões detectadas
- Eficiência de algoritmos (consenso, pathfinding, física)

#### Como Usar o Profiling
```python
from backend.profiling import get_profiler, enable_profiling

# Habilitar profiling global
enable_profiling()

# Obter relatório de performance
profiler = get_profiler()
report = profiler.get_swarm_report()
print(f"Gargalos identificados: {len(profiler.get_bottlenecks())}")
```

## Configuration

### Swarm Parameters
```python
config = SwarmConfig(
    num_drones=10,
    separation_distance=8.0,  # meters
    max_velocity=3.0,         # m/s
    communication_range=25.0, # meters
    formation=Formation.V_FORMATION
)
```

### Physics Parameters
```python
drone = DronePhysics(
    mass=1.5,           # kg
    max_thrust=25.0,    # N
    drag_coefficient=0.1
)
```

### WebSocket Server
```python
server = WebSocketServer(simulation, host="0.0.0.0", port=8765)
await server.start()
```

## Visualization

### 3D Scene Elements
- **Drones**: Green cylinders with rotating propellers
- **Targets**: Red spheres showing formation positions
- **Obstacles**: Orange transparent spheres
- **Paths**: Blue lines showing planned trajectories
- **Ground**: Textured plane with grid overlay

### Interactive Controls
- **Mouse**: Orbit camera around scene
- **Scroll**: Zoom in/out
- **Formation Buttons**: Change swarm formation
- **Obstacle Placement**: Add obstacles via coordinate input

## Real-world Application

This simulator demonstrates techniques used in **cutting-edge autonomous systems research**:

### UAV Swarm Coordination Systems (2024)
- **Military Applications**: Coordinated drone swarms for reconnaissance and combat operations
- **Commercial Operations**: Delivery networks, agricultural monitoring, and infrastructure inspection
- **Search & Rescue**: Autonomous area coverage and target localization

### Multi-agent Autonomous Systems
- **Robotics Coordination**: Industrial robot collaboration in manufacturing
- **Autonomous Vehicles**: Platooning and cooperative driving systems
- **Space Exploration**: Satellite constellations and rover coordination

### Advanced Research Areas
- **GPS-denied Navigation**: Vision-based navigation and SLAM integration
- **Real-time Path Planning**: Dynamic obstacle avoidance in complex environments
- **Consensus Protocols**: Fault-tolerant distributed decision-making
- **Swarm Intelligence**: Emergent behavior from simple local rules

### Industry Integration
- **ROS Compatibility**: Robot Operating System integration for real hardware
- **Hardware-in-the-Loop**: Testing with actual drone hardware
- **Simulation-to-Reality**: Transfer learning from simulation to physical drones

## Extending the Simulator

### Modular Architecture

O sistema agora suporta extensibilidade através de interfaces modulares:

#### Interfaces Abstratas
- **IConsensusAlgorithm**: Para algoritmos de consenso (Raft, Paxos, customizados)
- **IPathfindingAlgorithm**: Para algoritmos de pathfinding (A*, RRT*, customizados)
- **ICollisionAvoidance**: Para avoidance de colisão (campos de potencial, velocity obstacles)
- **IPhysicsEngine**: Para motores de física (corpos rígidos, customizados)
- **IFormationPlanner**: Para planejadores de formação
- **ICommunicationManager**: Para gerenciadores de comunicação

#### Como Criar Componentes Customizados

1. **Implemente a interface apropriada**:
```python
from backend.interfaces import IPathfindingAlgorithm

class MyCustomPathfinder(IPathfindingAlgorithm):
    def find_path(self, start, goal, **kwargs):
        # Sua implementação customizada
        return path
```

2. **Use a fábrica customizada**:
```python
from backend.factories import create_custom_factory

factory = create_custom_factory(
    pathfinding_algorithm=MyCustomPathfinder,
    consensus_algorithm=MyCustomConsensus
)
```

3. **Integre no sistema**:
```python
coordinator = ModularSwarmCoordinator(config, factory)
```

### Adding New Algorithms (Legacy)
1. Implement algorithm in appropriate module
2. Add configuration options to `SwarmConfig`
3. Integrate with coordinator's step function
4. Update WebSocket API if needed

### Custom Formations
```python
def custom_formation(drone_id, num_drones, center, radius):
    angle = 2 * math.pi * drone_id / num_drones
    x = center[0] + radius * math.cos(angle + math.pi/4)  # 45-degree offset
    y = center[1] + radius * math.sin(angle + math.pi/4)
    z = center[2] + drone_id * 2  # Height variation
    return [x, y, z]
```

### Integration with Real Hardware
```python
# ROS integration example
import rclpy
from geometry_msgs.msg import PoseStamped

def publish_drone_pose(drone_id, position, orientation):
    msg = PoseStamped()
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.pose.position.x = position[0]
    msg.pose.position.y = position[1]
    msg.pose.position.z = position[2]
    # Publish to ROS topic
```

## Testing

### Test Suite Completa

Este projeto inclui uma suíte abrangente de testes cobrindo:

- **Testes Unitários**: Algoritmos críticos (consensus, pathfinding, physics)
- **Testes de Integração**: Coordenação entre módulos
- **Benchmarks de Performance**: Análise de tempo real e escalabilidade
- **Testes de Tolerância a Falhas**: Recuperação de falhas

### Executando os Testes

#### Todos os Testes
```bash
# Instalar dependências de desenvolvimento
pip install -r requirements-dev.txt

# Executar toda a suíte
python tests/run_tests.py

# Ou diretamente com pytest
pytest
```

#### Testes Específicos
```bash
# Apenas testes unitários
python tests/run_tests.py unit

# Apenas testes de integração
python tests/run_tests.py integration

# Apenas algoritmos de consenso
python tests/run_tests.py consensus

# Com relatório de cobertura
python tests/run_tests.py --coverage
```

#### Benchmarks de Performance
```bash
# Executar benchmarks
python tests/run_tests.py --benchmarks

# Exemplo de resultado esperado:
# Coordinator step: 3.45ms (target: <10ms)
# Pathfinding A*: 12.34ms (target: <100ms)
# Consensus convergence: 0.89s (target: <1s)
```

### Cobertura de Código

- **Objetivo**: ≥80% cobertura geral
- **Módulos Críticos**: ≥95% coverage (consensus, pathfinding, physics)
- **Relatório HTML**: `pytest --cov=backend --cov-report=html`

### Estrutura dos Testes

```
tests/
├── test_consensus.py         # Algoritmo Raft-based
├── test_pathfinding.py       # A* e RRT* 3D
├── test_physics.py           # Simulação física
├── test_collision_avoidance.py # Formações e avoidance
├── test_integration.py       # Integração e performance
├── conftest.py              # Fixtures compartilhadas
└── run_tests.py             # Script de execução
```

### Configuração de Desenvolvimento

Para desenvolvimento com testes:

```bash
# Instalar dependências
pip install -r requirements.txt
pip install -r requirements-dev.txt

# Executar testes antes de commit
python tests/run_tests.py --coverage

# Verificar cobertura mínima
python tests/run_tests.py --check-coverage
```

## Troubleshooting

### Common Issues

**WebSocket Connection Failed**
- Check if backend server is running on correct port
- Verify firewall settings allow WebSocket connections
- Check browser console for connection errors

**Simulation Running Slowly**
- Reduce number of drones or increase step time
- Disable complex pathfinding for large swarms
- Check system resources (CPU, memory)

**Pathfinding Fails**
- Add obstacle clearance to pathfinding bounds
- Increase RRT* iterations for complex environments
- Check coordinate system consistency

### Debug Mode
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## Future Enhancements

- **Machine Learning Integration**: RL-based swarm control
- **Hardware-in-the-Loop**: Real drone integration
- **Multi-swarm Coordination**: Multiple independent swarms
- **Terrain Following**: 3D terrain-aware pathfinding
- **Communication Models**: Realistic wireless communication simulation
- **Energy Management**: Battery-aware path planning

## Contributing

We welcome contributions! This project is perfect for:

- **Students**: Learn multi-agent systems, robotics algorithms, and real-time simulation
- **Researchers**: Extend algorithms for academic publications
- **Developers**: Add new features like machine learning integration or hardware interfaces

**Please read our [Contributing Guide](CONTRIBUTING.md) for detailed information on:**
- Development setup and environment configuration
- Code standards and formatting guidelines
- Testing requirements and coverage goals
- Pull request process and commit conventions

### Quick Start for Contributors
```bash
git clone https://github.com/lucien-vallois/drone-swarm-coordination.git
cd drone-swarm-coordination
pip install -r requirements.txt
pip install -r requirements-dev.txt
python backend/simulation.py --num-drones 5
```

## Documentation

- **[Algorithms Guide](docs/algorithms.md)**: Detailed technical documentation
- **[Examples](examples/)**: Code samples and demonstrations
- **[API Reference](README.md#api-reference)**: WebSocket and Python API docs
- **[Contributing Guide](CONTRIBUTING.md)**: How to contribute to this project
- **[Changelog](CHANGELOG.md)**: History of changes and releases

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Featured Project

**This project demonstrates expertise in:**
- **Multi-agent Systems & Swarm Intelligence**
- **Robotics & Autonomous Systems**
- **Real-time Web Applications**
- **Algorithm Design & Optimization**
- **3D Visualization & Interactive Graphics**

*Perfect showcase for portfolios in Robotics, AI, and Computer Graphics!*
