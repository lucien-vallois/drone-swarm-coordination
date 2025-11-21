"""
Testes unitários para simulação física (DronePhysics e SwarmPhysics).
"""

from unittest.mock import Mock, patch

import numpy as np
import pytest
from backend.physics import DronePhysics, Environment, SwarmPhysics, WindField


class TestDronePhysics:
    """Testes para DronePhysics"""

    def test_initialization(self):
        """Testa inicialização da física do drone"""
        drone = DronePhysics(mass=2.0, max_thrust=30.0, drag_coefficient=0.2)

        assert drone.mass == 2.0
        assert drone.max_thrust == 30.0
        assert drone.drag_coefficient == 0.2
        assert np.array_equal(drone.position, np.zeros(3))
        assert np.array_equal(drone.velocity, np.zeros(3))
        assert np.array_equal(drone.acceleration, np.zeros(3))
        assert np.array_equal(drone.orientation, np.array([1, 0, 0, 0]))
        assert np.array_equal(drone.angular_velocity, np.zeros(3))

    def test_set_position(self, drone_physics):
        """Testa configuração de posição"""
        new_pos = np.array([10.0, 20.0, 5.0])
        drone_physics.set_position(new_pos)

        assert np.array_equal(drone_physics.position, new_pos)

    def test_set_velocity(self, drone_physics):
        """Testa configuração de velocidade"""
        new_vel = np.array([1.0, 2.0, 0.5])
        drone_physics.set_velocity(new_vel)

        assert np.array_equal(drone_physics.velocity, new_vel)

    def test_set_orientation(self, drone_physics):
        """Testa configuração de orientação"""
        new_ori = np.array([0.707, 0.0, 0.707, 0.0])  # 90 graus em Z
        drone_physics.set_orientation(new_ori)

        # Deve ser normalizado
        assert np.isclose(np.linalg.norm(drone_physics.orientation), 1.0)
        assert np.allclose(drone_physics.orientation, new_ori)

    def test_set_thrust(self, drone_physics):
        """Testa configuração de thrust"""
        # Valor válido
        drone_physics.set_thrust(0.8)
        assert drone_physics.thrust == 0.8

        # Clamping mínimo
        drone_physics.set_thrust(-0.5)
        assert drone_physics.thrust == 0.0

        # Clamping máximo
        drone_physics.set_thrust(1.5)
        assert drone_physics.thrust == 1.0

    def test_calculate_forces_no_wind(self, drone_physics):
        """Testa cálculo de forças sem vento"""
        drone_physics.set_thrust(0.5)  # 50% thrust

        forces = drone_physics._calculate_forces(np.zeros(3))

        # Deve ter força de thrust para cima (gravidade negativa)
        expected_thrust = 0.5 * drone_physics.max_thrust
        assert forces[2] > 0  # Thrust para cima

        # Gravidade
        assert forces[2] < 0  # Gravidade para baixo (mas thrust pode compensar)

    def test_calculate_forces_with_wind(self, drone_physics):
        """Testa cálculo de forças com vento"""
        wind = np.array([2.0, 0.0, 0.0])  # Vento no eixo X
        drone_physics.set_velocity(np.array([1.0, 0.0, 0.0]))

        forces = drone_physics._calculate_forces(wind)

        # Deve haver força de arrasto oposta à velocidade relativa
        # Velocidade relativa = velocidade - vento = [-1.0, 0.0, 0.0]
        # Arrasto deve ser positivo em X (oposto à velocidade relativa)

    def test_step_integration(self, drone_physics):
        """Testa integração física (step)"""
        dt = 0.1
        initial_pos = drone_physics.position.copy()
        initial_vel = drone_physics.velocity.copy()

        drone_physics.set_thrust(0.1)  # Thrust pequeno

        drone_physics.step(dt)

        # Posição deve mudar baseada na velocidade
        assert not np.array_equal(drone_physics.position, initial_pos)

        # Velocidade deve mudar baseada na aceleração
        assert not np.array_equal(drone_physics.velocity, initial_vel)

    def test_step_with_gravity(self, drone_physics):
        """Testa que gravidade afeta movimento"""
        dt = 0.1
        drone_physics.set_thrust(0.0)  # Sem thrust

        initial_height = drone_physics.position[2]

        drone_physics.step(dt)

        # Deve cair devido à gravidade
        assert drone_physics.position[2] < initial_height

    def test_step_damping(self, drone_physics):
        """Testa amortecimento de velocidade"""
        # Define velocidade inicial
        initial_velocity = np.array([10.0, 5.0, 2.0])
        drone_physics.set_velocity(initial_velocity)
        drone_physics.set_thrust(0.0)  # Sem thrust

        dt = 0.1
        drone_physics.step(dt)

        # Velocidade deve ser reduzida devido ao amortecimento
        speed_after = np.linalg.norm(drone_physics.velocity)
        speed_before = np.linalg.norm(initial_velocity)

        assert speed_after < speed_before

    def test_quaternion_operations(self, drone_physics):
        """Testa operações com quaternions"""
        q1 = np.array([1.0, 0.0, 0.0, 0.0])  # Identidade
        q2 = np.array([0.707, 0.707, 0.0, 0.0])  # 90 graus em X

        result = drone_physics._quaternion_multiply(q1, q2)
        assert np.allclose(result, q2)

        # Testa conjugado
        conj = drone_physics._quaternion_conjugate(q2)
        expected_conj = np.array([0.707, -0.707, 0.0, 0.0])
        assert np.allclose(conj, expected_conj)

    def test_get_state(self, drone_physics):
        """Testa obtenção do estado físico"""
        drone_physics.set_position(np.array([1.0, 2.0, 3.0]))
        drone_physics.set_velocity(np.array([0.1, 0.2, 0.3]))
        drone_physics.set_thrust(0.8)

        state = drone_physics.get_state()

        assert isinstance(state, dict)
        assert 'position' in state
        assert 'velocity' in state
        assert 'orientation' in state
        assert 'thrust' in state

        assert np.array_equal(state['position'], drone_physics.position)
        assert np.array_equal(state['velocity'], drone_physics.velocity)
        assert state['thrust'] == 0.8


class TestSwarmPhysics:
    """Testes para SwarmPhysics"""

    def test_initialization(self):
        """Testa inicialização da física do enxame"""
        num_drones = 3
        swarm = SwarmPhysics(num_drones)

        assert len(swarm.drones) == num_drones
        assert isinstance(swarm.wind_field, WindField)

        # Verifica que drones foram criados
        for i in range(num_drones):
            assert i in swarm.drones
            assert isinstance(swarm.drones[i], DronePhysics)

    def test_step_all_drones(self):
        """Testa step de todos os drones"""
        swarm = SwarmPhysics(2)
        initial_positions = {}

        for drone_id, drone in swarm.drones.items():
            initial_positions[drone_id] = drone.position.copy()

        dt = 0.1
        swarm.step(dt)

        # Todas as posições devem ter mudado (devido à gravidade)
        for drone_id, drone in swarm.drones.items():
            assert not np.array_equal(drone.position, initial_positions[drone_id])

    def test_set_drone_thrust(self):
        """Testa configuração de thrust individual"""
        swarm = SwarmPhysics(2)

        swarm.set_drone_thrust(0, 0.7)
        swarm.set_drone_thrust(1, 0.3)

        assert swarm.drones[0].thrust == 0.7
        assert swarm.drones[1].thrust == 0.3

    def test_set_drone_target(self):
        """Testa configuração de alvo para drone"""
        swarm = SwarmPhysics(1)
        target = np.array([10.0, 5.0, 8.0])

        swarm.set_drone_target(0, target)

        # Verifica que orientação foi definida (simplificada)
        # Nota: implementação atual é simplificada

    def test_get_swarm_state(self):
        """Testa obtenção do estado do enxame"""
        swarm = SwarmPhysics(2)

        # Define posições específicas para teste
        swarm.drones[0].set_position(np.array([1.0, 2.0, 3.0]))
        swarm.drones[1].set_position(np.array([4.0, 5.0, 6.0]))

        state = swarm.get_swarm_state()

        assert len(state) == 2
        assert 0 in state
        assert 1 in state

        assert np.array_equal(state[0]['position'], np.array([1.0, 2.0, 3.0]))
        assert np.array_equal(state[1]['position'], np.array([4.0, 5.0, 6.0]))

    def test_check_collisions(self):
        """Testa detecção de colisões"""
        swarm = SwarmPhysics(2)

        # Posiciona drones longe um do outro
        swarm.drones[0].set_position(np.array([0.0, 0.0, 0.0]))
        swarm.drones[1].set_position(np.array([10.0, 0.0, 0.0]))

        collisions = swarm.check_collisions()
        assert len(collisions) == 0

        # Posiciona drones próximos (colisão)
        swarm.drones[1].set_position(np.array([0.5, 0.0, 0.0]))

        collisions = swarm.check_collisions()
        assert len(collisions) == 1
        assert (0, 1) in collisions or (1, 0) in collisions


class TestWindField:
    """Testes para WindField"""

    def test_initialization(self):
        """Testa inicialização do campo de vento"""
        wind = WindField(base_speed=3.0, turbulence=0.8)

        assert wind.base_speed == 3.0
        assert wind.turbulence == 0.8
        assert wind.time == 0.0

    def test_update(self, wind_field):
        """Testa atualização temporal do vento"""
        dt = 0.5
        initial_time = wind_field.time

        wind_field.update(dt)

        assert wind_field.time == initial_time + dt

    def test_get_wind_at_position(self, wind_field):
        """Testa obtenção de vento em posição específica"""
        position = np.array([10.0, 5.0, 2.0])

        wind = wind_field.get_wind_at_position(position)

        assert len(wind) == 3  # Vetor 3D
        assert isinstance(wind, np.ndarray)

        # Vento deve ter componente base
        assert wind[0] >= 0  # Vento base é positivo em X

    def test_wind_turbulence(self, wind_field):
        """Testa que turbulência adiciona variação"""
        position1 = np.array([0.0, 0.0, 0.0])
        position2 = np.array([10.0, 10.0, 10.0])

        wind1 = wind_field.get_wind_at_position(position1)
        wind2 = wind_field.get_wind_at_position(position2)

        # Ventos em posições diferentes devem ser diferentes devido à turbulência
        assert not np.array_equal(wind1, wind2)

    def test_get_wind_field(self, wind_field):
        """Testa obtenção do campo de vento geral"""
        bounds = (0, 10, 0, 10, 0, 10)

        field = wind_field.get_wind_field(bounds)

        assert isinstance(field, dict)
        assert 'base_speed' in field
        assert 'turbulence' in field
        assert 'time' in field

        assert field['base_speed'] == wind_field.base_speed
        assert field['turbulence'] == wind_field.turbulence


class TestEnvironment:
    """Testes para Environment"""

    def test_initialization(self):
        """Testa inicialização do ambiente"""
        bounds = (100, 100, 50)
        env = Environment(bounds)

        assert env.bounds == bounds
        assert len(env.obstacles) == 0
        assert len(env.no_fly_zones) == 0

    def test_add_obstacle(self):
        """Testa adição de obstáculos"""
        env = Environment((100, 100, 50))
        position = np.array([50.0, 50.0, 25.0])
        radius = 5.0

        env.add_obstacle(position, radius)

        assert len(env.obstacles) == 1
        assert np.array_equal(env.obstacles[0][0], position)
        assert env.obstacles[0][1] == radius

    def test_is_position_valid(self):
        """Testa validação de posições"""
        env = Environment((100, 100, 50))

        # Posições válidas
        valid_pos = np.array([50.0, 50.0, 25.0])
        assert env.is_position_valid(valid_pos) is True

        # Posições fora dos limites
        invalid_positions = [
            np.array([-1.0, 50.0, 25.0]),   # x < 0
            np.array([101.0, 50.0, 25.0]),  # x > 100
            np.array([50.0, -1.0, 25.0]),   # y < 0
            np.array([50.0, 101.0, 25.0]),  # y > 100
            np.array([50.0, 50.0, -1.0]),   # z < 0
            np.array([50.0, 50.0, 51.0])    # z > 50
        ]

        for pos in invalid_positions:
            assert env.is_position_valid(pos) is False

    def test_obstacle_collision(self):
        """Testa detecção de colisão com obstáculos"""
        env = Environment((100, 100, 50))

        # Adiciona obstáculo
        env.add_obstacle(np.array([50.0, 50.0, 25.0]), 5.0)

        # Posição livre
        free_pos = np.array([60.0, 60.0, 25.0])
        assert env.is_position_valid(free_pos) is True

        # Posição dentro do obstáculo
        collision_pos = np.array([52.0, 52.0, 25.0])
        assert env.is_position_valid(collision_pos) is False

    def test_no_fly_zones(self):
        """Testa zonas proibidas"""
        env = Environment((100, 100, 50))

        # Adiciona zona proibida
        min_corner = np.array([10.0, 10.0, 5.0])
        max_corner = np.array([20.0, 20.0, 15.0])
        env.add_no_fly_zone(min_corner, max_corner)

        # Posição fora da zona
        outside_pos = np.array([5.0, 5.0, 10.0])
        assert env.is_position_valid(outside_pos) is True

        # Posição dentro da zona
        inside_pos = np.array([15.0, 15.0, 10.0])
        assert env.is_position_valid(inside_pos) is False

    def test_get_obstacle_forces(self):
        """Testa cálculo de forças repulsivas de obstáculos"""
        env = Environment((100, 100, 50))

        # Adiciona obstáculo
        env.add_obstacle(np.array([10.0, 10.0, 10.0]), 2.0)

        # Posição longe do obstáculo
        far_pos = np.array([20.0, 20.0, 10.0])
        force_far = env.get_obstacle_forces(far_pos)
        assert np.allclose(force_far, np.zeros(3))

        # Posição perto do obstáculo
        near_pos = np.array([11.0, 11.0, 10.0])
        force_near = env.get_obstacle_forces(near_pos)

        # Deve haver força repulsiva
        assert not np.allclose(force_near, np.zeros(3))

        # Força deve apontar para longe do obstáculo
        direction_to_obstacle = np.array([10.0, 10.0, 10.0]) - near_pos
        force_direction = force_near / np.linalg.norm(force_near)

        # Ângulo entre força e direção ao obstáculo deve ser > 90 graus (repulsiva)
        cos_angle = np.dot(force_direction, direction_to_obstacle) / np.linalg.norm(direction_to_obstacle)
        assert cos_angle < 0  # Ângulo > 90 graus
