"""
Testes unitários para algoritmos de pathfinding (A* e RRT*).
"""

from unittest.mock import Mock, patch

import numpy as np
import pytest
from backend.pathfinding import FormationPathPlanner, Pathfinding3D


class TestPathfinding3D:
    """Testes para Pathfinding3D"""

    def test_initialization(self):
        """Testa inicialização do pathfinder"""
        bounds = (100, 100, 50)
        pathfinder = Pathfinding3D(bounds)

        assert pathfinder.bounds == bounds
        assert len(pathfinder.obstacles) == 0

    def test_add_obstacle(self, pathfinder_3d):
        """Testa adição de obstáculos"""
        position = np.array([10.0, 20.0, 5.0])
        radius = 3.0

        pathfinder_3d.add_obstacle(position, radius)

        assert len(pathfinder_3d.obstacles) == 1
        assert np.array_equal(pathfinder_3d.obstacles[0][0], position)
        assert pathfinder_3d.obstacles[0][1] == radius

    def test_clear_obstacles(self, pathfinder_3d):
        """Testa limpeza de obstáculos"""
        pathfinder_3d.add_obstacle(np.array([10, 10, 10]), 2.0)
        pathfinder_3d.add_obstacle(np.array([20, 20, 20]), 3.0)

        assert len(pathfinder_3d.obstacles) == 2

        pathfinder_3d.clear_obstacles()
        assert len(pathfinder_3d.obstacles) == 0

    def test_is_valid_position_bounds(self, pathfinder_3d):
        """Testa validação de posição dentro dos limites"""
        # Posição válida
        valid_pos = np.array([50.0, 50.0, 25.0])
        assert pathfinder_3d.is_valid_position(valid_pos) is True

        # Posições inválidas
        invalid_positions = [
            np.array([-1.0, 50.0, 25.0]),  # x negativo
            np.array([150.0, 50.0, 25.0]),  # x além do limite
            np.array([50.0, -1.0, 25.0]),  # y negativo
            np.array([50.0, 150.0, 25.0]),  # y além do limite
            np.array([50.0, 50.0, -1.0]),  # z negativo
            np.array([50.0, 50.0, 60.0])   # z além do limite
        ]

        for pos in invalid_positions:
            assert pathfinder_3d.is_valid_position(pos) is False

    def test_is_valid_position_obstacles(self, pathfinder_3d):
        """Testa validação de posição com obstáculos"""
        # Adiciona obstáculo
        obstacle_pos = np.array([50.0, 50.0, 25.0])
        pathfinder_3d.add_obstacle(obstacle_pos, 5.0)

        # Posição livre
        free_pos = np.array([60.0, 60.0, 25.0])
        assert pathfinder_3d.is_valid_position(free_pos) is True

        # Posição dentro do obstáculo
        inside_obstacle = np.array([52.0, 52.0, 25.0])
        assert pathfinder_3d.is_valid_position(inside_obstacle) is False

    def test_get_neighbors(self, pathfinder_3d):
        """Testa obtenção de vizinhos válidos"""
        position = np.array([50.0, 50.0, 25.0])

        neighbors = pathfinder_3d.get_neighbors(position)

        # Deve ter 26 vizinhos (3x3x3 - 1 centro)
        assert len(neighbors) == 26

        # Todos os vizinhos devem ser válidos
        for neighbor in neighbors:
            assert pathfinder_3d.is_valid_position(neighbor)

        # Vizinhos devem estar a 1 unidade de distância
        for neighbor in neighbors:
            distance = np.linalg.norm(neighbor - position)
            assert abs(distance - 1.0) < 1e-6

    def test_heuristic(self, pathfinder_3d):
        """Testa função heurística (distância Euclidiana)"""
        a = np.array([0.0, 0.0, 0.0])
        b = np.array([3.0, 4.0, 0.0])  # Triângulo retângulo

        heuristic = pathfinder_3d.heuristic(a, b)
        expected = 5.0  # sqrt(3² + 4² + 0²)

        assert abs(heuristic - expected) < 1e-6

    def test_astar_simple_path(self, pathfinder_3d):
        """Testa A* com caminho simples"""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([5.0, 0.0, 0.0])

        path = pathfinder_3d.astar(start, goal)

        assert path is not None
        assert len(path) > 0
        assert np.allclose(path[0], start)
        assert np.allclose(path[-1], goal)

        # Caminho deve ser monotonicamente crescente em x
        for i in range(1, len(path)):
            assert path[i][0] >= path[i-1][0]

    def test_astar_with_obstacle(self, pathfinder_3d):
        """Testa A* com obstáculo bloqueando caminho direto"""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([10.0, 0.0, 0.0])

        # Adiciona obstáculo no caminho direto
        pathfinder_3d.add_obstacle(np.array([5.0, 0.0, 0.0]), 2.0)

        path = pathfinder_3d.astar(start, goal)

        assert path is not None
        assert len(path) > 0
        assert np.allclose(path[0], start)
        assert np.allclose(path[-1], goal, atol=1.0)  # Tolerância maior para pathfinding

        # Verifica que o caminho evita o obstáculo
        for waypoint in path:
            distance_to_obstacle = np.linalg.norm(waypoint - np.array([5.0, 0.0, 0.0]))
            assert distance_to_obstacle > 2.0

    def test_astar_impossible_path(self, pathfinder_3d):
        """Testa A* quando não há caminho possível"""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([10.0, 0.0, 0.0])

        # Cerca completamente o goal
        obstacles = [
            (np.array([10.0, 0.0, 0.0]), 1.0),
            (np.array([9.0, 0.0, 0.0]), 1.0),
            (np.array([11.0, 0.0, 0.0]), 1.0),
            (np.array([10.0, 1.0, 0.0]), 1.0),
            (np.array([10.0, -1.0, 0.0]), 1.0),
        ]

        for pos, radius in obstacles:
            pathfinder_3d.add_obstacle(pos, radius)

        path = pathfinder_3d.astar(start, goal)

        assert path is None

    def test_rrt_star_simple_path(self, pathfinder_3d):
        """Testa RRT* com caminho simples"""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([10.0, 10.0, 5.0])

        path = pathfinder_3d.rrt_star(start, goal, max_iterations=500)

        assert path is not None
        assert len(path) > 0
        assert np.allclose(path[0], start)
        assert np.allclose(path[-1], goal, atol=2.0)

    def test_rrt_star_with_obstacles(self, pathfinder_3d):
        """Testa RRT* com obstáculos"""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([20.0, 0.0, 0.0])

        # Adiciona obstáculos
        pathfinder_3d.add_obstacle(np.array([5.0, 0.0, 0.0]), 2.0)
        pathfinder_3d.add_obstacle(np.array([15.0, 0.0, 0.0]), 2.0)

        path = pathfinder_3d.rrt_star(start, goal, max_iterations=1000)

        assert path is not None
        assert len(path) > 0
        assert np.allclose(path[0], start)
        assert np.allclose(path[-1], goal, atol=3.0)

        # Verifica que o caminho evita obstáculos
        for waypoint in path:
            for obs_pos, obs_radius in pathfinder_3d.obstacles:
                distance = np.linalg.norm(waypoint - obs_pos)
                assert distance > obs_radius

    def test_smooth_path(self, pathfinder_3d):
        """Testa suavização de caminho"""
        # Caminho com waypoints desnecessários
        path = [
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
            np.array([2.0, 0.0, 0.0]),
            np.array([3.0, 0.0, 0.0]),
            np.array([4.0, 1.0, 0.0]),
            np.array([5.0, 1.0, 0.0]),
            np.array([6.0, 1.0, 0.0]),
            np.array([7.0, 1.0, 0.0]),
            np.array([8.0, 1.0, 0.0]),
            np.array([9.0, 1.0, 0.0]),
            np.array([10.0, 1.0, 0.0])
        ]

        smoothed = pathfinder_3d.smooth_path(path)

        # Caminho suavizado deve ser mais curto ou igual
        assert len(smoothed) <= len(path)

        # Deve manter start e goal
        assert np.allclose(smoothed[0], path[0])
        assert np.allclose(smoothed[-1], path[-1])

        # Todos os pontos devem ser válidos
        for point in smoothed:
            assert pathfinder_3d.is_valid_position(point)

    def test_collision_free(self, pathfinder_3d):
        """Testa verificação de colisão livre"""
        # Caminho sem obstáculos
        a = np.array([0.0, 0.0, 0.0])
        b = np.array([10.0, 0.0, 0.0])

        assert pathfinder_3d.collision_free(a, b) is True

        # Adiciona obstáculo
        pathfinder_3d.add_obstacle(np.array([5.0, 0.0, 0.0]), 1.0)

        # Agora deve detectar colisão
        assert pathfinder_3d.collision_free(a, b) is False

    def test_find_path_algorithm_selection(self, pathfinder_3d):
        """Testa seleção de algoritmo em find_path"""
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([5.0, 0.0, 0.0])

        # Testa A*
        path_astar = pathfinder_3d.find_path(start, goal, "astar")
        assert path_astar is not None

        # Testa RRT*
        path_rrt = pathfinder_3d.find_path(start, goal, "rrt_star")
        assert path_rrt is not None

        # Testa algoritmo inválido
        with pytest.raises(ValueError):
            pathfinder_3d.find_path(start, goal, "invalid")


class TestFormationPathPlanner:
    """Testes para FormationPathPlanner"""

    def test_initialization(self, pathfinder_3d):
        """Testa inicialização do planejador de formações"""
        planner = FormationPathPlanner(pathfinder_3d)

        assert planner.pathfinder == pathfinder_3d

    def test_plan_formation_paths(self, pathfinder_3d):
        """Testa planejamento de caminhos para formação"""
        planner = FormationPathPlanner(pathfinder_3d)

        # Posições atuais e desejadas
        current_positions = {
            0: np.array([0.0, 0.0, 0.0]),
            1: np.array([10.0, 0.0, 0.0])
        }

        target_positions = {
            0: np.array([20.0, 20.0, 10.0]),
            1: np.array([30.0, 20.0, 10.0])
        }

        paths = planner.plan_formation_paths(current_positions, target_positions)

        assert len(paths) == 2
        assert 0 in paths
        assert 1 in paths

        # Cada caminho deve começar na posição atual e terminar na alvo
        for drone_id in paths:
            path = paths[drone_id]
            assert np.allclose(path[0], current_positions[drone_id])
            assert np.allclose(path[-1], target_positions[drone_id], atol=1.0)

    def test_optimize_formation_paths(self, pathfinder_3d):
        """Testa otimização de caminhos de formação"""
        planner = FormationPathPlanner(pathfinder_3d)

        # Caminhos iniciais com conflitos
        paths = {
            0: [np.array([0.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0]), np.array([10.0, 0.0, 0.0])],
            1: [np.array([10.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])]
        }

        optimized = planner.optimize_formation_paths(paths)

        # Deve manter start e goal
        assert np.allclose(optimized[0][0], paths[0][0])
        assert np.allclose(optimized[0][-1], paths[0][-1])
        assert np.allclose(optimized[1][0], paths[1][0])
        assert np.allclose(optimized[1][-1], paths[1][-1])

        # Caminhos otimizados devem ser válidos
        for drone_paths in optimized.values():
            for point in drone_paths:
                assert pathfinder_3d.is_valid_position(point)
