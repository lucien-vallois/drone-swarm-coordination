"""
Testes unitários para o sistema de profiling.
"""

import time
from unittest.mock import Mock, patch

import numpy as np
import pytest
from backend.profiling import (
    PerformanceMetrics,
    Profiler,
    SwarmProfiler,
    disable_profiling,
    enable_profiling,
    get_profiler,
)


class TestPerformanceMetrics:
    """Testes para PerformanceMetrics"""

    def test_initialization(self):
        """Testa inicialização das métricas"""
        metrics = PerformanceMetrics()

        assert metrics.total_time == 0.0
        assert metrics.call_count == 0
        assert metrics.average_time == 0.0
        assert metrics.min_time == float('inf')
        assert metrics.max_time == 0.0
        assert metrics.peak_memory_mb == 0.0

    def test_update_timing(self):
        """Testa atualização de métricas de timing"""
        metrics = PerformanceMetrics()

        metrics.update_timing(0.1)
        assert metrics.total_time == 0.1
        assert metrics.call_count == 1
        assert metrics.average_time == 0.1
        assert metrics.min_time == 0.1
        assert metrics.max_time == 0.1

        metrics.update_timing(0.2)
        assert metrics.total_time == 0.3
        assert metrics.call_count == 2
        assert metrics.average_time == 0.15
        assert metrics.min_time == 0.1
        assert metrics.max_time == 0.2

    def test_update_memory(self):
        """Testa atualização de métricas de memória"""
        metrics = PerformanceMetrics()

        metrics.update_memory(100.0)
        assert metrics.peak_memory_mb == 100.0
        assert len(metrics.memory_samples) == 1

        metrics.update_memory(150.0)
        assert metrics.peak_memory_mb == 150.0
        assert metrics.average_memory_mb == 125.0

    def test_update_cpu(self):
        """Testa atualização de métricas de CPU"""
        metrics = PerformanceMetrics()

        metrics.update_cpu(50.0)
        assert len(metrics.cpu_samples) == 1

        metrics.update_cpu(75.0)
        assert len(metrics.cpu_samples) == 2
        assert metrics.cpu_percent > 0

    def test_to_dict(self):
        """Testa conversão para dicionário"""
        metrics = PerformanceMetrics()
        metrics.update_timing(0.1)
        metrics.update_memory(100.0)

        result = metrics.to_dict()

        assert isinstance(result, dict)
        assert 'total_time' in result
        assert 'call_count' in result
        assert 'average_time' in result
        assert 'peak_memory_mb' in result


class TestProfiler:
    """Testes para Profiler"""

    def test_initialization(self):
        """Testa inicialização do profiler"""
        profiler = Profiler(enabled=True)

        assert profiler.enabled is True
        assert len(profiler.metrics) >= 0

    def test_timer_context_manager(self):
        """Testa uso do timer como context manager"""
        profiler = Profiler(enabled=True)

        with profiler.timer('test_function'):
            time.sleep(0.01)  # Pequeno delay

        assert 'test_function' in profiler.metrics
        assert profiler.metrics['test_function'].call_count == 1
        assert profiler.metrics['test_function'].total_time > 0

    def test_timer_disabled(self):
        """Testa que timer não mede quando desabilitado"""
        profiler = Profiler(enabled=False)

        with profiler.timer('test_function'):
            time.sleep(0.01)

        assert 'test_function' not in profiler.metrics

    def test_start_stop_timer(self):
        """Testa uso manual de timer"""
        profiler = Profiler(enabled=True)

        profiler.start_timer('manual_timer')
        time.sleep(0.01)
        elapsed = profiler.stop_timer('manual_timer')

        assert elapsed > 0
        assert 'manual_timer' in profiler.metrics

    def test_get_metrics_report(self):
        """Testa geração de relatório de métricas"""
        profiler = Profiler(enabled=True)

        with profiler.timer('test1'):
            pass
        with profiler.timer('test2'):
            pass

        report = profiler.get_metrics_report()

        assert isinstance(report, dict)
        assert 'test1' in report
        assert 'test2' in report

    def test_get_bottlenecks(self):
        """Testa identificação de gargalos"""
        profiler = Profiler(enabled=True)

        # Função rápida
        with profiler.timer('fast_function'):
            time.sleep(0.001)

        # Função lenta
        with profiler.timer('slow_function'):
            time.sleep(0.02)

        bottlenecks = profiler.get_bottlenecks(threshold_ms=10.0)

        assert len(bottlenecks) > 0
        assert any(b['function'] == 'slow_function' for b in bottlenecks)

    def test_reset_metrics(self):
        """Testa reset de métricas"""
        profiler = Profiler(enabled=True)

        with profiler.timer('test'):
            pass

        assert len(profiler.metrics) > 0

        profiler.reset_metrics()

        assert len(profiler.metrics) == 0

    def test_detailed_profiling(self):
        """Testa profiling detalhado com cProfile"""
        profiler = Profiler(enabled=True)

        profiler.start_detailed_profiling()

        # Executa alguma operação
        sum(range(1000))

        report = profiler.stop_detailed_profiling()

        assert isinstance(report, str)
        assert len(report) > 0


class TestSwarmProfiler:
    """Testes para SwarmProfiler"""

    def test_initialization(self):
        """Testa inicialização do SwarmProfiler"""
        profiler = SwarmProfiler(enabled=True)

        assert profiler.enabled is True
        assert 'consensus_steps' in profiler.swarm_metrics

    def test_profile_consensus(self):
        """Testa profiling de consenso"""
        profiler = SwarmProfiler(enabled=True)

        with profiler.profile_consensus():
            time.sleep(0.001)

        assert profiler.swarm_metrics['consensus_steps'] == 1
        assert 'consensus.total' in profiler.metrics

    def test_profile_pathfinding(self):
        """Testa profiling de pathfinding"""
        profiler = SwarmProfiler(enabled=True)

        with profiler.profile_pathfinding('astar'):
            time.sleep(0.001)

        assert profiler.swarm_metrics['pathfinding_calls'] == 1
        assert 'pathfinding.astar' in profiler.metrics

    def test_profile_physics(self):
        """Testa profiling de física"""
        profiler = SwarmProfiler(enabled=True)

        with profiler.profile_physics():
            time.sleep(0.001)

        assert profiler.swarm_metrics['physics_steps'] == 1
        assert 'physics.total' in profiler.metrics

    def test_record_collision(self):
        """Testa registro de colisões"""
        profiler = SwarmProfiler(enabled=True)

        profiler.record_collision()
        assert profiler.swarm_metrics['collisions_detected'] == 1

        profiler.record_collision(count=3)
        assert profiler.swarm_metrics['collisions_detected'] == 4

    def test_record_formation_change(self):
        """Testa registro de mudança de formação"""
        profiler = SwarmProfiler(enabled=True)

        profiler.record_formation_change()
        assert profiler.swarm_metrics['formations_changed'] == 1

    def test_get_swarm_report(self):
        """Testa geração de relatório do enxame"""
        profiler = SwarmProfiler(enabled=True)

        with profiler.profile_consensus():
            pass
        with profiler.profile_pathfinding('astar'):
            pass

        report = profiler.get_swarm_report()

        assert isinstance(report, dict)
        assert 'swarm_metrics' in report
        assert 'performance_metrics' in report
        assert 'derived_metrics' in report

    def test_calculate_derived_metrics(self):
        """Testa cálculo de métricas derivadas"""
        profiler = SwarmProfiler(enabled=True)

        # Simula algumas operações
        with profiler.profile_consensus():
            time.sleep(0.01)
        with profiler.profile_pathfinding('astar'):
            time.sleep(0.01)
        with profiler.profile_physics():
            time.sleep(0.01)

        derived = profiler._calculate_derived_metrics()

        assert isinstance(derived, dict)
        # Pode ter métricas de eficiência se houver dados suficientes


class TestProfilerGlobal:
    """Testes para funções globais do profiler"""

    def test_get_profiler(self):
        """Testa obtenção do profiler global"""
        profiler = get_profiler()

        assert isinstance(profiler, SwarmProfiler)

    def test_enable_disable_profiling(self):
        """Testa habilitação/desabilitação de profiling"""
        profiler = get_profiler()

        initial_state = profiler.enabled

        disable_profiling()
        assert profiler.enabled is False

        enable_profiling()
        assert profiler.enabled is True

        # Restaura estado original
        if not initial_state:
            disable_profiling()







