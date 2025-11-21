"""
Sistema de profiling avançado para análise de performance do simulador de drones
Fornece medições detalhadas de tempo, memória e gargalos de performance
"""

import cProfile
import io
import logging
import pstats
import threading
import time
from collections import defaultdict, deque
from contextlib import contextmanager
from dataclasses import dataclass, field
from functools import wraps
from typing import Any, Callable, Dict, List, Optional

import numpy as np
import psutil


@dataclass
class PerformanceMetrics:
    """Métricas de performance coletadas durante a execução"""

    # Timing
    total_time: float = 0.0
    call_count: int = 0
    average_time: float = 0.0
    min_time: float = float('inf')
    max_time: float = 0.0
    last_time: float = 0.0

    # Memory
    peak_memory_mb: float = 0.0
    average_memory_mb: float = 0.0
    memory_samples: List[float] = field(default_factory=list)

    # CPU
    cpu_percent: float = 0.0
    cpu_samples: List[float] = field(default_factory=list)

    # Custom metrics
    custom_metrics: Dict[str, Any] = field(default_factory=dict)

    def update_timing(self, execution_time: float):
        """Atualiza métricas de timing"""
        self.total_time += execution_time
        self.call_count += 1
        self.average_time = self.total_time / self.call_count
        self.min_time = min(self.min_time, execution_time)
        self.max_time = max(self.max_time, execution_time)
        self.last_time = execution_time

    def update_memory(self, memory_mb: float):
        """Atualiza métricas de memória"""
        self.memory_samples.append(memory_mb)
        if len(self.memory_samples) > 100:  # Mantém últimas 100 amostras
            self.memory_samples.pop(0)

        self.peak_memory_mb = max(self.peak_memory_mb, memory_mb)
        self.average_memory_mb = np.mean(self.memory_samples)

    def update_cpu(self, cpu_percent: float):
        """Atualiza métricas de CPU"""
        self.cpu_samples.append(cpu_percent)
        if len(self.cpu_samples) > 50:  # Mantém últimas 50 amostras
            self.cpu_samples.pop(0)

        self.cpu_percent = np.mean(self.cpu_samples[-10:])  # Média dos últimos 10

    def to_dict(self) -> Dict[str, Any]:
        """Converte métricas para dicionário"""
        return {
            'total_time': self.total_time,
            'call_count': self.call_count,
            'average_time': self.average_time,
            'min_time': self.min_time,
            'max_time': self.max_time,
            'last_time': self.last_time,
            'peak_memory_mb': self.peak_memory_mb,
            'average_memory_mb': self.average_memory_mb,
            'cpu_percent': self.cpu_percent,
            'custom_metrics': self.custom_metrics
        }


class Profiler:
    """Sistema principal de profiling com medição automática"""

    def __init__(self, enabled: bool = True, sample_interval: float = 0.1):
        self.enabled = enabled
        self.sample_interval = sample_interval
        self.metrics: Dict[str, PerformanceMetrics] = defaultdict(PerformanceMetrics)
        self.active_timers: Dict[str, float] = {}
        self.memory_baseline = psutil.Process().memory_info().rss / 1024 / 1024  # MB

        # Profiling contínuo
        self.continuous_profiling = False
        self.profiling_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()

        # Profiling detalhado
        self.profiler = cProfile.Profile() if enabled else None
        self.profiling_active = False

        self.logger = logging.getLogger("Profiler")

        if enabled:
            self.start_continuous_monitoring()

    def start_continuous_monitoring(self):
        """Inicia monitoramento contínuo de recursos do sistema"""
        if not self.enabled:
            return

        self.continuous_profiling = True
        self.profiling_thread = threading.Thread(target=self._monitor_resources, daemon=True)
        self.profiling_thread.start()
        self.logger.info("Continuous profiling started")

    def stop_continuous_monitoring(self):
        """Para monitoramento contínuo"""
        if not self.continuous_profiling:
            return

        self.continuous_profiling = False
        self.stop_event.set()
        if self.profiling_thread:
            self.profiling_thread.join()
        self.logger.info("Continuous profiling stopped")

    def _monitor_resources(self):
        """Thread de monitoramento de recursos"""
        process = psutil.Process()

        while not self.stop_event.is_set():
            try:
                # CPU
                cpu_percent = process.cpu_percent()
                self.metrics['system'].update_cpu(cpu_percent)

                # Memória
                memory_mb = process.memory_info().rss / 1024 / 1024
                memory_mb -= self.memory_baseline  # Memória relativa
                self.metrics['system'].update_memory(memory_mb)

                # Métricas customizadas
                self._update_system_metrics()

            except Exception as e:
                self.logger.error(f"Error in resource monitoring: {e}")

            self.stop_event.wait(self.sample_interval)

    def _update_system_metrics(self):
        """Atualiza métricas personalizadas do sistema"""
        try:
            # Número de threads
            thread_count = threading.active_count()
            self.metrics['system'].custom_metrics['thread_count'] = thread_count

            # Uso de disco (se aplicável)
            disk_usage = psutil.disk_usage('/').percent
            self.metrics['system'].custom_metrics['disk_usage_percent'] = disk_usage

        except Exception:
            pass

    @contextmanager
    def timer(self, name: str):
        """Context manager para medição de tempo"""
        if not self.enabled:
            yield
            return

        start_time = time.perf_counter()
        try:
            yield
        finally:
            execution_time = time.perf_counter() - start_time
            self.metrics[name].update_timing(execution_time)

    def time_function(self, name: str = None):
        """Decorador para medição automática de funções"""
        def decorator(func: Callable):
            func_name = name or f"{func.__module__}.{func.__qualname__}"

            @wraps(func)
            def wrapper(*args, **kwargs):
                if not self.enabled:
                    return func(*args, **kwargs)

                with self.timer(func_name):
                    return func(*args, **kwargs)

            return wrapper
        return decorator

    def start_timer(self, name: str):
        """Inicia timer manual"""
        if not self.enabled:
            return

        self.active_timers[name] = time.perf_counter()

    def stop_timer(self, name: str) -> float:
        """Para timer manual e retorna tempo decorrido"""
        if not self.enabled or name not in self.active_timers:
            return 0.0

        execution_time = time.perf_counter() - self.active_timers[name]
        del self.active_timers[name]
        self.metrics[name].update_timing(execution_time)
        return execution_time

    def start_detailed_profiling(self):
        """Inicia profiling detalhado com cProfile"""
        if not self.enabled or self.profiler is None:
            return

        self.profiler.enable()
        self.profiling_active = True
        self.logger.info("Detailed profiling started")

    def stop_detailed_profiling(self, sort_by: str = 'cumulative', lines: int = 20) -> str:
        """Para profiling detalhado e retorna relatório"""
        if not self.profiling_active or self.profiler is None:
            return "Detailed profiling not active"

        self.profiler.disable()
        self.profiling_active = False

        # Gera relatório
        s = io.StringIO()
        ps = pstats.Stats(self.profiler, stream=s).sort_stats(sort_by)
        ps.print_stats(lines)

        report = s.getvalue()
        self.logger.info("Detailed profiling stopped")
        return report

    def get_metrics_report(self, include_system: bool = True) -> Dict[str, Any]:
        """Gera relatório completo de métricas"""
        report = {}

        for name, metrics in self.metrics.items():
            if not include_system and name == 'system':
                continue
            report[name] = metrics.to_dict()

        return report

    def get_bottlenecks(self, threshold_ms: float = 10.0) -> List[Dict[str, Any]]:
        """Identifica gargalos de performance"""
        bottlenecks = []

        for name, metrics in self.metrics.items():
            if name == 'system':
                continue

            if metrics.average_time > threshold_ms / 1000:  # Converter para segundos
                bottlenecks.append({
                    'function': name,
                    'average_time_ms': metrics.average_time * 1000,
                    'max_time_ms': metrics.max_time * 1000,
                    'call_count': metrics.call_count,
                    'total_time_s': metrics.total_time
                })

        # Ordena por tempo médio decrescente
        bottlenecks.sort(key=lambda x: x['average_time_ms'], reverse=True)
        return bottlenecks

    def reset_metrics(self):
        """Reseta todas as métricas coletadas"""
        self.metrics.clear()
        self.active_timers.clear()
        self.logger.info("Metrics reset")

    def log_performance_summary(self):
        """Log sumário de performance"""
        if not self.enabled:
            return

        bottlenecks = self.get_bottlenecks()

        self.logger.info("=== Performance Summary ===")
        self.logger.info(f"Total functions profiled: {len(self.metrics) - 1}")  # -1 para system

        if bottlenecks:
            self.logger.info("Top bottlenecks:")
            for i, bottleneck in enumerate(bottlenecks[:5], 1):
                self.logger.info(f"  {i}. {bottleneck['function']}: "
                               f"{bottleneck['average_time_ms']:.2f}ms "
                               f"(calls: {bottleneck['call_count']})")

        system_metrics = self.metrics.get('system')
        if system_metrics:
            self.logger.info(f"CPU usage: {system_metrics.cpu_percent:.1f}%")
            self.logger.info(f"Memory usage: {system_metrics.average_memory_mb:.1f}MB")

        self.logger.info("=========================")


class SwarmProfiler(Profiler):
    """Profiler especializado para análise do simulador de enxames"""

    def __init__(self, enabled: bool = True):
        super().__init__(enabled)
        self.swarm_metrics = {
            'consensus_steps': 0,
            'pathfinding_calls': 0,
            'physics_steps': 0,
            'collisions_detected': 0,
            'formations_changed': 0,
            'websocket_messages': 0
        }

    @contextmanager
    def profile_consensus(self):
        """Profile specifically consensus operations"""
        with self.timer('consensus.total'):
            yield
        self.swarm_metrics['consensus_steps'] += 1

    @contextmanager
    def profile_pathfinding(self, algorithm: str = 'unknown'):
        """Profile pathfinding operations"""
        with self.timer(f'pathfinding.{algorithm}'):
            yield
        self.swarm_metrics['pathfinding_calls'] += 1

    @contextmanager
    def profile_physics(self):
        """Profile physics simulation"""
        with self.timer('physics.total'):
            yield
        self.swarm_metrics['physics_steps'] += 1

    @contextmanager
    def profile_collision_detection(self):
        """Profile collision detection"""
        with self.timer('collision_detection'):
            yield

    def record_collision(self, count: int = 1):
        """Record collision detection"""
        self.swarm_metrics['collisions_detected'] += count

    def record_formation_change(self):
        """Record formation change"""
        self.swarm_metrics['formations_changed'] += 1

    def record_websocket_message(self):
        """Record WebSocket message"""
        self.swarm_metrics['websocket_messages'] += 1

    def get_swarm_report(self) -> Dict[str, Any]:
        """Relatório específico do enxame"""
        base_report = self.get_metrics_report()

        # Adiciona métricas específicas do enxame
        swarm_report = {
            'swarm_metrics': self.swarm_metrics.copy(),
            'performance_metrics': base_report,
            'derived_metrics': self._calculate_derived_metrics()
        }

        return swarm_report

    def _calculate_derived_metrics(self) -> Dict[str, Any]:
        """Calcula métricas derivadas específicas do enxame"""
        derived = {}

        # Eficiência de consenso
        consensus_time = self.metrics.get('consensus.total', PerformanceMetrics()).total_time
        if self.swarm_metrics['consensus_steps'] > 0:
            derived['consensus_efficiency'] = consensus_time / self.swarm_metrics['consensus_steps']

        # Eficiência de pathfinding
        pathfinding_time = sum(metrics.total_time for name, metrics in self.metrics.items()
                              if name.startswith('pathfinding.'))
        if self.swarm_metrics['pathfinding_calls'] > 0:
            derived['pathfinding_efficiency'] = pathfinding_time / self.swarm_metrics['pathfinding_calls']

        # Eficiência física
        physics_time = self.metrics.get('physics.total', PerformanceMetrics()).total_time
        if self.swarm_metrics['physics_steps'] > 0:
            derived['physics_efficiency'] = physics_time / self.swarm_metrics['physics_steps']

        # Taxa de colisões
        if self.swarm_metrics['physics_steps'] > 0:
            derived['collision_rate'] = self.swarm_metrics['collisions_detected'] / self.swarm_metrics['physics_steps']

        return derived

    def log_swarm_performance_summary(self):
        """Log sumário específico do enxame"""
        self.log_performance_summary()

        self.logger.info("=== Swarm Performance Summary ===")
        self.logger.info(f"Consensus steps: {self.swarm_metrics['consensus_steps']}")
        self.logger.info(f"Pathfinding calls: {self.swarm_metrics['pathfinding_calls']}")
        self.logger.info(f"Physics steps: {self.swarm_metrics['physics_steps']}")
        self.logger.info(f"Collisions detected: {self.swarm_metrics['collisions_detected']}")
        self.logger.info(f"Formation changes: {self.swarm_metrics['formations_changed']}")
        self.logger.info(f"WebSocket messages: {self.swarm_metrics['websocket_messages']}")

        derived = self._calculate_derived_metrics()
        for metric, value in derived.items():
            if 'efficiency' in metric:
                self.logger.info(f"{metric}: {value*1000:.2f}ms")
            elif 'rate' in metric:
                self.logger.info(f"{metric}: {value:.4f}")
            else:
                self.logger.info(f"{metric}: {value}")

        self.logger.info("=================================")


# Instância global do profiler
_global_profiler = SwarmProfiler(enabled=True)


def get_profiler() -> SwarmProfiler:
    """Retorna instância global do profiler"""
    return _global_profiler


def enable_profiling():
    """Habilita profiling global"""
    _global_profiler.enabled = True
    _global_profiler.start_continuous_monitoring()


def disable_profiling():
    """Desabilita profiling global"""
    _global_profiler.enabled = False
    _global_profiler.stop_continuous_monitoring()


def profile_function(name: str = None):
    """Decorador para profiling de funções"""
    return _global_profiler.time_function(name)


# Decoradores específicos para o simulador
profile_consensus = _global_profiler.profile_consensus
profile_pathfinding = _global_profiler.profile_pathfinding
profile_physics = _global_profiler.profile_physics
profile_collision_detection = _global_profiler.profile_collision_detection
