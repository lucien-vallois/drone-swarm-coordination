#!/usr/bin/env python3
"""
Script para executar testes do drone swarm coordination.
"""

import argparse
import subprocess
import sys


def run_tests(test_type="all", verbose=False, coverage=False):
    """Executa os testes especificados"""

    cmd = ["python", "-m", "pytest"]

    if test_type == "unit":
        cmd.extend(["-m", "not integration and not performance"])
    elif test_type == "integration":
        cmd.extend(["-m", "integration"])
    elif test_type == "performance":
        cmd.extend(["-m", "performance"])
    elif test_type == "consensus":
        cmd.extend(["tests/test_consensus.py"])
    elif test_type == "pathfinding":
        cmd.extend(["tests/test_pathfinding.py"])
    elif test_type == "physics":
        cmd.extend(["tests/test_physics.py"])
    elif test_type == "collision":
        cmd.extend(["tests/test_collision_avoidance.py"])

    if verbose:
        cmd.append("-v")
    else:
        cmd.append("-q")

    if coverage:
        cmd.extend(["--cov=backend", "--cov-report=term-missing"])

    print(f"Executando: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=".")

    return result.returncode


def run_benchmarks():
    """Executa benchmarks de performance"""
    print("Executando benchmarks de performance...")

    cmd = [
        "python", "-m", "pytest",
        "tests/test_integration.py::TestPerformanceBenchmarks",
        "-v", "--tb=short"
    ]

    result = subprocess.run(cmd, cwd=".")
    return result.returncode


def check_coverage():
    """Verifica cobertura de código"""
    print("Verificando cobertura de código...")

    cmd = [
        "python", "-m", "pytest",
        "--cov=backend",
        "--cov-report=html",
        "--cov-report=term-missing",
        "--cov-fail-under=80"
    ]

    result = subprocess.run(cmd, cwd=".")
    return result.returncode


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Executar testes do Drone Swarm Coordination")
    parser.add_argument(
        "test_type",
        choices=["all", "unit", "integration", "performance", "consensus", "pathfinding", "physics", "collision"],
        default="all",
        nargs="?",
        help="Tipo de teste a executar"
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Saída verbosa")
    parser.add_argument("-c", "--coverage", action="store_true", help="Incluir relatório de cobertura")
    parser.add_argument("--benchmarks", action="store_true", help="Executar apenas benchmarks")
    parser.add_argument("--check-coverage", action="store_true", help="Verificar cobertura mínima")

    args = parser.parse_args()

    if args.check_coverage:
        sys.exit(check_coverage())
    elif args.benchmarks:
        sys.exit(run_benchmarks())
    else:
        sys.exit(run_tests(args.test_type, args.verbose, args.coverage))
