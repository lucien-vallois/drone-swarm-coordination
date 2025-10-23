# Drone Swarm Coordination - Test Suite

Suite completa de testes para o simulador de coordenação de enxames de drones.

## Estrutura dos Testes

```
tests/
├── __init__.py              # Inicialização do pacote de testes
├── conftest.py              # Configurações compartilhadas (fixtures)
├── test_consensus.py        # Testes do algoritmo Raft-based
├── test_pathfinding.py      # Testes A* e RRT*
├── test_physics.py          # Testes de física e dinâmica
├── test_collision_avoidance.py  # Testes de formação e avoidance
├── test_integration.py      # Testes de integração e performance
├── run_tests.py             # Script de execução dos testes
└── README.md               # Esta documentação
```

## Tipos de Testes

### Testes Unitários
- **Consensus**: Eleição de líder, heartbeats, tolerância a falhas
- **Pathfinding**: A* e RRT* com obstáculos 3D
- **Physics**: Simulação física, forças aerodinâmicas, integração
- **Collision Avoidance**: Campos de força, formação de enxames

### Testes de Integração
- Coordenação entre módulos (consensus + pathfinding + physics)
- Simulação completa com WebSocket
- Sincronização entre coordinator e physics

### Testes de Performance
- Benchmarks de tempo de execução
- Testes com enxames grandes (20+ drones)
- Análise de uso de memória
- Tempo de convergência do consensus

### Testes de Tolerância a Falhas
- Recuperação de falha de drones
- Reeleição após falha do líder
- Robustez com obstáculos dinâmicos

## Executando os Testes

### Todos os Testes
```bash
# Via pytest
pytest

# Via script
python tests/run_tests.py
```

### Testes Específicos
```bash
# Apenas testes unitários
python tests/run_tests.py unit

# Apenas testes de integração
python tests/run_tests.py integration

# Apenas algoritmos de consenso
python tests/run_tests.py consensus

# Com saída verbosa
python tests/run_tests.py -v
```

### Cobertura de Código
```bash
# Relatório de cobertura
python tests/run_tests.py --coverage

# Verificar cobertura mínima (80%)
python tests/run_tests.py --check-coverage
```

### Benchmarks de Performance
```bash
# Executar benchmarks
python tests/run_tests.py --benchmarks
```

## Métricas de Qualidade

### Cobertura de Código
- **Objetivo**: ≥80% de cobertura
- **Módulos Críticos**: 100% coverage em consensus, pathfinding, physics

### Performance
- **Step Coordinator**: <10ms (60 FPS)
- **Pathfinding**: <100ms para ambientes complexos
- **Consensus**: <1s convergência
- **Memória**: <50MB para 20 drones

### Qualidade do Código
- **Linting**: Black + Flake8
- **Type Hints**: MyPy compatível
- **Documentação**: Docstrings completos

## Configuração do Ambiente de Testes

### Dependências
```bash
pip install -r requirements-dev.txt
```

### Fixtures Compartilhadas
- `sample_config`: Configuração padrão do enxame
- `sample_drone_state`: Estado de drone de exemplo
- `pathfinder_3d`: Pathfinder 3D configurado
- `drone_physics`: Física de drone individual
- `wind_field`: Campo de vento para testes

## Executando Testes em CI/CD

### GitHub Actions
```yaml
- name: Run Tests
  run: |
    pip install -r requirements-dev.txt
    python tests/run_tests.py --coverage

- name: Check Coverage
  run: |
    python tests/run_tests.py --check-coverage
```

### Pre-commit Hooks
```yaml
repos:
  - repo: local
    hooks:
      - id: run-tests
        name: Run test suite
        entry: python tests/run_tests.py
        language: system
        pass_filenames: false
```

## Debugging de Testes

### Testes Falhando
1. Verificar imports e dependências
2. Checar fixtures e configurações
3. Analisar logs de erro detalhados
4. Usar `--pdb` para debug interativo

### Performance Issues
1. Profile com `pytest --profile`
2. Identificar gargalos com `cProfile`
3. Otimizar algoritmos críticos
4. Verificar uso de memória

## Extensão da Suite de Testes

### Adicionando Novos Testes
1. Criar arquivo `test_novo_modulo.py`
2. Usar fixtures existentes quando possível
3. Seguir padrão de nomenclatura `test_*`
4. Adicionar markers apropriados (`@pytest.mark.integration`)

### Novos Benchmarks
1. Adicionar classe em `test_integration.py`
2. Usar `time.time()` para medição
3. Definir thresholds realistas
4. Incluir asserts de performance

### Testes de Regressão
1. Criar testes para bugs específicos
2. Adicionar casos edge
3. Testar configurações extremas
4. Validar correções

## Relatórios e Métricas

### Cobertura HTML
```bash
pytest --cov=backend --cov-report=html
# Abre htmlcov/index.html
```

### Relatório de Performance
```bash
python tests/run_tests.py --benchmarks
# Resultados impressos no console
```

### CI/CD Integration
- Cobertura enviada para Codecov
- Performance thresholds verificados
- Relatórios em PR comments
