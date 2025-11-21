# Guia de Contribuição

Obrigado por considerar contribuir para o Drone Swarm Coordination Simulator! Este documento fornece diretrizes e informações sobre como contribuir para o projeto.

## Código de Conduta

Este projeto segue um código de conduta profissional. Ao participar, você concorda em manter um ambiente respeitoso e colaborativo.

## Como Posso Contribuir?

### Reportar Bugs

Se você encontrou um bug:

1. Verifique se o bug já não foi reportado nas [Issues](https://github.com/lucien-vallois/drone-swarm-coordination/issues)
2. Se não foi reportado, crie uma nova issue com:
   - Descrição clara do problema
   - Passos para reproduzir
   - Comportamento esperado vs. comportamento atual
   - Versão do Python e sistema operacional
   - Logs ou mensagens de erro relevantes

### Sugerir Melhorias

Sugestões são sempre bem-vindas! Para sugerir uma melhoria:

1. Abra uma issue com a tag `enhancement`
2. Descreva a funcionalidade proposta
3. Explique por que seria útil
4. Se possível, forneça exemplos de uso

### Contribuir com Código

#### Configuração do Ambiente de Desenvolvimento

1. **Fork o repositório** e clone localmente:
```bash
git clone https://github.com/seu-usuario/drone-swarm-coordination.git
cd drone-swarm-coordination
```

2. **Crie um ambiente virtual**:
```bash
python -m venv venv
source venv/bin/activate  # No Windows: venv\Scripts\activate
```

3. **Instale dependências**:
```bash
pip install -r requirements.txt
pip install -r requirements-dev.txt
```

4. **Verifique a instalação**:
```bash
python tests/run_tests.py
```

#### Processo de Desenvolvimento

1. **Crie uma branch** para sua feature/fix:
```bash
git checkout -b feature/nome-da-feature
# ou
git checkout -b fix/descricao-do-bug
```

2. **Faça suas alterações** seguindo os padrões de código abaixo

3. **Execute os testes** antes de commitar:
```bash
# Todos os testes
python tests/run_tests.py

# Com cobertura
python tests/run_tests.py --coverage

# Verificar qualidade de código
flake8 backend/ tests/
black --check backend/ tests/
mypy backend/
```

4. **Commit suas alterações**:
```bash
git add .
git commit -m "feat: adiciona nova funcionalidade X"
```

5. **Push para seu fork**:
```bash
git push origin feature/nome-da-feature
```

6. **Abra um Pull Request** no repositório original

## Padrões de Código

### Formatação

Este projeto usa **Black** para formatação automática:

```bash
black backend/ tests/ examples/
```

**Configuração**: Linha máxima de 127 caracteres (definido em `pyproject.toml`)

### Linting

Usamos **flake8** para verificação de estilo:

```bash
flake8 backend/ tests/ examples/
```

**Regras importantes**:
- Máximo de 127 caracteres por linha
- Sem imports não utilizados
- Nomes de variáveis descritivos

### Type Checking

Usamos **mypy** para verificação de tipos:

```bash
mypy backend/
```

**Nota**: Type hints são encorajados mas não obrigatórios em todo o código.

### Convenções de Nomenclatura

- **Classes**: `PascalCase` (ex: `SwarmCoordinator`)
- **Funções/Métodos**: `snake_case` (ex: `find_path`)
- **Constantes**: `UPPER_SNAKE_CASE` (ex: `MAX_VELOCITY`)
- **Variáveis**: `snake_case` (ex: `drone_position`)

### Docstrings

Use docstrings no formato Google Style:

```python
def calculate_distance(pos1: np.ndarray, pos2: np.ndarray) -> float:
    """
    Calcula a distância euclidiana entre duas posições.
    
    Args:
        pos1: Primeira posição [x, y, z]
        pos2: Segunda posição [x, y, z]
    
    Returns:
        Distância euclidiana em metros
    
    Raises:
        ValueError: Se as posições não tiverem dimensão 3
    """
    if len(pos1) != 3 or len(pos2) != 3:
        raise ValueError("Posições devem ter 3 dimensões")
    return np.linalg.norm(pos1 - pos2)
```

## Estrutura de Testes

### Escrevendo Testes

- Todos os novos recursos devem incluir testes
- Testes devem ser colocados em `tests/test_*.py`
- Use fixtures do `conftest.py` quando apropriado
- Nomeie testes descritivamente: `test_consensus_leader_election_success`

### Executando Testes

```bash
# Todos os testes
pytest

# Testes específicos
pytest tests/test_consensus.py

# Com cobertura
pytest --cov=backend --cov-report=html

# Apenas testes de integração
pytest -m integration
```

### Cobertura de Código

- **Objetivo**: ≥80% cobertura geral
- **Módulos críticos**: ≥95% (consensus, pathfinding, physics)
- Verifique cobertura antes de fazer PR:
```bash
python tests/run_tests.py --check-coverage
```

## Convenções de Commit

Seguimos o padrão [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Tipos de Commit

- `feat`: Nova funcionalidade
- `fix`: Correção de bug
- `docs`: Documentação
- `style`: Formatação (não afeta código)
- `refactor`: Refatoração
- `test`: Adição/correção de testes
- `chore`: Tarefas de manutenção

### Exemplos

```
feat(pathfinding): adiciona algoritmo RRT* para pathfinding 3D

fix(consensus): corrige eleição de líder em caso de falha simultânea

docs: atualiza README com instruções de instalação

test(physics): adiciona testes para simulação de vento
```

## Processo de Pull Request

### Antes de Submeter

- [ ] Código segue os padrões de formatação (Black)
- [ ] Todos os testes passam
- [ ] Novos testes foram adicionados para novas funcionalidades
- [ ] Documentação foi atualizada se necessário
- [ ] Código foi revisado por você mesmo
- [ ] Commits seguem o padrão Conventional Commits

### Template de Pull Request

Ao abrir um PR, inclua:

1. **Descrição**: O que foi alterado e por quê
2. **Tipo**: Feature, Bug Fix, Documentation, etc.
3. **Testes**: Como testar as alterações
4. **Checklist**: Itens acima marcados

### Revisão de Código

- PRs serão revisados por mantenedores
- Feedback será fornecido de forma construtiva
- Alterações solicitadas devem ser implementadas antes do merge

## Adicionando Novos Algoritmos

### Usando Interfaces Modulares

O projeto suporta extensibilidade através de interfaces. Para adicionar um novo algoritmo:

1. **Implemente a interface apropriada** em `backend/interfaces.py`
2. **Crie sua implementação** seguindo o contrato
3. **Use a factory** para integrar:
```python
from backend.factories import create_custom_factory
from backend.interfaces import IPathfindingAlgorithm

class MeuPathfinder(IPathfindingAlgorithm):
    def find_path(self, start, goal, **kwargs):
        # Sua implementação
        pass

factory = create_custom_factory(pathfinding_algorithm=MeuPathfinder)
```

4. **Adicione testes** para o novo algoritmo
5. **Documente** no README e docs/

## Estrutura do Projeto

```
drone-swarm-coordination/
├── backend/          # Código principal Python
├── frontend/         # Interface web (Three.js)
├── tests/            # Testes automatizados
├── examples/         # Exemplos de uso
├── docs/             # Documentação técnica
└── scripts/          # Scripts utilitários
```

## Dúvidas?

- Abra uma issue com a tag `question`
- Consulte a documentação em `docs/`
- Veja exemplos em `examples/`

## Agradecimentos

Obrigado por contribuir para tornar este projeto melhor! 🚀







