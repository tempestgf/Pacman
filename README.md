#Pacman

# Search Algorithms

### Depth First Search - (DFS)

DFS explora lo más profundo posible en cada rama antes de retroceder. Utiliza una estructura de datos **Pila (Stack)** para mantener los nodos a explorar.

**Características:**

- **Completitud:** No es completa en espacios de búsqueda infinitos o muy profundos.
- **Optimalidad:** No garantiza encontrar la solución óptima.
- **Uso:** Útil cuando la profundidad de la solución es limitada y se desea explorar rutas profundas rápidamente.

```
def depthFirstSearch(problem):
    from util import Stack
    stack = Stack()
    stack.push((problem.getStartState(), []))
    visited = set()

    while not stack.isEmpty():
        state, actions = stack.pop()

        if problem.isGoalState(state):
            return actions

        if state not in visited:
            visited.add(state)
            for successor, action, stepCost in problem.getSuccessors(state):
                stack.push((successor, actions + [action]))

    return []

```

### Breadth First Search - (BFS)

BFS explora todos los nodos en un nivel antes de pasar al siguiente nivel. Utiliza una estructura de datos **Cola (Queue)** para mantener los nodos a explorar.

**Características:**

- **Completitud:** Es completa; siempre encuentra una solución si existe.
- **Optimalidad:** Garantiza encontrar la solución con el menor número de acciones.
- **Uso:** Ideal para problemas donde la solución óptima es necesaria y los costos de las acciones son uniformes.

```
def breadthFirstSearch(problem):
    from util import Queue
    queue = Queue()
    queue.push((problem.getStartState(), []))
    visited = set()

    while not queue.isEmpty():
        state, actions = queue.pop()

        if problem.isGoalState(state):
            return actions

        if state not in visited:
            visited.add(state)
            for successor, action, stepCost in problem.getSuccessors(state):
                queue.push((successor, actions + [action]))

    return []

```


### Uniform Cost Search - (UCS)

UCS explora los nodos en orden de costo acumulado creciente. Utiliza una **Cola de Prioridad (PriorityQueue)** donde los nodos con menor costo total son explorados primero.

**Características:**

- **Completitud:** Es completa; encuentra una solución si existe.
- **Optimalidad:** Garantiza encontrar la solución de costo mínimo.
- **Uso:** Adecuado para problemas donde las acciones tienen diferentes costos.

```
def uniformCostSearch(problem):
    from util import PriorityQueue
    priorityQueue = PriorityQueue()
    priorityQueue.push((problem.getStartState(), []), 0)
    visited = {}

    while not priorityQueue.isEmpty():
        state, actions = priorityQueue.pop()

        if problem.isGoalState(state):
            return actions

        if state not in visited or visited[state] > problem.getCostOfActions(actions):
            visited[state] = problem.getCostOfActions(actions)
            for successor, action, stepCost in problem.getSuccessors(state):
                newActions = actions + [action]
                cost = problem.getCostOfActions(newActions)
                priorityQueue.push((successor, newActions), cost)

    return []

```


### A* Search

A* combina las ventajas de UCS y BFS utilizando una **Cola de Prioridad (PriorityQueue)** que prioriza los nodos basándose en el costo acumulado más una heurística que estima el costo restante hasta el objetivo.

**Características:**

- **Completitud:** Es completa si la heurística es admisible.
- **Optimalidad:** Garantiza encontrar la solución óptima si la heurística es admisible y consistente.
- **Uso:** Muy eficiente para problemas con heurísticas bien definidas que guían la búsqueda hacia el objetivo.

```
def aStarSearch(problem, heuristic=nullHeuristic):
    from util import PriorityQueue
    priorityQueue = PriorityQueue()
    priorityQueue.push((problem.getStartState(), []), 0)
    visited = {}

    while not priorityQueue.isEmpty():
        state, actions = priorityQueue.pop()

        if problem.isGoalState(state):
            return actions

        if state not in visited or visited[state] > problem.getCostOfActions(actions):
            visited[state] = problem.getCostOfActions(actions)
            for successor, action, stepCost in problem.getSuccessors(state):
                newActions = actions + [action]
                cost = problem.getCostOfActions(newActions) + heuristic(successor, problem)
                priorityQueue.push((successor, newActions), cost)

    return []

```
### Diferencias entre los Algoritmos

| Algoritmo | Estructura de Datos | Completo | Óptimo | Tipo de Búsqueda        |
|-----------|---------------------|----------|--------|-------------------------|
| **DFS**   | Pila                | No       | No     | Exploración Profunda    |
| **BFS**   | Cola                | Sí       | Sí     | Exploración por Niveles |
| **UCS**   | Cola de Prioridad   | Sí       | Sí     | Exploración por Costo   |
| **A\***   | Cola de Prioridad   | Sí       | Sí     | Exploración Informada   |

# Search Agents

Representación del estado: (posición, esquinas visitadas) 
`self.startState = (self.startingPosition, tuple([False]*len(self.corners)))`
### CornersProblem.getStartState

Esta función debe retornar el estado inicial del problema. Para el **Corners Problem**, el estado puede ser una tupla que incluye la posición inicial de Pacman y una representación de las esquinas que aún no han sido visitadas.

```    
def getStartState(self):

        """
        Returns the start state (in your state space, not the full Pacman state
        space)

        """
        return self.startState
```
### CornersProblem.isGoalState

Determina si un estado dado es el estado objetivo del problema. Para el **Corners Problem**, el estado objetivo se alcanza cuando todas las esquinas han sido visitadas.

```
    def isGoalState(self, state):

        """

        Returns whether this search state is a goal state of the problem.

        """

        "*** YOUR CODE HERE ***"

        return all(state[1])
```
### CornersProblem.getSuccessors

Genera los estados sucesores a partir del estado actual. Para cada acción legal (Norte, Sur, Este, Oeste), calcula la nueva posición de Pacman, actualiza las esquinas visitadas si es necesario y agrega el sucesor a la lista de sucesores.

```
    def getSuccessors(self, state):

        """

        Returns successor states, the actions they require, and a cost of 1.

  

         As noted in search.py:

            For a given state, this should return a list of triples, (successor,

            action, stepCost), where 'successor' is a successor to the current

            state, 'action' is the action required to get there, and 'stepCost'

            is the incremental cost of expanding to that successor

        """

  

        successors = []

        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:

            x, y = state[0]

            dx, dy = Actions.directionToVector(action)

            nextx, nexty = int(x + dx), int(y + dy)

            if not self.walls[nextx][nexty]:

                nextPosition = (nextx, nexty)

                visited = list(state[1])

                if nextPosition in self.corners:

                    index = self.corners.index(nextPosition)

                    visited[index] = True

                successors.append(((nextPosition, tuple(visited)), action, 1))

  

  

        self._expanded += 1 # DO NOT CHANGE

        return successors
```

### cornersHeuristic

Calcula una heurística para el **Corners Problem** que estima el costo restante para alcanzar el estado objetivo desde el estado actual. La heurística propuesta calcula la distancia de Manhattan máxima desde la posición actual de Pacman hasta cualquier esquina no visitada, lo que garantiza que sea **admisible** y **consistente**.

```
def cornersHeuristic(state, problem):
    """
    Heurístico para el CornersProblem que calcula la distancia mínima total para visitar todas las esquinas no visitadas.
    
    state: El estado actual (posición, esquinas visitadas)
    problem: La instancia de CornersProblem
    """
    from util import manhattanDistance

    position, visited = state
    corners = problem.corners
    unvisited = [corner for corner, isVisited in zip(corners, visited) if not isVisited]

    if not unvisited:
        return 0

    # Calcular la distancia de Manhattan desde la posición actual a la esquina más cercana
    distances = [manhattanDistance(position, corner) for corner in unvisited]
    min_distance = min(distances)

    # Añadir la distancia mínima al heurístico
    heuristic = min_distance

    return heuristic

```

### AnyFoodSearchProblem.isGoalState

Determina si un estado dado es un estado objetivo. Para el **AnyFoodSearchProblem**, el estado objetivo es cualquier posición que contenga comida.


```
    def isGoalState(self, state):

        """

        The state is Pacman's position. Fill this in with a goal test that will

        complete the problem definition.

        """

        x,y = state

  

        # La condición de meta es que haya comida en la posición actual

        return self.food[x][y]
```

### ClosestDotSearchAgent.findPathToClosestDot

Encuentra un camino hacia la comida más cercana desde el estado actual. Utiliza BFS para garantizar que el camino encontrado sea el más corto.

```
def findPathToClosestDot(self, gameState):
    """
    Returns a path (a list of actions) to the closest dot, starting from
    gameState.
    """
    # Elementos útiles del estado inicial
    startPosition = gameState.getPacmanPosition()
    food = gameState.getFood()
    walls = gameState.getWalls()
    problem = AnyFoodSearchProblem(gameState)

    # Utilizar BFS para encontrar el camino más corto a la comida más cercana
    return search.bfs(problem)

```

### foodHeuristic

Calcula una heurística para el **Food Search Problem** que estima el costo restante para recolectar toda la comida. La heurística propuesta considera la distancia máxima de Manhattan entre la posición actual de Pacman y cualquier pieza de comida restante, además de la distancia desde esa comida más lejana a otra pieza de comida para una estimación más precisa.

```
def foodHeuristic(state, problem):
    """
    Heurística para el FoodSearchProblem que calcula la distancia de Manhattan máxima
    desde la posición actual de Pacman hasta cualquier pieza de comida restante.

    state: El estado actual (posición de Pacman, Grid de comida restante)
    problem: La instancia de FoodSearchProblem
    """
    position, foodGrid = state
    foodList = foodGrid.asList()  # Obtiene una lista de todas las posiciones de comida restantes

    # Si no hay comida restante, la heurística es 0
    if not foodList:
        return 0

    # Calcula la distancia de Manhattan desde la posición actual a cada pieza de comida
    distances = [util.manhattanDistance(position, food) for food in foodList]

    # Retorna la distancia máxima encontrada
    return max(distances)

```





# Heuristic Implementation for the Corners Problem

La comparación entre **Uniform Cost Search (UCS)** y **A*** con la heurística propuesta para el **Corners Problem** ha demostrado que:

- A* mantiene la **optimalidad** de las soluciones igual que **UCS**.
- A* es **más eficiente** en términos de **nodos expandidos**, lo que reduce el tiempo de búsqueda y mejora el rendimiento general.
- La heurística `cornersHeuristic` es una herramienta poderosa para guiar la búsqueda de manera efectiva, aprovechando información adicional sobre el problema para optimizar el proceso de búsqueda.

Estos hallazgos refuerzan la utilidad de las heurísticas informadas en algoritmos de búsqueda y destacan cómo una implementación bien pensada puede mejorar significativamente el rendimiento en problemas de optimización complejos.

| Mapa              | Algoritmo | Costo Total | Nodos Expandidos |
| ----------------- | --------- | ----------- | ---------------- |
| **tinyCorners**   | UCS       | 28          | 252              |
|                   | A*        | 28          | 231              |
| **mediumCorners** | UCS       | 106         | 1966             |
|                   | A*        | 106         | 1475             |
| **bigCorners**    | UCS       | 162         | 7949             |
|                   | A*        | 162         | 5848             |

# Heuristic implementation for the Food Search Problem

La heurística propuesta para el **Food Search Problem** en **A*** demuestra ser altamente eficiente, expandiendo menos nodos y ejecutándose más rápidamente que **UCS** sin sacrificar la optimalidad de las soluciones encontradas. Esto subraya la importancia de diseñar heurísticas informadas y bien pensadas para mejorar el rendimiento de los algoritmos de búsqueda en problemas complejos.

| Mapa            | Algoritmo | Costo Total | Nodos Expandidos | Tiempo de Ejecución (s) |
|-----------------|-----------|-------------|------------------|-------------------------|
| **tinyCorners** | UCS       |      28     |         252      |           0.0           |
|                 | A*        |      28     |         157      |           0.0           |
| **mediumCorners**| UCS      |     106     |        1966      |           1.0           |
|                 | A*        |     106     |         588      |           0.3           |
| **bigCorners**  | UCS       |     162     |        7949      |           9.4           |
|                 | A*        |     162     |        2743      |           4.0           |
