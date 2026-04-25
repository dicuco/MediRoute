import heapq
from config import (
    GRID,
    TRIGGER_CELL,
    PENALTY_TARGETS,
    PENALTY_VALUE,
    NEW_BLOCKED_CELLS,
    FREE,
    CONGESTED,
    BLOCKED,
    TENTATIVE_BLOCK_COST,
)
import event_log


def direction_from_cells(a, b):
    """
    Devuelve la orientación necesaria para pasar de una celda a la siguiente.
    """
    dr = b[0] - a[0]
    dc = b[1] - a[1]

    if dr == -1 and dc == 0:
        return 0  # norte
    elif dr == 0 and dc == 1:
        return 1  # este
    elif dr == 1 and dc == 0:
        return 2  # sur
    elif dr == 0 and dc == -1:
        return 3  # oeste
    else:
        raise ValueError(f"Movimiento inválido entre celdas {a} -> {b}")


def in_bounds(cell):
    """Comprueba que la celda está dentro del mapa."""
    r, c = cell
    return 0 <= r < len(GRID) and 0 <= c < len(GRID[0])


def passable(state, cell):
    """
    Comprueba que la celda no está bloqueada ni estructural ni dinámicamente.
    """
    r, c = cell

    if GRID[r][c] != 0:
        return False

    if state["cell_states"][cell] == BLOCKED:
        return False

    return True


def neighbors(state, cell):
    """Devuelve vecinos válidos en 4 direcciones."""
    r, c = cell
    candidates = [
        (r - 1, c),
        (r + 1, c),
        (r, c - 1),
        (r, c + 1)
    ]
    return [n for n in candidates if in_bounds(n) and passable(state, n)]


def heuristic(a, b):
    """Heurística Manhattan para A*."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def traversal_cost(cost_map, cell):
    """Coste de atravesar una celda."""
    r, c = cell
    return cost_map[r][c]


def astar(state, cost_map, start_cell, goal_cell):
    """
    Algoritmo A* sobre la rejilla con mapa de costes.
    Tiene en cuenta bloqueos estructurales y bloqueos dinámicos.
    """
    frontier = []
    heapq.heappush(frontier, (0, start_cell))

    came_from = {start_cell: None}
    cost_so_far = {start_cell: 0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal_cell:
            break

        for nxt in neighbors(state, current):
            new_cost = cost_so_far[current] + traversal_cost(cost_map, nxt)

            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + heuristic(nxt, goal_cell)
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    if goal_cell not in came_from:
        return []

    path = []
    current = goal_cell
    while current is not None:
        path.append(current)
        current = came_from[current]

    path.reverse()
    return path


def update_cell_state_from_cost(state, cost_map, cell):
    """
    Actualiza el estado explícito de una celda según su situación actual.
    BLOCKED tiene prioridad sobre todo lo demás.
    """
    r, c = cell

    if GRID[r][c] != 0:
        state["cell_states"][cell] = BLOCKED
        return

    if cell in state["dynamic_blocked_cells"]:
        state["cell_states"][cell] = BLOCKED
        return

    # Si el coste base se ha incrementado, interpretamos congestión.
    if cost_map[r][c] > 1:
        state["cell_states"][cell] = CONGESTED
    else:
        state["cell_states"][cell] = FREE


def penalize_cell(state, cost_map, cell, penalty):
    """
    Incrementa el coste de una celda libre y actualiza su estado explícito.
    """
    r, c = cell
    if GRID[r][c] == 0 and cell not in state["dynamic_blocked_cells"]:
        cost_map[r][c] += penalty
        update_cell_state_from_cost(state, cost_map, cell)
        print(
            f"Celda {cell} penalizada. "
            f"Nuevo coste = {cost_map[r][c]} | Estado = {state['cell_states'][cell]}"
        )


def block_cell(state, cell):
    """
    Marca una celda como bloqueada dinámicamente.
    """
    r, c = cell

    if GRID[r][c] == 0:
        state["dynamic_blocked_cells"].add(cell)
        state["cell_states"][cell] = BLOCKED
        print(f"Celda {cell} BLOQUEADA dinámicamente. Estado = {state['cell_states'][cell]}")


def tentatively_unblock_cell(state, cost_map, cell):
    """
    Caduca un bloqueo dinámico: la celda vuelve a ser transitable pero con
    coste alto (TENTATIVE_BLOCK_COST). Si el LIDAR confirma que sigue
    bloqueada al acercarse, se re-bloquea automáticamente. Si está libre,
    el aprendizaje adaptativo irá bajando el coste con el tiempo.
    """
    r, c = cell
    if GRID[r][c] != 0:
        return
    old_cost = cost_map[r][c]
    state["dynamic_blocked_cells"].discard(cell)
    cost_map[r][c] = TENTATIVE_BLOCK_COST
    update_cell_state_from_cost(state, cost_map, cell)
    event_log.log_unblock(cell, old_cost, TENTATIVE_BLOCK_COST, cost_map)
    print(f"[DECAY] Bloqueo expirado en {cell} → CONGESTED coste={TENTATIVE_BLOCK_COST}")


def apply_dynamic_event(state, cost_map):
    """
    Simula un evento dinámico:
    - penaliza varias celdas (congestión)
    - bloquea nuevas celdas (bloqueo real)
    """
    if state["replan_already_done"]:
        return False

    print(f"[EVENTO] Cambio dinámico detectado al llegar a {TRIGGER_CELL}")

    for cell in PENALTY_TARGETS:
        penalize_cell(state, cost_map, cell, PENALTY_VALUE)

    for cell in NEW_BLOCKED_CELLS:
        block_cell(state, cell)

    state["replan_already_done"] = True
    return True