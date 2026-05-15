from config import (LOCATIONS, TRIGGER_CELL, GRID, BLOCK_DECAY_TRAVERSALS, BLOCK_DECAY_SECONDS)
from navigation import (astar, direction_from_cells, traversal_cost,
                        apply_dynamic_event, passable, block_cell,
                        tentatively_unblock_cell)
from motion import rotate_to, move_one_cell, read_gps_cell, check_lidar_obstacle
from metrics import register_cell_traversal, append_task_metric
from qlearning import q_update, sync_cell_cost, QL_INIT_VALUE
import event_log

# Segundos de simulación que el robot espera cuando no hay ruta disponible,
# para dar tiempo a que los peatones despejen el pasillo.
WAIT_CLEAR_SECONDS = 2.0
MAX_WAIT_ATTEMPTS = 10


def _wait_sim_seconds(robot, timestep, seconds):
    """Avanza la simulación sin mover el robot durante el número de segundos indicado."""
    steps = max(1, int(seconds * 1000 / timestep))
    for _ in range(steps):
        if robot.step(timestep) == -1:
            return False
    return True


def _force_unblock_adjacent(state, cost_map, cell, lidar_recovered):
    """
    Libera tentativamente todos los bloqueos dinámicos adyacentes a cell.
    También los elimina de lidar_recovered para que puedan volver a
    intentar la recuperación normal si el obstáculo desaparece.
    """
    r, c = cell
    released = []
    for adj in [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]:
        if adj in state["dynamic_blocked_cells"]:
            tentatively_unblock_cell(state, cost_map, adj)
            state["block_ages"].pop(adj, None)
            state["block_times"].pop(adj, None)
            lidar_recovered.discard(adj)
            released.append(adj)
    return released


def _ql_update(q_table, cost_map, state, s, action, forward_time, s_prime):
    """
    Actualiza Q(s, action) con la ecuación de Bellman y sincroniza cost_map[s].
    Devuelve 1 si el coste de la celda cambió, 0 en caso contrario.
    """
    _DIRS = ["N", "E", "S", "O"]
    r, c = s
    if GRID[r][c] != 0 or s in state["dynamic_blocked_cells"]:
        return 0

    old_cost = cost_map[r][c]
    q_update(q_table, s, action, forward_time, s_prime)
    sync_cell_cost(q_table, cost_map, state, s)

    worst_q = min(q_table[r][c])
    new_cost = cost_map[r][c]
    deviation = abs(worst_q - QL_INIT_VALUE)

    if deviation > 0.05 or new_cost != old_cost:
        marker = "  ★ coste cambiado" if new_cost != old_cost else ""
        print(
            f"[Q-LEARN] ({r:2d},{c:2d}) ←{_DIRS[action]}  "
            f"t={forward_time:.3f}s  Q_worst={worst_q:8.3f}  coste={new_cost}{marker}"
        )

    return 1 if new_cost != old_cost else 0


def _decay_blocks(state, cost_map):
    """Envejece los bloqueos dinámicos y caduca los que superan el umbral de traversals."""
    for cell in list(state["dynamic_blocked_cells"]):
        age = state["block_ages"].get(cell, 0) + 1
        state["block_ages"][cell] = age
        if age >= BLOCK_DECAY_TRAVERSALS:
            tentatively_unblock_cell(state, cost_map, cell)
            state["block_ages"].pop(cell, None)
            state["block_times"].pop(cell, None)


def _time_decay_blocks(state, cost_map, current_time):
    """
    Caduca bloqueos LIDAR que llevan más de BLOCK_DECAY_SECONDS activos,
    independientemente del número de celdas recorridas.
    Permite que el robot replanifique aunque esté parado esperando.
    """
    for cell in list(state["dynamic_blocked_cells"]):
        block_time = state["block_times"].get(cell)
        if block_time is not None and current_time - block_time >= BLOCK_DECAY_SECONDS:
            print(f"[TIME-DECAY] Bloqueo en {cell} caducado por tiempo ({current_time - block_time:.1f}s)")
            tentatively_unblock_cell(state, cost_map, cell)
            state["block_ages"].pop(cell, None)
            state["block_times"].pop(cell, None)


def follow_route_with_replanning(robot, timestep, devices, state, cost_map, route, final_goal, cell_metrics, q_table):
    """
    Sigue una ruta celda a celda con tres niveles de robustez:
    1. Antes de cada movimiento: replanifica si la siguiente celda está bloqueada.
    2. Evento dinámico en TRIGGER_CELL: aplica cambios de mapa y replanifica.
    3. Después de cada movimiento: corrige la posición lógica con GPS y
       replanifica si el robot se ha desviado a una celda no adyacente a la ruta.

    Devuelve:
        (success, cells_traversed, replans, route_cost,
         wait_attempts, lidar_blocks, rotation_time_total, forward_time_total, q_cost_changes)
    """
    if not route or len(route) < 2:
        print("Ruta vacía o demasiado corta")
        return False, 0, 0, 0, 0, 0, 0.0, 0.0, 0

    local_cells_traversed = 0
    local_replans = 0
    local_route_cost = 0
    lidar_recovered = set()
    wait_attempts = 0
    prev_cell = None
    lidar_blocks = 0
    local_rotation_time = 0.0
    local_forward_time = 0.0
    local_q_changes = 0

    state["current_cell"] = route[0]
    print("Inicio en:", state["current_cell"])

    route_index = 1

    while route_index < len(route):
        next_cell = route[route_index]

        # --- 1. Replanning proactivo: celda del plan bloqueada antes de moverse ---
        if not passable(state, next_cell):
            print(f"[REPLAN] Celda {next_cell} bloqueada en ruta, replanificando desde {state['current_cell']}...")
            new_route = astar(state, cost_map, state["current_cell"], final_goal)
            local_replans += 1
            if not new_route:
                print("No se encontró ruta alternativa.")
                return False, local_cells_traversed, local_replans, local_route_cost, wait_attempts, lidar_blocks, local_rotation_time, local_forward_time, local_q_changes
            route = new_route
            route_index = 1
            state["current_cell"] = route[0]
            continue

        target_heading = direction_from_cells(state["current_cell"], next_cell)
        print(f"Ir de {state['current_cell']} a {next_cell}, orientación objetivo = {target_heading}")

        rotation_start_time = robot.getTime()
        rotate_to(robot, timestep, devices, state, target_heading)
        rotation_end_time = robot.getTime()
        rotation_time = rotation_end_time - rotation_start_time

        # --- LIDAR: obstáculo dinámico detectado tras girar hacia next_cell ---
        if next_cell != prev_cell and check_lidar_obstacle(devices):
            print(f"[LIDAR] Obstáculo frente a {next_cell}, bloqueando y replanificando...")
            lidar_blocks += 1
            old_cost = cost_map[next_cell[0]][next_cell[1]]
            block_cell(state, next_cell)
            cost_map[next_cell[0]][next_cell[1]] = 999
            state["block_ages"][next_cell] = 0
            state["block_times"][next_cell] = robot.getTime()
            event_log.log_block(next_cell, old_cost, cost_map, sim_time=robot.getTime())
            new_route = astar(state, cost_map, state["current_cell"], final_goal)
            local_replans += 1
            if not new_route:
                if next_cell not in lidar_recovered:
                    lidar_recovered.add(next_cell)
                    tentatively_unblock_cell(state, cost_map, next_cell)
                    state["block_ages"].pop(next_cell, None)
                    state["block_times"].pop(next_cell, None)
                    new_route = astar(state, cost_map, state["current_cell"], final_goal)
                    if new_route:
                        print(f"[RECOVERY] Ruta encontrada relajando bloqueo en {next_cell}")
                if not new_route and wait_attempts < MAX_WAIT_ATTEMPTS:
                    wait_attempts += 1
                    print(
                        f"[RECOVERY] Sin ruta. Esperando {WAIT_CLEAR_SECONDS}s "
                        f"(intento {wait_attempts}/{MAX_WAIT_ATTEMPTS})..."
                    )
                    if not _wait_sim_seconds(robot, timestep, WAIT_CLEAR_SECONDS):
                        return False, local_cells_traversed, local_replans, local_route_cost, wait_attempts, lidar_blocks, local_rotation_time, local_forward_time, local_q_changes
                    _time_decay_blocks(state, cost_map, robot.getTime())
                    released = _force_unblock_adjacent(state, cost_map, state["current_cell"], lidar_recovered)
                    if released:
                        print(f"[RECOVERY] Celdas liberadas tras espera: {released}")
                    new_route = astar(state, cost_map, state["current_cell"], final_goal)
                    if new_route:
                        print("[RECOVERY] Ruta encontrada tras espera")
                if not new_route:
                    print(f"[WARN] Sin ruta posible desde {state['current_cell']} a {final_goal}, abandonando tarea.")
                    return False, local_cells_traversed, local_replans, local_route_cost, wait_attempts, lidar_blocks, local_rotation_time, local_forward_time, local_q_changes
            route = new_route
            route_index = 1
            continue

        forward_start_time = robot.getTime()
        move_ok = move_one_cell(robot, timestep, devices, state=state, target_cell=next_cell, obstacle_check=True)
        forward_end_time = robot.getTime()
        forward_time = forward_end_time - forward_start_time

        # --- LIDAR: obstáculo detectado DURANTE el avance ---
        if not move_ok:
            print(f"[LIDAR] Obstáculo durante avance hacia {next_cell}, bloqueando y replanificando...")
            lidar_blocks += 1
            old_cost = cost_map[next_cell[0]][next_cell[1]]
            block_cell(state, next_cell)
            cost_map[next_cell[0]][next_cell[1]] = 999
            state["block_ages"][next_cell] = 0
            state["block_times"][next_cell] = robot.getTime()
            event_log.log_block(next_cell, old_cost, cost_map, sim_time=robot.getTime())

            gps_cell = read_gps_cell(devices)
            if gps_cell is not None and gps_cell not in state["dynamic_blocked_cells"]:
                state["current_cell"] = gps_cell
                print(f"[GPS] Posición tras parada de emergencia: {state['current_cell']}")
            else:
                print(f"[GPS] GPS indica celda bloqueada ({gps_cell}), manteniendo {state['current_cell']}")

            new_route = astar(state, cost_map, state["current_cell"], final_goal)
            local_replans += 1
            if not new_route:
                if next_cell not in lidar_recovered:
                    lidar_recovered.add(next_cell)
                    tentatively_unblock_cell(state, cost_map, next_cell)
                    state["block_ages"].pop(next_cell, None)
                    state["block_times"].pop(next_cell, None)
                    new_route = astar(state, cost_map, state["current_cell"], final_goal)
                    if new_route:
                        print(f"[RECOVERY] Ruta encontrada relajando bloqueo en {next_cell}")
                if not new_route and wait_attempts < MAX_WAIT_ATTEMPTS:
                    wait_attempts += 1
                    print(
                        f"[RECOVERY] Sin ruta. Esperando {WAIT_CLEAR_SECONDS}s "
                        f"(intento {wait_attempts}/{MAX_WAIT_ATTEMPTS})..."
                    )
                    if not _wait_sim_seconds(robot, timestep, WAIT_CLEAR_SECONDS):
                        return False, local_cells_traversed, local_replans, local_route_cost, wait_attempts, lidar_blocks, local_rotation_time, local_forward_time, local_q_changes
                    _time_decay_blocks(state, cost_map, robot.getTime())
                    released = _force_unblock_adjacent(state, cost_map, state["current_cell"], lidar_recovered)
                    if released:
                        print(f"[RECOVERY] Celdas liberadas tras espera: {released}")
                    new_route = astar(state, cost_map, state["current_cell"], final_goal)
                    if new_route:
                        print("[RECOVERY] Ruta encontrada tras espera")
                if not new_route:
                    print(f"[WARN] Sin ruta posible desde {state['current_cell']} a {final_goal}, abandonando tarea.")
                    return False, local_cells_traversed, local_replans, local_route_cost, wait_attempts, lidar_blocks, local_rotation_time, local_forward_time, local_q_changes
            route = new_route
            route_index = 1
            continue

        total_transition_time = rotation_time + forward_time
        local_rotation_time += rotation_time
        local_forward_time += forward_time

        prev_cell = state["current_cell"]
        state["current_cell"] = next_cell

        # --- 2. Corrección de posición con GPS ---
        gps_cell = read_gps_cell(devices)
        if gps_cell is not None and gps_cell != state["current_cell"]:
            print(f"[GPS] Posición corregida: odometría -> {state['current_cell']}, GPS -> {gps_cell}")
            state["current_cell"] = gps_cell
            if route_index + 1 < len(route):
                planned_next = route[route_index + 1]
                dr = abs(planned_next[0] - gps_cell[0])
                dc = abs(planned_next[1] - gps_cell[1])
                if dr + dc != 1:
                    new_route = astar(state, cost_map, state["current_cell"], final_goal)
                    local_replans += 1
                    if not new_route:
                        print("No se encontró ruta desde posición GPS corregida.")
                        return False, local_cells_traversed, local_replans, local_route_cost, wait_attempts, lidar_blocks, local_rotation_time, local_forward_time, local_q_changes
                    route = new_route
                    route_index = 1
                    print(f"[GPS] Ruta recalculada: {route}")
                    register_cell_traversal(state, cell_metrics, cost_map, state["current_cell"], rotation_time, forward_time)
                    local_cells_traversed += 1
                    local_route_cost += traversal_cost(cost_map, state["current_cell"])
                    continue

        print("Celda actual:", state["current_cell"])
        print(
            f"Tiempos en celda {state['current_cell']} -> "
            f"giro: {rotation_time:.3f}s | "
            f"avance: {forward_time:.3f}s | "
            f"total: {total_transition_time:.3f}s"
        )

        register_cell_traversal(state, cell_metrics, cost_map, state["current_cell"], rotation_time, forward_time)

        local_cells_traversed += 1
        local_route_cost += traversal_cost(cost_map, state["current_cell"])

        # --- Q-Learning: actualiza Q(celda_actual, dirección_entrada) y caducidad ---
        s_prime = route[route_index + 1] if route_index + 1 < len(route) else None
        if q_table is not None:
            local_q_changes += _ql_update(q_table, cost_map, state, state["current_cell"], target_heading, forward_time, s_prime)
        _decay_blocks(state, cost_map)
        _time_decay_blocks(state, cost_map, robot.getTime())

        # --- 3. Evento dinámico en TRIGGER_CELL ---
        if state["current_cell"] == TRIGGER_CELL and not state["replan_already_done"]:
            if apply_dynamic_event(state, cost_map):
                print("Costes tras el evento:")
                for row in cost_map:
                    print(row)

                print("Bloqueos dinámicos actuales:", sorted(state["dynamic_blocked_cells"]))

                new_route = astar(state, cost_map, state["current_cell"], final_goal)
                print("Nueva ruta recalculada desde la celda actual:", new_route)
                local_replans += 1

                if not new_route:
                    print("No se encontró ruta alternativa.")
                    return False, local_cells_traversed, local_replans, local_route_cost, wait_attempts, lidar_blocks, local_rotation_time, local_forward_time, local_q_changes

                route = new_route
                route_index = 1
                continue

        route_index += 1

    return True, local_cells_traversed, local_replans, local_route_cost, wait_attempts, lidar_blocks, local_rotation_time, local_forward_time, local_q_changes


def execute_task(robot, timestep, devices, state, cost_map, task_metrics, cell_metrics, q_table,
                 origin_name, destination_name):
    """
    Ejecuta una tarea completa:
    - ir al origen si hace falta
    - ir al destino
    - guardar métricas de la tarea
    """
    origin_from_current = False
    if origin_name == "CURRENT":
        origin = state["current_cell"]
        origin_from_current = True
    else:
        origin = LOCATIONS[origin_name]
    destination = LOCATIONS[destination_name]

    print("\n==============================")
    print(f"Nueva tarea: {origin_name} -> {destination_name}")
    print(f"Origen lógico: {origin} | Destino lógico: {destination}")

    task_start_time = robot.getTime()
    cells_traversed = 0
    replans = 0
    executed_route_cost = 0
    total_wait_attempts = 0
    total_lidar_blocks = 0
    total_rotation_time = 0.0
    total_forward_time = 0.0
    total_q_changes = 0
    initial_route_length = 0

    _time_decay_blocks(state, cost_map, robot.getTime())

    if not origin_from_current and state["current_cell"] != origin:
        route_to_origin = astar(state, cost_map, state["current_cell"], origin)
        if not route_to_origin:
            released = _force_unblock_adjacent(state, cost_map, state["current_cell"], set())
            if released:
                print(f"[RECOVERY] Celdas liberadas para ruta al origen: {released}")
                route_to_origin = astar(state, cost_map, state["current_cell"], origin)
        print("Ruta hasta el origen:", route_to_origin)

        if route_to_origin and len(route_to_origin) >= 2:
            initial_route_length += len(route_to_origin) - 1
            ok, partial_cells, partial_replans, partial_cost, partial_waits, partial_lidar, partial_rot, partial_fwd, partial_q = follow_route_with_replanning(
                robot, timestep, devices, state, cost_map, route_to_origin, origin, cell_metrics, q_table
            )
            cells_traversed += partial_cells
            replans += partial_replans
            executed_route_cost += partial_cost
            total_wait_attempts += partial_waits
            total_lidar_blocks += partial_lidar
            total_rotation_time += partial_rot
            total_forward_time += partial_fwd
            total_q_changes += partial_q

            if not ok:
                return False
        elif state["current_cell"] != origin:
            print("No se encontró ruta hasta el origen.")
            return False

    route_to_destination = astar(state, cost_map, state["current_cell"], destination)
    if not route_to_destination:
        released = _force_unblock_adjacent(state, cost_map, state["current_cell"], set())
        if released:
            print(f"[RECOVERY] Celdas liberadas para ruta al destino: {released}")
            route_to_destination = astar(state, cost_map, state["current_cell"], destination)
    print("Ruta de entrega:", route_to_destination)

    if not route_to_destination:
        print("No se encontró ruta al destino.")
        return False

    initial_route_length += max(0, len(route_to_destination) - 1)

    ok, partial_cells, partial_replans, partial_cost, partial_waits, partial_lidar, partial_rot, partial_fwd, partial_q = follow_route_with_replanning(
        robot, timestep, devices, state, cost_map, route_to_destination, destination, cell_metrics, q_table
    )
    cells_traversed += partial_cells
    replans += partial_replans
    executed_route_cost += partial_cost
    total_wait_attempts += partial_waits
    total_lidar_blocks += partial_lidar
    total_rotation_time += partial_rot
    total_forward_time += partial_fwd
    total_q_changes += partial_q

    task_end_time = robot.getTime()
    travel_time = task_end_time - task_start_time

    append_task_metric(
        task_metrics,
        origin_name,
        destination_name,
        origin,
        destination,
        travel_time,
        cells_traversed,
        replans,
        executed_route_cost,
        success=ok,
        wait_attempts=total_wait_attempts,
        lidar_blocks=total_lidar_blocks,
        initial_route_length=initial_route_length,
        rotation_time_total=total_rotation_time,
        forward_time_total=total_forward_time,
        q_cost_changes=total_q_changes,
    )

    if ok:
        print(f"Tarea completada: {origin_name} -> {destination_name}")
        print(
            f"Tiempo: {travel_time:.3f}s | "
            f"Celdas: {cells_traversed} | "
            f"Replans: {replans} | "
            f"Coste: {executed_route_cost} | "
            f"Esperas: {total_wait_attempts} | "
            f"Bloqueos LIDAR: {total_lidar_blocks} | "
            f"Cambios Q: {total_q_changes}"
        )

    return ok
