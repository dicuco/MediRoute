from config import LOCATIONS, TRIGGER_CELL
from navigation import astar, direction_from_cells, traversal_cost, apply_dynamic_event, passable
from motion import rotate_to, move_one_cell, read_gps_cell
from metrics import register_cell_traversal, append_task_metric


def follow_route_with_replanning(robot, timestep, devices, state, cost_map, route, final_goal, cell_metrics):
    """
    Sigue una ruta celda a celda con tres niveles de robustez:
    1. Antes de cada movimiento: replanifica si la siguiente celda está bloqueada.
    2. Evento dinámico en TRIGGER_CELL: aplica cambios de mapa y replanifica.
    3. Después de cada movimiento: corrige la posición lógica con GPS y
       replanifica si el robot se ha desviado a una celda no adyacente a la ruta.
    """
    if not route or len(route) < 2:
        print("Ruta vacía o demasiado corta")
        return False, 0, 0, 0

    local_cells_traversed = 0
    local_replans = 0
    local_route_cost = 0

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
                return False, local_cells_traversed, local_replans, local_route_cost
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

        forward_start_time = robot.getTime()
        move_one_cell(robot, timestep, devices, state=state, target_cell=next_cell)
        forward_end_time = robot.getTime()
        forward_time = forward_end_time - forward_start_time

        total_transition_time = rotation_time + forward_time

        state["current_cell"] = next_cell

        # --- 2. Corrección de posición con GPS ---
        gps_cell = read_gps_cell(devices)
        if gps_cell is not None and gps_cell != state["current_cell"]:
            print(f"[GPS] Posición corregida: odometría -> {state['current_cell']}, GPS -> {gps_cell}")
            state["current_cell"] = gps_cell
            # Solo replanifica si el siguiente paso ya no es adyacente a la posición real
            if route_index + 1 < len(route):
                planned_next = route[route_index + 1]
                dr = abs(planned_next[0] - gps_cell[0])
                dc = abs(planned_next[1] - gps_cell[1])
                if dr + dc != 1:
                    new_route = astar(state, cost_map, state["current_cell"], final_goal)
                    local_replans += 1
                    if not new_route:
                        print("No se encontró ruta desde posición GPS corregida.")
                        return False, local_cells_traversed, local_replans, local_route_cost
                    route = new_route
                    route_index = 1
                    print(f"[GPS] Ruta recalculada: {route}")
                    # Registra la celda antes de continuar con la nueva ruta
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
                    return False, local_cells_traversed, local_replans, local_route_cost

                route = new_route
                route_index = 1
                continue

        route_index += 1

    return True, local_cells_traversed, local_replans, local_route_cost


def execute_task(robot, timestep, devices, state, cost_map, task_metrics, cell_metrics,
                 origin_name, destination_name):
    """
    Ejecuta una tarea completa:
    - ir al origen si hace falta
    - ir al destino
    - guardar métricas de la tarea
    """
    origin = LOCATIONS[origin_name]
    destination = LOCATIONS[destination_name]

    print("\n==============================")
    print(f"Nueva tarea: {origin_name} -> {destination_name}")
    print(f"Origen lógico: {origin} | Destino lógico: {destination}")

    task_start_time = robot.getTime()
    cells_traversed = 0
    replans = 0
    executed_route_cost = 0

    if state["current_cell"] != origin:
        route_to_origin = astar(state, cost_map, state["current_cell"], origin)
        print("Ruta hasta el origen:", route_to_origin)

        if route_to_origin and len(route_to_origin) >= 2:
            ok, partial_cells, partial_replans, partial_cost = follow_route_with_replanning(
                robot, timestep, devices, state, cost_map, route_to_origin, origin, cell_metrics
            )
            cells_traversed += partial_cells
            replans += partial_replans
            executed_route_cost += partial_cost

            if not ok:
                return False
        elif state["current_cell"] != origin:
            print("No se encontró ruta hasta el origen.")
            return False

    route_to_destination = astar(state, cost_map, state["current_cell"], destination)
    print("Ruta de entrega:", route_to_destination)

    if not route_to_destination:
        print("No se encontró ruta al destino.")
        return False

    ok, partial_cells, partial_replans, partial_cost = follow_route_with_replanning(
        robot, timestep, devices, state, cost_map, route_to_destination, destination, cell_metrics
    )
    cells_traversed += partial_cells
    replans += partial_replans
    executed_route_cost += partial_cost

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
        executed_route_cost
    )

    if ok:
        print(f"Tarea completada: {origin_name} -> {destination_name}")
        print(
            f"Tiempo: {travel_time:.3f}s | "
            f"Celdas: {cells_traversed} | "
            f"Replans: {replans} | "
            f"Coste: {executed_route_cost}"
        )

    return ok