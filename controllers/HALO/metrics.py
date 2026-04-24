from collections import defaultdict
from config import EXPECTED_FORWARD_CELL_TIME, AUTO_DELAY_PENALTY
from navigation import penalize_cell


def create_task_metrics():
    """Lista vacía para métricas por tarea."""
    return []


def create_cell_metrics():
    """Diccionario para métricas por celda."""
    return defaultdict(lambda: {
        "visits": 0,
        "total_time": 0.0,
        "avg_time": 0.0,
        "rotation_total_time": 0.0,
        "rotation_avg_time": 0.0,
        "forward_total_time": 0.0,
        "forward_avg_time": 0.0,
        "delays": 0,
        "auto_penalties": 0
    })


def register_cell_traversal(state, cell_metrics, cost_map, cell, rotation_time, forward_time):
    """
    Guarda estadísticas de paso por celda separando:
    - tiempo de giro
    - tiempo de avance
    - tiempo total
    """
    total_time = rotation_time + forward_time

    cell_metrics[cell]["visits"] += 1
    cell_metrics[cell]["total_time"] += total_time
    cell_metrics[cell]["rotation_total_time"] += rotation_time
    cell_metrics[cell]["forward_total_time"] += forward_time

    visits = cell_metrics[cell]["visits"]

    cell_metrics[cell]["avg_time"] = cell_metrics[cell]["total_time"] / visits
    cell_metrics[cell]["rotation_avg_time"] = cell_metrics[cell]["rotation_total_time"] / visits
    cell_metrics[cell]["forward_avg_time"] = cell_metrics[cell]["forward_total_time"] / visits

    if forward_time > EXPECTED_FORWARD_CELL_TIME:
        cell_metrics[cell]["delays"] += 1
        cell_metrics[cell]["auto_penalties"] += 1
        penalize_cell(state, cost_map, cell, AUTO_DELAY_PENALTY)
        print(
            f"[AUTO] Retraso detectado en {cell}: "
            f"avance {forward_time:.3f}s > {EXPECTED_FORWARD_CELL_TIME:.3f}s"
        )


def append_task_metric(task_metrics, origin_name, destination_name, origin, destination,
                       travel_time, cells_traversed, replans, route_cost):
    task_metrics.append({
        "task": f"{origin_name}->{destination_name}",
        "origin": origin,
        "destination": destination,
        "travel_time": round(travel_time, 3),
        "cells_traversed": cells_traversed,
        "replans": replans,
        "route_cost": route_cost
    })


def print_task_summary(task_metrics):
    print("\n=== RESUMEN DE MÉTRICAS POR TAREA ===")
    total_time = 0
    total_cells = 0
    total_replans = 0
    total_cost = 0

    for metric in task_metrics:
        print(metric)
        total_time += metric["travel_time"]
        total_cells += metric["cells_traversed"]
        total_replans += metric["replans"]
        total_cost += metric["route_cost"]

    print("\n=== TOTALES ===")
    print(f"Tiempo total: {total_time:.3f}s")
    print(f"Celdas recorridas: {total_cells}")
    print(f"Replanificaciones: {total_replans}")
    print(f"Coste acumulado: {total_cost}")

    if len(task_metrics) > 0:
        print("\n=== PROMEDIOS ===")
        print(f"Tiempo medio por tarea: {total_time / len(task_metrics):.3f}s")
        print(f"Celdas medias por tarea: {total_cells / len(task_metrics):.2f}")
        print(f"Replans medios por tarea: {total_replans / len(task_metrics):.2f}")


def print_cell_summary(cell_metrics):
    print("\n=== MÉTRICAS POR CELDA ===")
    for cell, data in sorted(cell_metrics.items()):
        print(
            f"Celda {cell} | "
            f"Visitas: {data['visits']} | "
            f"Giro total: {data['rotation_total_time']:.3f}s | "
            f"Giro medio: {data['rotation_avg_time']:.3f}s | "
            f"Avance total: {data['forward_total_time']:.3f}s | "
            f"Avance medio: {data['forward_avg_time']:.3f}s | "
            f"Tiempo total: {data['total_time']:.3f}s | "
            f"Tiempo medio total: {data['avg_time']:.3f}s | "
            f"Retrasos de avance: {data['delays']} | "
            f"Auto-penalizaciones: {data['auto_penalties']}"
        )


def print_cell_state_summary(cell_states):
    print("\n=== ESTADO EXPLÍCITO DE LAS CELDAS ===")
    for cell, state in sorted(cell_states.items()):
        print(f"Celda {cell} -> {state}")