import csv
import os
from collections import defaultdict
from config import EXPECTED_FORWARD_CELL_TIME, AUTO_DELAY_PENALTY, AUTO_DELAY_THRESHOLD_RATIO
from navigation import penalize_cell

CSV_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "validation_metrics.csv")


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

    delay_threshold = EXPECTED_FORWARD_CELL_TIME * AUTO_DELAY_THRESHOLD_RATIO
    if forward_time > delay_threshold:
        cell_metrics[cell]["delays"] += 1
        if AUTO_DELAY_PENALTY > 0:
            cell_metrics[cell]["auto_penalties"] += 1
            penalize_cell(state, cost_map, cell, AUTO_DELAY_PENALTY)
            print(
                f"[AUTO] Retraso detectado en {cell}: "
                f"avance {forward_time:.3f}s > {delay_threshold:.3f}s"
            )


def append_task_metric(task_metrics, origin_name, destination_name, origin, destination,
                       travel_time, cells_traversed, replans, route_cost,
                       success=True, wait_attempts=0, lidar_blocks=0,
                       initial_route_length=0, rotation_time_total=0.0,
                       forward_time_total=0.0, q_cost_changes=0):
    route_overhead = round(cells_traversed / max(1, initial_route_length), 3)
    task_metrics.append({
        "task": f"{origin_name}->{destination_name}",
        "origin": str(origin),
        "destination": str(destination),
        "success": success,
        "travel_time": round(travel_time, 3),
        "cells_traversed": cells_traversed,
        "initial_route_length": initial_route_length,
        "route_overhead": route_overhead,
        "replans": replans,
        "wait_attempts": wait_attempts,
        "lidar_blocks": lidar_blocks,
        "route_cost": route_cost,
        "rotation_time_total": round(rotation_time_total, 3),
        "forward_time_total": round(forward_time_total, 3),
        "q_cost_changes": q_cost_changes,
    })


def print_task_summary(task_metrics):
    print("\n=== RESUMEN DE MÉTRICAS POR TAREA ===")
    total_time = 0
    total_cells = 0
    total_replans = 0
    total_cost = 0
    total_waits = 0
    total_lidar = 0
    total_q_changes = 0
    success_count = 0

    for metric in task_metrics:
        print(metric)
        total_time += metric["travel_time"]
        total_cells += metric["cells_traversed"]
        total_replans += metric["replans"]
        total_cost += metric["route_cost"]
        total_waits += metric.get("wait_attempts", 0)
        total_lidar += metric.get("lidar_blocks", 0)
        total_q_changes += metric.get("q_cost_changes", 0)
        if metric.get("success", True):
            success_count += 1

    n = len(task_metrics)
    print("\n=== TOTALES ===")
    print(f"Tareas completadas: {success_count}/{n}")
    print(f"Tiempo total: {total_time:.3f}s")
    print(f"Celdas recorridas: {total_cells}")
    print(f"Replanificaciones: {total_replans}")
    print(f"Coste acumulado: {total_cost}")
    print(f"Esperas (wait_attempts): {total_waits}")
    print(f"Bloqueos LIDAR detectados: {total_lidar}")
    print(f"Cambios de coste por Q-Learning: {total_q_changes}")

    if n > 0:
        print("\n=== PROMEDIOS ===")
        print(f"Tiempo medio por tarea: {total_time / n:.3f}s")
        print(f"Celdas medias por tarea: {total_cells / n:.2f}")
        print(f"Replans medios por tarea: {total_replans / n:.2f}")
        print(f"Bloqueos LIDAR medios por tarea: {total_lidar / n:.2f}")
        print(f"Cambios Q medios por tarea: {total_q_changes / n:.2f}")


def save_metrics_csv(task_metrics):
    """
    Guarda todas las métricas por tarea en un CSV con filas de TOTAL y PROMEDIO al final.
    El fichero se crea en el mismo directorio que este módulo.
    """
    if not task_metrics:
        print("[METRICS] Sin datos para guardar en CSV.")
        return

    fieldnames = list(task_metrics[0].keys())
    numeric_keys = [
        "travel_time", "cells_traversed", "initial_route_length",
        "replans", "wait_attempts", "lidar_blocks", "route_cost",
        "rotation_time_total", "forward_time_total", "q_cost_changes",
    ]
    n = len(task_metrics)

    with open(CSV_FILE, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(task_metrics)

        # Fila de totales
        totals = {k: "" for k in fieldnames}
        totals["task"] = "TOTAL"
        totals["success"] = f"{sum(1 for m in task_metrics if m.get('success', True))}/{n}"
        for key in numeric_keys:
            totals[key] = round(sum(m.get(key, 0) for m in task_metrics), 3)
        writer.writerow(totals)

        # Fila de promedios
        avgs = {k: "" for k in fieldnames}
        avgs["task"] = "PROMEDIO"
        avgs["success"] = f"{sum(1 for m in task_metrics if m.get('success', True)) / n:.1%}"
        for key in numeric_keys:
            avgs[key] = round(sum(m.get(key, 0) for m in task_metrics) / n, 3)
        writer.writerow(avgs)

    print(f"[METRICS] CSV de validación guardado en: {CSV_FILE}")


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
