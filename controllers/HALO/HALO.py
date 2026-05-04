from controller import Robot
import event_log

from config import (
    LOCATIONS,
    GRID,
    FREE,
    BLOCKED,
    LEFT_MOTOR_CANDIDATES,
    RIGHT_MOTOR_CANDIDATES,
    LEFT_SENSOR_CANDIDATES,
    RIGHT_SENSOR_CANDIDATES,
    PROXIMITY_SENSOR_CANDIDATE_GROUPS,
    INITIAL_HEADING,
    create_task_queue,
    create_cost_map,
    TASK_SERVER_ENABLED,
    TASK_SERVER_HOST,
    TASK_SERVER_PORT,
    EXIT_ON_EMPTY_QUEUE,
)
from metrics import (
    create_task_metrics,
    create_cell_metrics,
    print_task_summary,
    print_cell_summary,
    print_cell_state_summary
)
from tasks import execute_task
from qlearning import create_q_table, print_q_summary, QL_CONFIG_STR

try:
    from task_server import start_task_server
except ImportError:
    start_task_server = None

# ============================================================
# ARRANQUE DE WEBOTS
# ============================================================

robot = Robot()
timestep = int(robot.getBasicTimeStep())
event_log.init()


def get_first_available_device(robot_instance, candidates, label):
    """Devuelve el primer dispositivo disponible entre varios nombres candidatos."""
    for name in candidates:
        try:
            device = robot_instance.getDevice(name)
            return device, name
        except BaseException:
            continue

    raise RuntimeError(f"No se encontro un dispositivo valido para: {label}. Candidatos: {candidates}")


def load_proximity_sensors(robot_instance, timestep_ms):
    """Carga el primer grupo de sensores de proximidad completamente disponible."""
    for candidate_group in PROXIMITY_SENSOR_CANDIDATE_GROUPS:
        loaded = []
        missing = False

        for name in candidate_group:
            try:
                sensor = robot_instance.getDevice(name)
                sensor.enable(timestep_ms)
                loaded.append(sensor)
            except BaseException:
                missing = True
                break

        if not missing:
            print(f"Sensores de proximidad cargados: {list(candidate_group)}")
            return loaded

    print("No se encontraron sensores de proximidad compatibles; se continua sin ellos.")
    return []

# Dispositivos del robot
left_motor, left_motor_name = get_first_available_device(robot, LEFT_MOTOR_CANDIDATES, "motor izquierdo")
right_motor, right_motor_name = get_first_available_device(robot, RIGHT_MOTOR_CANDIDATES, "motor derecho")
left_sensor, left_sensor_name = get_first_available_device(robot, LEFT_SENSOR_CANDIDATES, "sensor rueda izquierda")
right_sensor, right_sensor_name = get_first_available_device(robot, RIGHT_SENSOR_CANDIDATES, "sensor rueda derecha")

devices = {
    "left_motor": left_motor,
    "right_motor": right_motor,
    "left_sensor": left_sensor,
    "right_sensor": right_sensor,
}

print(
    f"Motores seleccionados: izq='{left_motor_name}', der='{right_motor_name}' | "
    f"Sensores rueda: izq='{left_sensor_name}', der='{right_sensor_name}'"
)

devices["left_sensor"].enable(timestep)
devices["right_sensor"].enable(timestep)

devices["left_motor"].setPosition(float('inf'))
devices["right_motor"].setPosition(float('inf'))
devices["left_motor"].setVelocity(0.0)
devices["right_motor"].setVelocity(0.0)

devices["prox_sensors"] = load_proximity_sensors(robot, timestep)

# GPS (añadido al robot Pioneer3DX como nodo extensionSlot)
try:
    gps_device = robot.getDevice("gps")
    gps_device.enable(timestep)
    devices["gps"] = gps_device
    print("GPS activado")
except Exception:
    devices["gps"] = None
    print("GPS no disponible, usando solo odometria")

# LIDAR (añadido al robot Pioneer3DX como nodo extensionSlot)
try:
    lidar_device = robot.getDevice("lidar")
    lidar_device.enable(timestep)
    devices["lidar"] = lidar_device
    print("LIDAR activado")
except Exception:
    devices["lidar"] = None
    print("LIDAR no disponible")

# ============================================================
# ESTADO GLOBAL
# ============================================================

# Estado explícito inicial de celdas
cell_states = {}
for r in range(len(GRID)):
    for c in range(len(GRID[0])):
        if GRID[r][c] == 0:
            cell_states[(r, c)] = FREE
        else:
            cell_states[(r, c)] = BLOCKED

state = {
    "heading": INITIAL_HEADING,
    "current_cell": (0, 0),
    "replan_already_done": False,
    "dynamic_blocked_cells": set(),
    "cell_states": cell_states,
    "block_ages": {},  # cell -> nº de celdas recorridas desde que se bloqueó
}

idle_return_pending = True

# Datos dinámicos
cost_map = create_cost_map()
q_table = create_q_table()
print(f"[Q-LEARN] Activo — {QL_CONFIG_STR}")
event_log.log_initial_cost_map(cost_map)
task_queue = create_task_queue()
if TASK_SERVER_ENABLED:
    if start_task_server is None:
        print("[WARN] Servidor de tareas no disponible. Instala fastapi y uvicorn para habilitarlo.")
    else:
        start_task_server(task_queue, host=TASK_SERVER_HOST, port=TASK_SERVER_PORT)
        print(f"Servidor de tareas activo en http://{TASK_SERVER_HOST}:{TASK_SERVER_PORT}")
task_metrics = create_task_metrics()
cell_metrics = create_cell_metrics()

# ============================================================
# EJECUCIÓN PRINCIPAL
# ============================================================

robot.step(timestep)

print("Costes iniciales:")
for row in cost_map:
    print(row)

# Posición inicial del robot = origen de la primera tarea
first_task = task_queue.peek_next()
if first_task is not None:
    if first_task.origin in LOCATIONS:
        state["current_cell"] = LOCATIONS[first_task.origin]
        print("Posición inicial del robot:", state["current_cell"])
else:
    state["current_cell"] = LOCATIONS["PHARMACY"]
    print("Posición inicial del robot (por defecto):", state["current_cell"])

exit_on_empty = EXIT_ON_EMPTY_QUEUE or not TASK_SERVER_ENABLED
if not exit_on_empty:
    print("Modo cola dinamica activo. Esperando nuevas tareas si la cola queda vacia.")

# Procesamiento de la cola de tareas
while True:
    task = task_queue.pop_next()
    if task is None:
        if not exit_on_empty:
            if task_queue.is_empty() and idle_return_pending:
                if state["current_cell"] != LOCATIONS["PHARMACY"]:
                    print("[IDLE] Sin tareas. Regresando a PHARMACY...")
                    success = execute_task(
                        robot,
                        timestep,
                        devices,
                        state,
                        cost_map,
                        task_metrics,
                        cell_metrics,
                        q_table,
                        "CURRENT",
                        "PHARMACY",
                    )
                    idle_return_pending = False
                    if not success:
                        print("[WARN] No se pudo regresar a PHARMACY en modo idle.")
                else:
                    idle_return_pending = False

        if exit_on_empty and task_queue.is_empty():
            break
        if robot.step(timestep) == -1:
            break
        continue

    try:
        success = execute_task(
            robot,
            timestep,
            devices,
            state,
            cost_map,
            task_metrics,
            cell_metrics,
            q_table,
            task.origin,
            task.destination
        )
    except Exception as exc:
        print(f"[ERROR] Tarea {task.origin}->{task.destination} fallo: {exc}")
        success = False

    task_queue.complete(task.task_id, success)
    idle_return_pending = True

    if not success:
        print(
            f"[WARN] Tarea {task.origin}->{task.destination} fallida, continuando con la siguiente."
        )

# ============================================================
# RESUMEN FINAL
# ============================================================

print("\nTodas las tareas procesadas o cola finalizada.")
print("Costes finales:")
for row in cost_map:
    print(row)

print_task_summary(task_metrics)
print_cell_summary(cell_metrics)
print_cell_state_summary(state["cell_states"])
print_q_summary(q_table)

event_log.flush()

while robot.step(timestep) != -1:
    pass