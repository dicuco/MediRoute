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
)
from metrics import (
    create_task_metrics,
    create_cell_metrics,
    print_task_summary,
    print_cell_summary,
    print_cell_state_summary
)
from tasks import execute_task

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

# Datos dinámicos
cost_map = create_cost_map()
task_queue = create_task_queue()
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
if task_queue:
    first_origin, _ = task_queue[0]
    state["current_cell"] = LOCATIONS[first_origin]
    print("Posición inicial del robot:", state["current_cell"])

# Procesamiento de la cola de tareas
while task_queue:
    origin_name, destination_name = task_queue.popleft()
    success = execute_task(
        robot,
        timestep,
        devices,
        state,
        cost_map,
        task_metrics,
        cell_metrics,
        origin_name,
        destination_name
    )

    if not success:
        print(f"[WARN] Tarea {origin_name}->{destination_name} fallida, continuando con la siguiente.")

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

event_log.flush()

while robot.step(timestep) != -1:
    pass