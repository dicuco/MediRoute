import os

from task_queue import TaskQueue

# ============================================================
# CONSTANTES DEL ROBOT Y DEL MAPA
# ============================================================

ROBOT_MODEL = os.getenv("HALO_ROBOT_MODEL", "pioneer3dx").lower()
#Como el primer mapa que hicimos era para un robot demasiado pequeño, para reutilizar todo
# el diseño de ese mapa sin tener que redibujarlo, lo que hicimos fue escalarlo 4x.
MAP_SCALE = float(os.getenv("HALO_MAP_SCALE", "4.0"))

ROBOT_PROFILES = {
    "pioneer3dx": {
        "wheel_radius": 0.0975,
        "axle_length": 0.33,
        "forward_speed": 2.5,
        "turn_speed": 2.0,
        "left_motor_candidates": ("left wheel", "left wheel motor"),
        "right_motor_candidates": ("right wheel", "right wheel motor"),
        "left_sensor_candidates": ("left wheel sensor",),
        "right_sensor_candidates": ("right wheel sensor",),
        "prox_sensor_candidate_groups": (
            tuple(f"so{i}" for i in range(16)),
            tuple(f"ps{i}" for i in range(8)),
        ),
    },
}

if ROBOT_MODEL not in ROBOT_PROFILES:
    raise ValueError(
        f"ROBOT_MODEL no valido: {ROBOT_MODEL}. Opciones: {', '.join(ROBOT_PROFILES.keys())}"
    )

ROBOT = ROBOT_PROFILES[ROBOT_MODEL]

WHEEL_RADIUS = ROBOT["wheel_radius"]
AXLE_LENGTH = ROBOT["axle_length"]

FORWARD_SPEED = ROBOT["forward_speed"]
TURN_SPEED = ROBOT["turn_speed"]

LEFT_MOTOR_CANDIDATES = ROBOT["left_motor_candidates"]
RIGHT_MOTOR_CANDIDATES = ROBOT["right_motor_candidates"]
LEFT_SENSOR_CANDIDATES = ROBOT["left_sensor_candidates"]
RIGHT_SENSOR_CANDIDATES = ROBOT["right_sensor_candidates"]
PROXIMITY_SENSOR_CANDIDATE_GROUPS = ROBOT["prox_sensor_candidate_groups"]

SLOW_SPEED_FACTOR = 0.2

# Parametros de robustez para control por odometria.
FORWARD_TIMEOUT_FACTOR = float(os.getenv("HALO_FORWARD_TIMEOUT_FACTOR", "2.5"))
TURN_TIMEOUT_FACTOR = float(os.getenv("HALO_TURN_TIMEOUT_FACTOR", "2.8"))
STRAIGHT_CORRECTION_GAIN = float(os.getenv("HALO_STRAIGHT_CORRECTION_GAIN", "6.0"))
TURN_BRAKE_RATIO = float(os.getenv("HALO_TURN_BRAKE_RATIO", "0.45"))

# Heading inicial logico (0=norte, 1=este, 2=sur, 3=oeste).
INITIAL_HEADING = int(os.getenv("HALO_INITIAL_HEADING", "1")) % 4

# Servidor de tareas en tiempo real (REST + WebSocket)
TASK_SERVER_ENABLED = os.getenv("HALO_TASK_SERVER_ENABLED", "1") == "1"
TASK_SERVER_HOST = os.getenv("HALO_TASK_SERVER_HOST", "127.0.0.1")
TASK_SERVER_PORT = int(os.getenv("HALO_TASK_SERVER_PORT", "8000"))
EXIT_ON_EMPTY_QUEUE = os.getenv("HALO_EXIT_ON_EMPTY_QUEUE", "0") == "1"

#CONGESTED_CELLS = {(13, 7), (14, 7), (15, 7), (16, 7)}
CONGESTED_CELLS={}

# Tamaño lógico de una celda (centro a centro).
# El mapa grande actual es un escalado 4x del mapa original.
BASE_CELL_SIZE = 0.25
CELL_SIZE = BASE_CELL_SIZE * MAP_SCALE

BASE_GRID_ORIGIN_X = -2.2
BASE_GRID_ORIGIN_Y = 3.25
GRID_ORIGIN_X = BASE_GRID_ORIGIN_X * MAP_SCALE
GRID_ORIGIN_Y = BASE_GRID_ORIGIN_Y * MAP_SCALE

# Tiempo esperado SOLO para el avance de una celda
NOMINAL_LINEAR_SPEED = WHEEL_RADIUS * FORWARD_SPEED
EXPECTED_FORWARD_CELL_TIME = (CELL_SIZE / NOMINAL_LINEAR_SPEED) * 1.15

# Penalización automática por retraso real de avance
AUTO_DELAY_PENALTY = 1
AUTO_DELAY_THRESHOLD_RATIO = float(os.getenv("HALO_AUTO_DELAY_THRESHOLD_RATIO", "1.2"))

# Aprendizaje adaptativo de costes
# Celdas totales recorridas antes de que un bloqueo LIDAR caduque y el robot
# lo intente de nuevo con coste alto (passable, pero caro).
BLOCK_DECAY_TRAVERSALS = 15
# Coste asignado a una celda cuando su bloqueo LIDAR caduca.
TENTATIVE_BLOCK_COST = 25

# ============================================================
# MAPA DEL HOSPITAL  (28 filas × 22 columnas)
# ============================================================
# Rejilla alineada con las paredes del mundo HOSPITAL-MEDIROUTE.wbt.
# Los centros de celda se calculan con:
#   x(col) = GRID_ORIGIN_X + col * CELL_SIZE
#   y(row) = GRID_ORIGIN_Y - row * CELL_SIZE
# Con MAP_SCALE=4.0 (mapa grande):
#   col  0 -> x = -8.80  ...  col 21 -> x = 12.20
#   row  0 -> y = 13.00  ...  row 27 -> y = -14.00
#
# 0 = libre,  1 = bloqueado (pared / fuera del hospital)

GRID = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 0, y=13.00
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1],  # row 1, y=12.00
    [1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1],  # row 2, y=11.00
    [1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1],  # row 3, y=10.00
    [1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1],  # row 4, y=9.00
    [1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1],  # row 5, y=8.00
    [1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # row 6, y=7.00
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1],  # row 7, y=6.00
    [1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # row 8, y=5.00
    [1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1],  # row 9, y=4.00
    [1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1],  # row 10, y=3.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1],  # row 11, y=2.00
    [1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1],  # row 12, y=1.00
    [1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1],  # row 13, y=0.00
    [1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1],  # row 14, y=-1.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1],  # row 15, y=-2.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # row 16, y=-3.00
    [1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],  # row 17, y=-4.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1],  # row 18, y=-5.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1],  # row 19, y=-6.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1],  # row 20, y=-7.00
    [1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1],  # row 21, y=-8.00
    [1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 22, y=-9.00
    [1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 23, y=-10.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 24, y=-11.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 25, y=-12.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 26, y=-13.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 27, y=-14.00
]
#   [0   1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21]

# ============================================================
# UBICACIONES LÓGICAS DEL HOSPITAL
# ============================================================

LOCATIONS = {
    # Marcadores temporales: cajas de carton del mundo HOSPITAL-MEDIROUTE.
    # Calculadas con world_to_cell(x, y) para MAP_SCALE=4.0.
    "Toilet": (13, 5),   # Toilet
    "Single_Room_1": (8, 4),    # Individual Room 1
    "Double_Room_1": (6, 4),    # Doble Room 1 (Arriba Izquierda)
    "TV_Room": (11, 9),   # TV Room
    "Kitchen": (10, 15),  # Kitchen
    "Dining_Room": (13, 9),   # Dining Room
    "Waiting_Room": (13, 15),  # Waiting Room
    "Double_Room_5": (22, 6),   # Doble Room 5 (abajo izquierda)
    "Double_Room_2": (3, 10),   # Doble Room 2 (arriba centro)
    "Double_Room_3": (3, 14),   # Dooble Room 3 (arriba derecha)
    "Single_Room_2": (10, 18),  # Individual Room 2
    "Double_Room_4": (18, 16),  # Doble Room 4 (abajo derecha)
    "PHARMACY": (27, 7),      # Punto inicial del robot y origen de la primera tarea.
}

# ============================================================
# COLA INICIAL DE TAREAS
# ============================================================
# INITIAL_TASKS = [
# ]

INITIAL_TASKS = [
    ("PHARMACY", "Dining_Room"),
    ("Dining_Room", "PHARMACY"),
    ("PHARMACY", "Single_Room_1"),
    ("Single_Room_1", "PHARMACY"),
    ("PHARMACY", "Single_Room_1"),
    ("Single_Room_1", "PHARMACY"),
    ("PHARMACY", "Double_Room_2"),
    ("Double_Room_2", "PHARMACY"),
    ("PHARMACY", "Double_Room_3"),
    ("Double_Room_3", "PHARMACY"),
    ("PHARMACY", "Double_Room_4"),
    ("Double_Room_4", "PHARMACY"),
]
# INITIAL_TASKS = [
#     ("PHARMACY", "Double_Room_1"),
#     ("Double_Room_1","Single_Room_2"),
#     ("Single_Room_2", "PHARMACY"),
#     ]

# ============================================================
# EVENTO DINÁMICO DE EJEMPLO
# ============================================================

# Al llegar a TRIGGER_CELL se penalizan varias celdas y se bloquea una.
# El robot se ve obligado a replanificar su ruta.
TRIGGER_CELL = (27, 21)
PENALTY_TARGETS = []
PENALTY_VALUE = 50

NEW_BLOCKED_CELLS = []


def cell_to_world(cell):
    """Convierte celda logica (row, col) a coordenadas (x, y) del mundo."""
    row, col = cell
    x = GRID_ORIGIN_X + col * CELL_SIZE
    y = GRID_ORIGIN_Y - row * CELL_SIZE
    return x, y


def world_to_cell(x, y):
    """Convierte coordenadas (x, y) del mundo a celda logica (row, col)."""
    col = int(round((x - GRID_ORIGIN_X) / CELL_SIZE))
    row = int(round((GRID_ORIGIN_Y - y) / CELL_SIZE))
    return row, col


def create_task_queue():
    """Devuelve una cola de tareas con prioridad."""
    return TaskQueue(INITIAL_TASKS)


def create_cost_map():
    """Devuelve una copia nueva del mapa de costes."""
    return [[999 if cell == 1 else 1 for cell in row] for row in GRID]

# ============================================================
# ESTADOS EXPLÍCITOS DE CELDA
# ============================================================

FREE = "FREE"
CONGESTED = "CONGESTED"
BLOCKED = "BLOCKED"