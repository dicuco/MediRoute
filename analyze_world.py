"""
Parsea HOSPITAL-MEDIROUTE.wbt y genera la rejilla GRID para config.py.

1. Extrae paredes, posición del robot y arena del archivo .wbt
2. Rasteriza las paredes en una rejilla fina (0.02 m)
3. Flood-fill desde la posición del robot para detectar celdas alcanzables
4. Agrega a una rejilla gruesa (0.25 m) alineada con las paredes
5. Imprime GRID, posición del robot y centros de fila/columna

Uso:
    cd MEDIROUTE
    python analyze_world.py
"""
import math
from collections import deque
from pathlib import Path

candidate_paths = [
    Path("worlds/HOSPITAL-MEDIROUTE.wbt"),
    Path("MEDIROUTE/worlds/HOSPITAL-MEDIROUTE.wbt"),
]

wbt_path = None
for path in candidate_paths:
    if path.exists():
        wbt_path = path
        break

if wbt_path is None:
    raise FileNotFoundError("No se encontro HOSPITAL-MEDIROUTE.wbt en rutas conocidas.")

print(f"Analizando mundo: {wbt_path}")

with open(wbt_path, 'r') as f:
    lines = f.read().split('\n')

walls = []
robot_pos = None
robot_proto = None
arena_info = {}

i = 0
while i < len(lines):
    stripped = lines[i].strip()

    if stripped.startswith('E-puck') or stripped.startswith('Pioneer3dx'):
        robot_proto = stripped.split()[0]
        i += 1
        while i < len(lines) and '}' not in lines[i]:
            line = lines[i].strip()
            if line.startswith('translation'):
                parts = line.split()
                robot_pos = (float(parts[1]), float(parts[2]))
            i += 1

    elif stripped.startswith('RectangleArena'):
        i += 1
        while i < len(lines) and '}' not in lines[i]:
            line = lines[i].strip()
            if line.startswith('translation'):
                parts = line.split()
                arena_info['tx'] = float(parts[1])
                arena_info['ty'] = float(parts[2])
            elif line.startswith('floorSize'):
                parts = line.split()
                arena_info['sx'] = float(parts[1])
                arena_info['sy'] = float(parts[2])
            i += 1

    elif stripped.startswith('Wall'):
        wall = {}
        i += 1
        while i < len(lines) and '}' not in lines[i]:
            line = lines[i].strip()
            if line.startswith('translation'):
                parts = line.split()
                wall['tx'] = float(parts[1])
                wall['ty'] = float(parts[2])
            elif line.startswith('rotation'):
                parts = line.split()
                wall['angle'] = float(parts[4])
            elif line.startswith('size'):
                parts = line.split()
                wall['sx'] = float(parts[1])
                wall['sy'] = float(parts[2])
                wall['sz'] = float(parts[3])
            i += 1
        if 'tx' in wall and 'sx' in wall:
            if 'angle' not in wall:
                wall['angle'] = 0.0
            walls.append(wall)

    i += 1

print(f"Parsed {len(walls)} walls")
print(f"Robot proto: {robot_proto}")
print(f"Robot at: {robot_pos}")

if robot_pos is None:
    raise RuntimeError("No se encontro la posicion del robot en el .wbt")

# --- Fine grid rasterization ---
FINE = 0.02
arena_cx = arena_info.get('tx', 0.05)
arena_cy = arena_info.get('ty', 0)
arena_w = arena_info.get('sx', 8)
arena_h = arena_info.get('sy', 8)

x_min_a = arena_cx - arena_w / 2
x_max_a = arena_cx + arena_w / 2
y_min_a = arena_cy - arena_h / 2
y_max_a = arena_cy + arena_h / 2

nx = int(round((x_max_a - x_min_a) / FINE))
ny = int(round((y_max_a - y_min_a) / FINE))

print(f"Fine grid: {nx} x {ny}")

grid_fine = bytearray(nx * ny)

MIN_THICKNESS = 0.08

for w in walls:
    tx, ty = w['tx'], w['ty']
    angle = w['angle']
    half_x = max(w['sx'], MIN_THICKNESS) / 2
    half_y = max(w['sy'], MIN_THICKNESS) / 2

    corners_local = [
        (-half_x, -half_y),
        (half_x, -half_y),
        (half_x, half_y),
        (-half_x, half_y),
    ]

    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    wxs, wys = [], []
    for lx, ly in corners_local:
        wx = tx + lx * cos_a - ly * sin_a
        wy = ty + lx * sin_a + ly * cos_a
        wxs.append(wx)
        wys.append(wy)

    bb_x_min = min(wxs)
    bb_x_max = max(wxs)
    bb_y_min = min(wys)
    bb_y_max = max(wys)

    ci_min = max(0, int((bb_x_min - x_min_a) / FINE))
    ci_max = min(nx - 1, int((bb_x_max - x_min_a) / FINE))
    ri_min = max(0, int((y_max_a - bb_y_max) / FINE))
    ri_max = min(ny - 1, int((y_max_a - bb_y_min) / FINE))

    for ri in range(ri_min, ri_max + 1):
        for ci in range(ci_min, ci_max + 1):
            grid_fine[ri * nx + ci] = 1

# --- Flood fill from robot ---
robot_x, robot_y = robot_pos
robot_ci = int((robot_x - x_min_a) / FINE)
robot_ri = int((y_max_a - robot_y) / FINE)

robot_ci = max(0, min(nx - 1, robot_ci))
robot_ri = max(0, min(ny - 1, robot_ri))

print(f"Robot in fine grid: row={robot_ri}, col={robot_ci}, wall={grid_fine[robot_ri * nx + robot_ci]}")

if grid_fine[robot_ri * nx + robot_ci]:
    print("Robot on wall, searching nearby...")
    found = False
    for d in range(1, 20):
        for dr in range(-d, d + 1):
            for dc in range(-d, d + 1):
                nr, nc = robot_ri + dr, robot_ci + dc
                if 0 <= nr < ny and 0 <= nc < nx and not grid_fine[nr * nx + nc]:
                    robot_ri, robot_ci = nr, nc
                    found = True
                    break
            if found:
                break
        if found:
            break

visited = bytearray(nx * ny)
visited[robot_ri * nx + robot_ci] = 1
queue = deque([(robot_ri, robot_ci)])

while queue:
    r, c = queue.popleft()
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < ny and 0 <= nc < nx:
            idx = nr * nx + nc
            if not visited[idx] and not grid_fine[idx]:
                visited[idx] = 1
                queue.append((nr, nc))

# --- Coarsen to 0.25m grid (shifted so walls are at cell centers) ---
map_scale = arena_w / 8.0
CELL = 0.25 * map_scale

# Grid aligned so cell centers match wall positions (0.25m multiples from arena center)
# Cell boundaries are offset by 0.125m from cell centers
base_hosp_x_min = -2.325   # first cell center at -2.200
base_hosp_x_max = 3.175    # last cell center at  3.050
base_hosp_y_min = -3.625   # last cell center at -3.500
base_hosp_y_max = 3.375    # first cell center at 3.250

hosp_x_min = base_hosp_x_min * map_scale
hosp_x_max = base_hosp_x_max * map_scale
hosp_y_min = base_hosp_y_min * map_scale
hosp_y_max = base_hosp_y_max * map_scale

print(f"Map scale detectado: {map_scale:.3f}x")

n_cols = int(round((hosp_x_max - hosp_x_min) / CELL))
n_rows = int(round((hosp_y_max - hosp_y_min) / CELL))

print(f"\nCoarse grid ({CELL:.3f}m, shifted): {n_rows} rows x {n_cols} cols")

coarse_grid = []
for r in range(n_rows):
    row = []
    for c in range(n_cols):
        cx_min = hosp_x_min + c * CELL
        cx_max = cx_min + CELL
        cy_max = hosp_y_max - r * CELL
        cy_min = cy_max - CELL

        total = 0
        reachable = 0
        wall_count = 0

        fi_min = max(0, int((cx_min - x_min_a) / FINE))
        fi_max = min(nx - 1, int((cx_max - x_min_a) / FINE))
        fj_min = max(0, int((y_max_a - cy_max) / FINE))
        fj_max = min(ny - 1, int((y_max_a - cy_min) / FINE))

        for fj in range(fj_min, fj_max + 1):
            for fi in range(fi_min, fi_max + 1):
                total += 1
                if grid_fine[fj * nx + fi]:
                    wall_count += 1
                if visited[fj * nx + fi]:
                    reachable += 1

        wall_frac = wall_count / total if total > 0 else 0
        reach_frac = reachable / total if total > 0 else 0

        if wall_frac > 0.05:
            row.append(1)
        elif reach_frac > 0.3:
            row.append(0)
        else:
            row.append(1)

    coarse_grid.append(row)

# --- Print results ---
print("\nGRID = [")
for r, row_data in enumerate(coarse_grid):
    cy = hosp_y_max - r * CELL - CELL / 2
    print(f"    {row_data},  # row {r}, y={cy:.2f}")
print("]")

robot_c_col = int((robot_x - hosp_x_min) / CELL)
robot_c_row = int((hosp_y_max - robot_y) / CELL)
print(f"\nRobot coarse position: row={robot_c_row}, col={robot_c_col}")

print("\nColumn X centers:")
for c in range(n_cols):
    cx = hosp_x_min + c * CELL + CELL / 2
    print(f"  col {c:2d}: x = {cx:.3f}")

print("\nRow Y centers:")
for r in range(n_rows):
    cy = hosp_y_max - r * CELL - CELL / 2
    print(f"  row {r:2d}: y = {cy:.3f}")

# Count free cells and identify corners
free_cells = [(r, c) for r in range(n_rows) for c in range(n_cols) if coarse_grid[r][c] == 0]
print(f"\nFree cells: {len(free_cells)}")

if free_cells:
    min_r = min(rc[0] for rc in free_cells)
    max_r = max(rc[0] for rc in free_cells)
    min_c = min(rc[1] for rc in free_cells)
    max_c = max(rc[1] for rc in free_cells)
    print(f"Bounding box: rows [{min_r}, {max_r}], cols [{min_c}, {max_c}]")

    for r, c in free_cells:
        cx = hosp_x_min + c * CELL + CELL / 2
        cy = hosp_y_max - r * CELL - CELL / 2
        if (r in [min_r, max_r] or c in [min_c, max_c]):
            if (r <= min_r + 1 or r >= max_r - 1) and (c <= min_c + 1 or c >= max_c + 1):
                print(f"  corner ({r}, {c}) -> world ({cx:.2f}, {cy:.2f})")
