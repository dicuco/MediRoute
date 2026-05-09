from config import GRID, EXPECTED_FORWARD_CELL_TIME

# ============================================================
# HIPERPARÁMETROS
# ============================================================

QL_ALPHA = 0.3          # Tasa de aprendizaje
QL_GAMMA = 0.0          # Factor de descuento (0 = EMA pura del reward inmediato, sin propagación futura)
QL_COST_SCALE = 5       # Escala para convertir desviación Q → coste A*
QL_MAX_COST = 30        # Coste máximo que puede asignar Q-Learning

# Q-valor inicial: con γ=0 el valor esperado en condiciones ideales (reward=-1) es -1.0.
# Fórmula general: -1/(1-γ); con γ=0 → -1.0.
QL_INIT_VALUE = -1.0 / (1.0 - QL_GAMMA)   # -1.0

QL_CONFIG_STR = (
    f"α={QL_ALPHA}  γ={QL_GAMMA}  "
    f"Q_init={QL_INIT_VALUE:.1f}  escala_coste={QL_COST_SCALE}  coste_max={QL_MAX_COST}"
)


# ============================================================
# TABLA Q
# ============================================================

def create_q_table():
    """
    Crea la tabla Q[fila][columna][acción] inicializada al valor ideal.
    Acciones: 0=Norte  1=Este  2=Sur  3=Oeste
    """
    rows = len(GRID)
    cols = len(GRID[0])
    return [[[QL_INIT_VALUE] * 4 for _ in range(cols)] for _ in range(rows)]


# ============================================================
# ACTUALIZACIÓN BELLMAN
# ============================================================

def q_update(q_table, s, action, forward_time, s_prime):
    """
    Aplica la ecuación de Bellman a Q(s, action):

      Q(s,a) ← Q(s,a) + α · [r + γ · max_a' Q(s', a') − Q(s, a)]

    Parámetros
    ----------
    s           : celda que acaba de traversarse  (row, col)
    action      : dirección con la que se entró a s  (0=N 1=E 2=S 3=O)
    forward_time: tiempo real de avance hasta s  (segundos)
    s_prime     : siguiente celda planificada, o None si s es el destino final
    """
    r, c = s

    # Recompensa normalizada: -1.0 en condiciones ideales, más negativa con retraso
    reward = -(forward_time / EXPECTED_FORWARD_CELL_TIME)

    # max Q(s', a')  →  0 en estado terminal (no hay más celdas)
    if s_prime is not None:
        rp, cp = s_prime
        best_next = max(q_table[rp][cp])
    else:
        best_next = 0.0

    q_old = q_table[r][c][action]
    q_table[r][c][action] = q_old + QL_ALPHA * (reward + QL_GAMMA * best_next - q_old)


# ============================================================
# CONVERSIÓN Q → COSTE PARA A*
# ============================================================

def q_to_cost(q_values):
    """
    Convierte los Q-valores de una celda a un coste escalar para A*.

    Usa min(Q) — la dirección más lenta — como base del coste. Esto evita que
    direcciones no visitadas (en QL_INIT_VALUE) enmascaren una dirección
    congestionada con max(Q).

    Lógica:
      - Celda ideal o no visitada  →  min(Q) = QL_INIT_VALUE (-10) → coste = 1
      - Celda lenta en alguna dir  →  min(Q) < QL_INIT_VALUE        → coste > 1

    Fórmula:
      desviación = -min(Q) − |QL_INIT_VALUE|   (0 para ideal, >0 para peor)
      coste = max(1, min(QL_MAX_COST, round(desviación × QL_COST_SCALE + 1)))

    Ejemplos con γ=0, escala=5 (celda aislada):
      traversal 1×  →  Q* = −1   →  desv=0   →  coste=1
      traversal 2×  →  Q* = −2   →  desv=1   →  coste=6
      traversal 3×  →  Q* = −3   →  desv=2   →  coste=11
      traversal 5×  →  Q* = −5   →  desv=4   →  coste=21
    """
    worst_q = min(q_values)          # dirección más penalizada (más negativa)
    deviation = -worst_q - abs(QL_INIT_VALUE)
    return max(1, min(QL_MAX_COST, round(deviation * QL_COST_SCALE + 1)))


# ============================================================
# SINCRONIZACIÓN Q → cost_map
# ============================================================

def sync_cell_cost(q_table, cost_map, state, cell):
    """
    Escribe en cost_map[cell] el coste derivado de la Q-tabla.
    No actúa sobre paredes ni celdas bloqueadas dinámicamente.
    """
    from navigation import update_cell_state_from_cost

    r, c = cell
    if GRID[r][c] != 0 or cell in state["dynamic_blocked_cells"]:
        return

    cost_map[r][c] = q_to_cost(q_table[r][c])
    update_cell_state_from_cost(state, cost_map, cell)


# ============================================================
# RESUMEN
# ============================================================

def print_q_summary(q_table):
    """
    Imprime el resumen final de Q-Learning:
    - Tabla de celdas que aprendieron algo (desviación >= 0.5 del valor inicial).
    - Mapa de calor ASCII con los costes finales aprendidos.
    """
    DIRS = ["N", "E", "S", "O"]
    THRESHOLD = 0.5

    # --- Tabla de celdas con aprendizaje ---
    print(f"\n{'='*60}")
    print(f"  RESUMEN Q-LEARNING  |  {QL_CONFIG_STR}")
    print(f"{'='*60}")

    learned = []
    for r in range(len(GRID)):
        for c in range(len(GRID[0])):
            if GRID[r][c] != 0:
                continue
            qs = q_table[r][c]
            worst = min(qs)          # dirección más congestionada
            deviation = abs(worst - QL_INIT_VALUE)
            if deviation >= THRESHOLD:
                learned.append((deviation, r, c, qs))

    if not learned:
        print("  (sin aprendizaje significativo todavía)")
    else:
        learned.sort(reverse=True)   # de mayor a menor desviación
        print(f"  {'Celda':8}  {'Q[N,E,S,O]':36}  {'Peor dir':8}  {'Coste':5}  {'Desv':5}")
        print(f"  {'-'*70}")
        for deviation, r, c, qs in learned:
            cost = q_to_cost(qs)
            worst_dir = DIRS[qs.index(min(qs))]   # dirección más penalizada
            q_str = "  ".join(f"{q:6.2f}" for q in qs)
            print(
                f"  ({r:2d},{c:2d})   [{q_str}]   ←{worst_dir}         {cost:3d}    {deviation:4.2f}"
            )
        print(f"\n  Total celdas con aprendizaje: {len(learned)}")

    # --- Mapa de calor ASCII del cost_map aprendido ---
    print(f"\n  Mapa de costes aprendidos (. = libre ideal, número = coste > 1, # = pared):")
    for r in range(len(GRID)):
        row_str = "  "
        for c in range(len(GRID[0])):
            if GRID[r][c] != 0:
                row_str += " #"
            else:
                cost = q_to_cost(q_table[r][c])
                row_str += " ." if cost == 1 else f"{cost:2d}"
        print(row_str)
    print(f"{'='*60}")
