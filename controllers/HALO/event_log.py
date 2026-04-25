"""
Log de eventos de actualización del mapa de costes.
Escribe en cost_map_events.txt y agrupa entradas repetidas consecutivas.
"""
import os
from datetime import datetime

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cost_map_events.txt")

_last_key = None    # (event_type, cell) del último evento escrito
_repeat_count = 0   # veces que ese evento se ha repetido sin escribir


def _ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def _write(line):
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(line + "\n")


def _write_cost_map(cost_map, highlight_cell=None):
    """
    Escribe el mapa de costes completo.
    La celda modificada se muestra entre corchetes [999].
    Celdas libres (coste=1) se muestran como '.' para reducir ruido visual.
    Cada celda ocupa 5 caracteres para alineación uniforme.
    """
    _write("  Mapa de costes actualizado:")
    for r, row in enumerate(cost_map):
        parts = []
        for c, val in enumerate(row):
            cell_str = "." if val == 1 else str(val)
            if (r, c) == highlight_cell:
                parts.append(f"[{cell_str:>3s}]")   # [999] o [ 20] — 5 chars
            else:
                parts.append(f"{cell_str:>5s}")      # "    ." o "  999" — 5 chars
        _write(f"  r{r:2d}:{''.join(parts)}")
    _write("")


def _flush_repeat():
    """Si el último evento se repitió, escribe el contador y limpia el estado."""
    global _last_key, _repeat_count
    if _last_key is not None and _repeat_count > 1:
        _write(f"  ↳ repetido {_repeat_count}x en total\n")
    _last_key = None
    _repeat_count = 0


def init():
    """Crea/sobreescribe el fichero de log y escribe la cabecera."""
    with open(LOG_FILE, "w", encoding="utf-8") as f:
        f.write(
            f"=== LIDAR/DECAY cost-map events — "
            f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n\n"
        )


def log_block(cell, old_cost, cost_map, sim_time=None):
    """
    Registra que el LIDAR bloqueó una celda (coste → 999) y vuelca el mapa.
    Si el evento anterior era el mismo bloqueo en la misma celda,
    solo incrementa el contador en lugar de escribir una línea nueva.
    """
    global _last_key, _repeat_count
    key = ("BLOCK", cell)
    if key == _last_key:
        _repeat_count += 1
        return
    _flush_repeat()
    t = f" [sim={sim_time:.1f}s]" if sim_time is not None else ""
    _write(f"[{_ts()}]{t}  BLOQUEO  celda={cell}  coste {old_cost}→999  (LIDAR)")
    _write_cost_map(cost_map, highlight_cell=cell)
    _last_key = key
    _repeat_count = 1


def log_unblock(cell, old_cost, new_cost, cost_map, sim_time=None):
    """
    Registra que el decay liberó una celda bloqueada (coste → new_cost) y vuelca el mapa.
    Si el evento anterior era el mismo para la misma celda, solo cuenta.
    """
    global _last_key, _repeat_count
    key = ("UNBLOCK", cell)
    if key == _last_key:
        _repeat_count += 1
        return
    _flush_repeat()
    t = f" [sim={sim_time:.1f}s]" if sim_time is not None else ""
    _write(f"[{_ts()}]{t}  LIBERADO celda={cell}  coste {old_cost}→{new_cost}  (DECAY)")
    _write_cost_map(cost_map, highlight_cell=cell)
    _last_key = key
    _repeat_count = 1


def flush():
    """Escribe cualquier repetición pendiente. Llamar al terminar la simulación."""
    _flush_repeat()
