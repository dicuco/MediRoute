import math
from config import (WHEEL_RADIUS, AXLE_LENGTH, FORWARD_SPEED,
                    TURN_SPEED, CELL_SIZE,
                    SLOW_SPEED_FACTOR, CONGESTED_CELLS,
                    FORWARD_TIMEOUT_FACTOR, TURN_TIMEOUT_FACTOR,
                    STRAIGHT_CORRECTION_GAIN, TURN_BRAKE_RATIO,
                    CONGESTED, world_to_cell, cell_to_world)

# Ganancia del término GPS lateral en move_forward.
# Complementa al STRAIGHT_CORRECTION_GAIN basado en encoders.
GPS_LATERAL_GAIN = 4.0
# Distancia al destino GPS (m) para dar el movimiento por completado.
GPS_STOP_THRESHOLD = 0.06


def stop(devices):
    """Detiene el robot."""
    devices["left_motor"].setVelocity(0.0)
    devices["right_motor"].setVelocity(0.0)


def clamp(value, low, high):
    """Limita un valor al rango [low, high]."""
    return max(low, min(high, value))


def get_wheel_distances(devices):
    """
    Convierte la lectura angular de las ruedas en distancia lineal recorrida.
    """
    left_dist = devices["left_sensor"].getValue() * WHEEL_RADIUS
    right_dist = devices["right_sensor"].getValue() * WHEEL_RADIUS
    return left_dist, right_dist


def move_forward(robot, timestep, devices, distance_m, slow=False,
                 gps_target=None, heading=None):
    """
    Avanza una distancia concreta en metros.
    Control híbrido: corrección por encoders + corrección lateral GPS (si disponible).
    Parada por distancia GPS cuando está a menos de GPS_STOP_THRESHOLD del objetivo.
    """
    speed = FORWARD_SPEED * SLOW_SPEED_FACTOR if slow else FORWARD_SPEED
    start_left, start_right = get_wheel_distances(devices)

    nominal_linear_speed = max(abs(speed) * WHEEL_RADIUS, 1e-6)
    expected_time = distance_m / nominal_linear_speed
    timeout_time = robot.getTime() + expected_time * FORWARD_TIMEOUT_FACTOR + 0.5

    max_wheel_speed = max(abs(speed) * 1.6, abs(speed) + 0.4)

    gps = devices.get("gps")
    use_gps = (gps is not None and gps_target is not None and heading is not None)
    if use_gps:
        target_x, target_y = gps_target

    devices["left_motor"].setVelocity(speed)
    devices["right_motor"].setVelocity(speed)

    while robot.step(timestep) != -1:
        current_left, current_right = get_wheel_distances(devices)
        delta_left = abs(current_left - start_left)
        delta_right = abs(current_right - start_right)
        avg_distance = (delta_left + delta_right) / 2.0

        encoder_error = delta_left - delta_right
        correction = STRAIGHT_CORRECTION_GAIN * encoder_error

        if use_gps:
            vals = gps.getValues()
            gps_x, gps_y = vals[0], vals[1]
            dx = target_x - gps_x
            dy = target_y - gps_y

            # Signos derivados del convenio left=speed-correction, right=speed+correction:
            # correction>0 → rueda derecha más rápida → giro a la IZQUIERDA (CCW).
            # N(0,+y): eje lateral=X. Robot desviado a la derecha → dx<0 → necesita giro IZQ → lateral_error=-dx
            # E(1,+x): eje lateral=Y. Robot desviado arriba    → dy<0 → necesita giro DER → lateral_error=dy
            # S(2,-y): eje lateral=X. Robot desviado a la izq  → dx>0 → necesita giro IZQ → lateral_error=dx
            # W(3,-x): eje lateral=Y. Robot desviado arriba    → dy<0 → necesita giro IZQ → lateral_error=-dy
            if heading == 0:
                lateral_error = -dx
            elif heading == 1:
                lateral_error = dy
            elif heading == 2:
                lateral_error = dx
            else:
                lateral_error = -dy

            correction += GPS_LATERAL_GAIN * lateral_error

            if avg_distance >= distance_m * 0.7:
                gps_dist = math.sqrt(dx * dx + dy * dy)
                if gps_dist <= GPS_STOP_THRESHOLD:
                    break

        left_cmd = clamp(speed - correction, -max_wheel_speed, max_wheel_speed)
        right_cmd = clamp(speed + correction, -max_wheel_speed, max_wheel_speed)
        devices["left_motor"].setVelocity(left_cmd)
        devices["right_motor"].setVelocity(right_cmd)

        if avg_distance >= distance_m:
            break

        if robot.getTime() >= timeout_time:
            print(
                f"[WARN] Timeout en avance: objetivo={distance_m:.3f}m, "
                f"recorrido={avg_distance:.3f}m"
            )
            break

    stop(devices)


def turn_in_place(robot, timestep, devices, angle_deg):
    """
    Gira sobre sí mismo el ángulo especificado.
    Positivo = izquierda, negativo = derecha.
    """
    angle_rad = math.radians(angle_deg)
    wheel_travel = (AXLE_LENGTH / 2.0) * abs(angle_rad)

    start_left, start_right = get_wheel_distances(devices)

    nominal_angular_wheel_speed = max(abs(TURN_SPEED) * WHEEL_RADIUS, 1e-6)
    expected_time = wheel_travel / nominal_angular_wheel_speed
    timeout_time = robot.getTime() + expected_time * TURN_TIMEOUT_FACTOR + 0.5

    left_sign = -1 if angle_deg > 0 else 1
    right_sign = 1 if angle_deg > 0 else -1

    devices["left_motor"].setVelocity(left_sign * TURN_SPEED)
    devices["right_motor"].setVelocity(right_sign * TURN_SPEED)

    while robot.step(timestep) != -1:
        current_left, current_right = get_wheel_distances(devices)

        delta_left = abs(current_left - start_left)
        delta_right = abs(current_right - start_right)

        progress = max(delta_left, delta_right) / max(wheel_travel, 1e-6)
        if progress > 0.7:
            turn_cmd = max(TURN_SPEED * TURN_BRAKE_RATIO, 0.25)
            devices["left_motor"].setVelocity(left_sign * turn_cmd)
            devices["right_motor"].setVelocity(right_sign * turn_cmd)

        if delta_left >= wheel_travel and delta_right >= wheel_travel * 0.9:
            break
        if delta_right >= wheel_travel and delta_left >= wheel_travel * 0.9:
            break

        if robot.getTime() >= timeout_time:
            print(
                f"[WARN] Timeout en giro: objetivo={abs(angle_deg):.1f}deg, "
                f"progreso ruedas=({delta_left:.3f},{delta_right:.3f})m"
            )
            break

    stop(devices)


def turn_left(robot, timestep, devices, state):
    """Gira 90 grados a la izquierda y actualiza la orientación lógica."""
    turn_in_place(robot, timestep, devices, 90)
    state["heading"] = (state["heading"] - 1) % 4
    print("Nueva orientación:", state["heading"])


def turn_right(robot, timestep, devices, state):
    """Gira 90 grados a la derecha y actualiza la orientación lógica."""
    turn_in_place(robot, timestep, devices, -90)
    state["heading"] = (state["heading"] + 1) % 4
    print("Nueva orientación:", state["heading"])


def turn_around(robot, timestep, devices, state):
    """Gira 180 grados y actualiza la orientación lógica."""
    turn_in_place(robot, timestep, devices, 180)
    state["heading"] = (state["heading"] + 2) % 4
    print("Nueva orientación:", state["heading"])


def read_gps_cell(devices):
    """Lee el GPS y devuelve la celda lógica actual, o None si el GPS no está disponible."""
    gps = devices.get("gps")
    if gps is None:
        return None
    try:
        values = gps.getValues()
        # En este mundo Webots: x=values[0], y=values[1], z=values[2] (altura ~0.0975)
        return world_to_cell(values[0], values[1])
    except Exception:
        return None


def move_one_cell(robot, timestep, devices, state=None, target_cell=None):
    """
    Avanza exactamente una celda lógica.
    Usa el estado dinámico de celdas para detectar congestión; cae al
    conjunto estático si no se proporciona estado.
    """
    slow = False
    if target_cell is not None:
        if state is not None:
            slow = state["cell_states"].get(target_cell) == CONGESTED
        else:
            slow = target_cell in CONGESTED_CELLS
    if slow:
        print(f"  Congestion en {target_cell}, reduciendo velocidad")
    gps_target = cell_to_world(target_cell) if target_cell is not None else None
    heading = state["heading"] if state is not None else None
    move_forward(robot, timestep, devices, CELL_SIZE, slow=slow,
                 gps_target=gps_target, heading=heading)
    print("Avanzada una celda")


def rotate_to(robot, timestep, devices, state, target_heading):
    """
    Rota el robot hasta que mire en la orientación lógica deseada.
    """
    diff = (target_heading - state["heading"]) % 4

    if diff == 0:
        return
    elif diff == 1:
        turn_right(robot, timestep, devices, state)
    elif diff == 2:
        turn_around(robot, timestep, devices, state)
    elif diff == 3:
        turn_left(robot, timestep, devices, state)