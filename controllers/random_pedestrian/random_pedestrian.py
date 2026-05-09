import argparse
import math
import os
import random
import sys
from collections import deque

from controller import Supervisor

BASE_DIR = os.path.dirname(__file__)
HALO_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "HALO"))
if HALO_DIR not in sys.path:
    sys.path.insert(0, HALO_DIR)

from config import CELL_SIZE, cell_to_world, world_to_cell


GRID = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 0, y=13.00
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1],  # row 1, y=12.00
    [1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1],  # row 2, y=11.00
    [1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # row 3, y=10.00
    [1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # row 4, y=9.00
    [1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 1],  # row 5, y=8.00
    [1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # row 6, y=7.00
    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1],  # row 7, y=6.00
    [1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # row 8, y=5.00
    [1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # row 9, y=4.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1],  # row 10, y=3.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1],  # row 11, y=2.00
    [1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1],  # row 12, y=1.00
    [1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1],  # row 13, y=0.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # row 14, y=-1.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # row 15, y=-2.00
    [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # row 16, y=-3.00
    [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],  # row 17, y=-4.00
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1],  # row 18, y=-5.00
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 19, y=-6.00
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 20, y=-7.00
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 21, y=-8.00
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 22, y=-9.00
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 23, y=-10.00
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 24, y=-11.00
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 25, y=-12.00
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 26, y=-13.00
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # row 27, y=-14.00
]
#   [0   1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21]


JOINT_NAMES = [
    "leftArmAngle",
    "leftLowerArmAngle",
    "leftHandAngle",
    "rightArmAngle",
    "rightLowerArmAngle",
    "rightHandAngle",
    "leftLegAngle",
    "leftLowerLegAngle",
    "leftFootAngle",
    "rightLegAngle",
    "rightLowerLegAngle",
    "rightFootAngle",
    "headAngle",
]


def is_free(cell):
    row, col = cell
    return 0 <= row < len(GRID) and 0 <= col < len(GRID[0]) and GRID[row][col] == 0


def neighbors(cell):
    row, col = cell
    candidates = [(row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)]
    return [c for c in candidates if is_free(c)]


def nearest_free(start_cell):
    if is_free(start_cell):
        return start_cell

    queue = deque([start_cell])
    seen = {start_cell}

    while queue:
        cell = queue.popleft()
        row, col = cell
        candidates = [(row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)]
        for candidate in candidates:
            if candidate in seen:
                continue
            if is_free(candidate):
                return candidate
            if 0 <= candidate[0] < len(GRID) and 0 <= candidate[1] < len(GRID[0]):
                seen.add(candidate)
                queue.append(candidate)

    return start_cell


class RandomPedestrian(Supervisor):
    def __init__(self, args):
        super().__init__()

        if args.seed is not None:
            random.seed(args.seed)

        self.speed = max(0.1, args.speed)
        self.pause_min = max(0.0, args.pause_min)
        self.pause_max = max(self.pause_min, args.pause_max)
        self.avoid_backtrack = args.avoid_backtrack
        self.jitter = min(args.jitter, CELL_SIZE * 0.45)

        if args.step and args.step > 0:
            self.time_step = args.step
        else:
            self.time_step = int(self.getBasicTimeStep())
        self.time_step_s = self.time_step / 1000.0

        self.root_node = self.getSelf()
        self.translation_field = self.root_node.getField("translation")
        self.rotation_field = self.root_node.getField("rotation")
        self.joint_fields = {name: self.root_node.getField(name) for name in JOINT_NAMES}

        start_pos = self.translation_field.getSFVec3f()
        self.base_z = start_pos[2]

        start_cell = world_to_cell(start_pos[0], start_pos[1])
        self.current_cell = nearest_free(start_cell)
        self.prev_cell = None
        self.target_cell = None
        self.target_world = None
        self.pause_remaining = 0.0

    def pick_next_target(self):
        next_cells = neighbors(self.current_cell)

        if self.avoid_backtrack and self.prev_cell in next_cells and len(next_cells) > 1:
            next_cells = [cell for cell in next_cells if cell != self.prev_cell]

        if next_cells:
            self.target_cell = random.choice(next_cells)
        else:
            self.target_cell = self.current_cell

        target_x, target_y = cell_to_world(self.target_cell)
        if self.jitter > 0:
            target_x += random.uniform(-self.jitter, self.jitter)
            target_y += random.uniform(-self.jitter, self.jitter)

        self.target_world = (target_x, target_y)

    def apply_gait(self, time_s, moving):
        if not self.joint_fields.get("leftArmAngle"):
            return

        speed = self.speed if moving else max(0.2, self.speed * 0.3)
        stride_hz = max(0.5, speed * 0.8)
        phase = time_s * stride_hz * 2.0 * math.pi

        swing = math.sin(phase)
        lift = math.sin(phase + math.pi / 2.0)

        left_arm = 0.6 * swing
        right_arm = -left_arm
        left_forearm = 0.3 * lift
        right_forearm = -left_forearm
        left_hand = 0.15 * lift
        right_hand = -left_hand

        left_leg = -0.7 * swing
        right_leg = -left_leg
        left_lower_leg = 0.8 * lift
        right_lower_leg = -left_lower_leg
        left_foot = 0.25 * lift
        right_foot = -left_foot
        head = 0.1 * math.sin(phase * 0.5)

        self.joint_fields["leftArmAngle"].setSFFloat(left_arm)
        self.joint_fields["rightArmAngle"].setSFFloat(right_arm)
        self.joint_fields["leftLowerArmAngle"].setSFFloat(left_forearm)
        self.joint_fields["rightLowerArmAngle"].setSFFloat(right_forearm)
        self.joint_fields["leftHandAngle"].setSFFloat(left_hand)
        self.joint_fields["rightHandAngle"].setSFFloat(right_hand)

        self.joint_fields["leftLegAngle"].setSFFloat(left_leg)
        self.joint_fields["rightLegAngle"].setSFFloat(right_leg)
        self.joint_fields["leftLowerLegAngle"].setSFFloat(left_lower_leg)
        self.joint_fields["rightLowerLegAngle"].setSFFloat(right_lower_leg)
        self.joint_fields["leftFootAngle"].setSFFloat(left_foot)
        self.joint_fields["rightFootAngle"].setSFFloat(right_foot)
        self.joint_fields["headAngle"].setSFFloat(head)

    def set_pose(self, x, y, angle, time_s, moving):
        bob = 0.02 * math.sin(time_s * 2.0 * math.pi * max(0.5, self.speed * 0.6))
        z = self.base_z + (bob if moving else 0.0)
        self.translation_field.setSFVec3f([x, y, z])
        self.rotation_field.setSFRotation([0.0, 0.0, 1.0, angle])
        self.apply_gait(time_s, moving)

    def run(self):
        while self.step(self.time_step) != -1:
            time_s = self.getTime()

            if self.pause_remaining > 0.0:
                self.pause_remaining = max(0.0, self.pause_remaining - self.time_step_s)
                self.apply_gait(time_s, False)
                continue

            if self.target_cell is None:
                self.pick_next_target()

            pos = self.translation_field.getSFVec3f()
            target_x, target_y = self.target_world
            dx = target_x - pos[0]
            dy = target_y - pos[1]
            dist = math.hypot(dx, dy)

            if dist < 1e-3:
                self.prev_cell = self.current_cell
                self.current_cell = self.target_cell
                self.target_cell = None
                self.target_world = None
                if self.pause_max > 0.0:
                    self.pause_remaining = random.uniform(self.pause_min, self.pause_max)
                continue

            step = self.speed * self.time_step_s
            if dist <= step:
                new_x, new_y = target_x, target_y
            else:
                new_x = pos[0] + (dx / dist) * step
                new_y = pos[1] + (dy / dist) * step

            angle = math.atan2(dy, dx)
            self.set_pose(new_x, new_y, angle, time_s, True)


def build_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.3, help="Walking speed (m/s).")
    parser.add_argument("--step", type=int, default=0, help="Controller timestep (ms).")
    parser.add_argument("--pause-min", type=float, default=0.2, help="Min pause at cell (s).")
    parser.add_argument("--pause-max", type=float, default=1.0, help="Max pause at cell (s).")
    parser.add_argument("--jitter", type=float, default=0.12, help="Random offset inside the cell (m).")
    parser.add_argument("--avoid-backtrack", action="store_true", help="Avoid immediate backtracking.")
    parser.add_argument("--seed", type=int, default=None, help="Random seed.")
    return parser


def main():
    parser = build_arg_parser()
    args, _ = parser.parse_known_args()
    controller = RandomPedestrian(args)
    controller.run()


if __name__ == "__main__":
    main()
