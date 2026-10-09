#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Measure the Thymio's angular speed (deg/s) when spinning in place.

The wheels are driven in opposite directions (left = -speed, right = +speed),
which makes the robot spin counter-clockwise (left). The Thymio has no gyroscope,
so the rotation is timed by hand:

  1. Put the robot on the floor and stick a piece of tape (or draw a line) in line
     with the robot's front.
  2. Run the script. The robot starts spinning.
  3. Press ENTER the moment the front passes the mark. The first ENTER starts
     the clock, every following ENTER closes one full 360 degree lap.
  4. After the requested number of laps the robot stops and the result is printed.

Usage
-----
    python test_max_angular_speed.py                 # 5 laps at max speed (500)
    python test_max_angular_speed.py --speed 250     # check linearity at half speed
    python test_max_angular_speed.py --laps 8 --port COM3
    python test_max_angular_speed.py --clockwise

Tips
----
- Use a fully charged battery: the maximum speed drops when the battery is low.
- Run it on the same floor surface used in the experiments (slip changes the result).
- Close Thymio Suite first, or the serial port will be busy.
"""

import argparse
import statistics
import time

from thymiodirect import Connection, Thymio

MAX_MOTOR = 500


def set_spin(robot, speed, clockwise=False):
    """Spin in place. Counter-clockwise by default (right wheel forward)."""
    sign = -1 if clockwise else 1
    robot["motor.left.target"] = int(-sign * speed)
    robot["motor.right.target"] = int(sign * speed)


def stop(robot):
    robot["motor.left.target"] = 0
    robot["motor.right.target"] = 0


def main():
    parser = argparse.ArgumentParser(description="Measure Thymio angular speed.")
    parser.add_argument(
        "--speed",
        type=int,
        default=MAX_MOTOR,
        help="wheel speed in motor units, 1-500 (default 500)",
    )
    parser.add_argument(
        "--laps", type=int, default=5, help="number of full turns to time (default 5)"
    )
    parser.add_argument("--port", default=None, help="serial port (default: auto)")
    parser.add_argument(
        "--clockwise",
        action="store_true",
        help="spin clockwise instead of counter-clockwise",
    )
    args = parser.parse_args()

    speed = max(1, min(MAX_MOTOR, args.speed))

    port = args.port or Connection.serial_default_port()
    th = Thymio(
        serial_port=port,
        on_connect=lambda node_id: print(f"Thymio {node_id} is connected"),
    )
    th.connect()
    robot = th[th.first_node()]
    time.sleep(1)  # let the connection settle

    print(f"\nSpin speed: +/-{speed}  |  laps: {args.laps}")
    print("Align the robot's front with the mark.")
    input("Press ENTER to start spinning... ")

    lap_times = []
    try:
        set_spin(robot, speed, args.clockwise)
        print(
            "Spinning. Press ENTER when the front passes the mark (starts the clock)."
        )
        input()
        t_prev = time.perf_counter()

        for i in range(1, args.laps + 1):
            input(f"  lap {i}/{args.laps}: press ENTER at the next pass... ")
            t_now = time.perf_counter()
            lap_times.append(t_now - t_prev)
            print(f"    lap time: {lap_times[-1]:.3f} s")
            t_prev = t_now
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop(robot)
        time.sleep(0.3)  # give thymiodirect time to send the stop command
        th.disconnect()

    if not lap_times:
        print("No laps recorded.")
        return

    mean_t = statistics.mean(lap_times)
    std_t = statistics.stdev(lap_times) if len(lap_times) > 1 else 0.0
    deg_s = 360.0 / mean_t

    print("\n================ RESULT ================")
    print(f"Laps timed          : {len(lap_times)}")
    print(f"Mean lap time       : {mean_t:.3f} s  (std {std_t:.3f} s)")
    print(f"Angular speed       : {deg_s:.1f} deg/s at wheel speed +/-{speed}")

    if speed != MAX_MOTOR:
        print(
            f"Extrapolated to 500 : {deg_s * MAX_MOTOR / speed:.1f} deg/s "
            "(assumes linearity; verify by running at 500)"
        )
        print(f"Deg/s per unit      : {deg_s / speed:.3f}")
    else:
        print(f"Deg/s per unit      : {deg_s / MAX_MOTOR:.3f}")
        print("\nUse in your code:")
        print(f"    MAX_ANGULAR_DEG_S = {deg_s:.1f}")
    print("========================================")


if __name__ == "__main__":
    main()
