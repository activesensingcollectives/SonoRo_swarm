#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created: 2025-12-17
Author: Alberto Doimo
email: alberto.doimo@uni-konstanz.de

"""

from thymiodirect import Connection
from thymiodirect import Thymio
from shared_queues import (
    angle_queue,
    level_queue,
    angle_queue_led,
    level_queue_led,
)
from utilities import degs_to_speed, speed_to_degs
import time
import random
from SoundLEDController import SoundLEDController


class ThymioNodeProxy:
    """Wrap a thymiodirect node so that variable writes are only sent when the value changes.

    With thymiodirect every write is a serial message to the Thymio, while reads come
    from a local cache refreshed in the background. Re-sending unchanged motor/LED values
    from the control loops floods the serial link and delays the commands that matter.
    """

    def __init__(self, node):
        self._node = node
        self._sent = {}

    def __getitem__(self, name):
        return self._node[name]

    def __setitem__(self, name, value):
        key = tuple(value) if isinstance(value, (list, tuple)) else value
        if self._sent.get(name) == key:
            return
        self._node[name] = value
        self._sent[name] = key

    def invalidate(self):
        """Forget the last sent values so the next writes are always sent."""
        self._sent.clear()


def clear_queue(q):
    """Safely and thread-safely clear all items from a Queue/LifoQueue."""
    try:
        with q.mutex:
            q.queue.clear()
            if hasattr(q, "all_tasks_done"):
                q.all_tasks_done.notify_all()
            if hasattr(q, "not_full"):
                q.not_full.notify_all()
    except Exception:
        while not q.empty():
            try:
                q.get_nowait()
            except Exception:
                break


class RobotMove:
    """This class manages robot motor control, LED indicators, ground sensors,
    and implements various behavioral modes

    Parameters
    ----------
    forward_speed : int
        Target speed for forward motor movement.
    behaviour_angular_speed : float
        Angular speed (deg/s) for audio-driven (behaviour) turns.
    left_sensor_threshold : int
        Threshold value for left ground sensor to detect  lines.
    right_sensor_threshold : int
        Threshold value for right ground sensor to detect  lines.
    trigger_level : float
        Sound pressure level (dB SPL) threshold to trigger detection.
    critical_level : float
        Critical sound pressure level (dB SPL) for avoiding collisions.
    ground_sensors_bool : bool, optional
        If True, initializes and monitors ground sensor values at start.
    arena_angular_speed : float, optional
        Angular speed (deg/s) for line avoidance and disentangling turns.
    attraction_angle_range : float, optional
        Half-width (deg) around the front within which a sound between trigger
        and critical level makes the robot turn towards it (attraction).
    repulsion_angle_range : float, optional
        Half-width (deg) around the front within which a sound above critical
        level makes the robot turn away from it (repulsion).

    Attributes
    ----------
    running : bool
        Flag indicating if the robot's main loop is running.
    turning_angle : int
        Default fixed turning angle in degrees (10 degrees).
    stop_bool : bool
        Flag indicating if robot should stop motion.
    robot : object
        Thymio robot instance for motor and sensor control.

    """

    def __init__(
        self,
        forward_speed,
        behaviour_angular_speed,
        turn_angle,
        left_sensor_threshold,
        right_sensor_threshold,
        critical_level,
        trigger_level,
        ground_sensors_bool=True,
        angle_shift=0,
        arena_angular_speed=150,
        line_avoid_angle=80,
        stuck_speed_threshold=100,
        stuck_time=1.0,
        unstuck_back_time=0.5,
        unstuck_angle=90,
        attraction_angle_range=50,
        repulsion_angle_range=160,
    ):
        self.forward_speed = forward_speed
        self.left_sensor_threshold = left_sensor_threshold
        self.right_sensor_threshold = right_sensor_threshold
        self.ground_sensors_bool = ground_sensors_bool
        self.critical_level = critical_level
        self.trigger_level = trigger_level
        self.angle_shift = angle_shift
        # Angular speeds in deg/s and as per-wheel Thymio speed offsets
        self.behaviour_angular_speed = behaviour_angular_speed
        self.behaviour_angular_speed_thymio = degs_to_speed(behaviour_angular_speed)
        self.arena_angular_speed = arena_angular_speed
        self.arena_angular_speed_thymio = degs_to_speed(arena_angular_speed)
        self.line_avoid_angle = line_avoid_angle
        self._avoiding_line = False
        self.stuck_speed_threshold = stuck_speed_threshold  # Thymio speed units
        self.stuck_time = stuck_time  # s of sustained mismatch before unsticking
        self.unstuck_back_time = unstuck_back_time  # s
        self.unstuck_angle = unstuck_angle  # degrees
        self._stuck_since = None
        # Half-widths (deg) around the front (0 deg) in which a sound triggers a turn
        self.attraction_angle_range = attraction_angle_range
        self.repulsion_angle_range = repulsion_angle_range

        # Polling period (s) of the control loops; Thymio sensors refresh every 0.1 s,
        # so polling faster only steals CPU from the audio threads
        self.loop_period = 0.01

        self.running = True
        self.turning_angle = turn_angle  # degrees
        self.stop_bool = False

        print("Initializing Thymio Robot")
        port = Connection.serial_default_port()
        th = Thymio(
            serial_port=port,
            on_connect=lambda node_id: print(f"Thymio {node_id} is connected"),
        )
        # Connect to Robot
        th.connect()
        self.th = th
        self.robot = ThymioNodeProxy(th[th.first_node()])

        # Print all variables
        # print(th.variables(th.first_node()))
        # time.sleep(1)

        print("Robot connected")

        if self.ground_sensors_bool:
            print("\nGround sensors values...\n")
            time.sleep(0.2)  # wait for sensors to initialize
            print("ground.delta  L R = ", self.robot["prox.ground.delta"])
            # print("ground.reflected  L R = ", self.robot["prox.ground.reflected"])
            print("\n")

    def angle_to_time(self, angle, angular_speed):
        """Calculate the time needed to turn the robot by a specified angle based on angular speed (deg/s).

        Parameters
        ----------
        angle : float
            The rotation angle in degrees.
        angular_speed : float
            The angular speed in degrees per second.

        Returns
        -------
        float
            The time required to turn by the specified angle, in seconds.

        Notes
        -----
        A = 612.33 and B = -0.94 are empirically determined constants.

        """
        # # calculate the time needed to turn the robot by a certain angle
        # A = 612.33
        # B = -0.94
        # t = A * speed**B
        # return t * abs(angle) / 360  # time to turn by angle in seconds

        print("time turn", abs(angle) / float(angular_speed))
        return abs(angle) / float(angular_speed)

    def _turn_speeds(self, arena=False):
        """Return (angular speed in deg/s, per-wheel Thymio offset) for a turn.

        Parameters
        ----------
        arena : bool, optional
            If True use the arena speed (line avoidance, disentangling),
            otherwise the behaviour speed (audio-driven turns).

        """
        if arena:
            return self.arena_angular_speed, self.arena_angular_speed_thymio
        return self.behaviour_angular_speed, self.behaviour_angular_speed_thymio

    def audio_move_360(self):
        """Execute attraction and repulsion-based movement behaviour for the robot.
        self.robot : object
            Thymio robot instance for motor and sensor control.
        self.turning_angle : int
            Fixed turning angle in degrees.

        Notes
        -----
        - The robot's circular yellow LEDs indicate DOA direction in 22 degrees steps
        - When no angle data is available, Top and bottom LEDs illuminate green and the robot moves forward
        - Top and bottom LEDs turn blue when sound is above trigger level and red at critical level

        """

        # 2. Initialize the controller
        try:
            led_controller = SoundLEDController(
                angle_queue=angle_queue_led,
                level_queue=level_queue_led,
                trigger_threshold=self.trigger_level,
                critical_threshold=self.critical_level,
                high_intensity=0x1F,  # Max brightness
                low_intensity=0x01,  # Min brightness
                angle_shift=self.angle_shift,
            )
        except Exception as exc:
            raise RuntimeError("Failed to initialize SoundLEDController") from exc

        self.led_controller = led_controller
        led_controller.start()

        angle = None
        level = None

        while self.running:
            try:
                # Check for a global stop signal (e.g., if the robot is lifted)
                if self.check_stop_all_motion():
                    self.stop_motion()
                    time.sleep(0.1)
                    continue  # Go back to top of loop
                # Priority 1: Check and avoid boundary lines FIRST before processing any audio input
                if self.avoid_line():
                    self.move_forward()
                    angle = None
                    level = None
                    continue

                # Priority 1b: Free the robot if its wheels are blocked (e.g. tangled with another robot)
                if self.is_stuck():
                    self.disentangle()
                    self.move_forward()
                    angle = None
                    level = None
                    continue

                # Flush the angle queue to obtain the latest angle value.
                while not angle_queue.empty():
                    angle = angle_queue.get()
                    # print(f"Angle: {angle} degrees")
                    if angle is not None:
                        if angle > 345 or angle < 22:
                            self.robot["leds.circle"] = [255, 0, 0, 0, 0, 0, 0, 0]
                        elif angle > 22 and angle < 67:
                            self.robot["leds.circle"] = [0, 255, 0, 0, 0, 0, 0, 0]
                        elif angle > 67 and angle < 110:
                            self.robot["leds.circle"] = [0, 0, 255, 0, 0, 0, 0, 0]
                        elif angle > 110 and angle < 155:
                            self.robot["leds.circle"] = [0, 0, 0, 255, 0, 0, 0, 0]
                        elif angle > 155 and angle < 200:
                            self.robot["leds.circle"] = [0, 0, 0, 0, 255, 0, 0, 0]
                        elif angle > 200 and angle < 245:
                            self.robot["leds.circle"] = [0, 0, 0, 0, 0, 255, 0, 0]
                        elif angle > 245 and angle < 300:
                            self.robot["leds.circle"] = [0, 0, 0, 0, 0, 0, 255, 0]
                        elif angle > 300 and angle < 345:
                            self.robot["leds.circle"] = [0, 0, 0, 0, 0, 0, 0, 255]

                    elif angle is None:
                        self.robot["leds.circle"] = [
                            255,
                            255,
                            255,
                            255,
                            255,
                            255,
                            255,
                            255,
                        ]
                        self.move_forward()  # Go straight if no angle is available.
                    else:
                        raise Warning(
                            "Angle value is None, but LED handling not working."
                        )

                while not level_queue.empty():
                    level = level_queue.get()

                # Priority 2: Always check boundary line detection prior to executing audio-driven movements
                if self.avoid_line():
                    self.move_forward()
                    angle = None
                    level = None
                    continue

                # Make a decision based on the latest values.
                if (
                    level is not None
                    and angle is not None
                    and level < self.critical_level
                    and level > self.trigger_level
                ):
                    self.robot["leds.top"] = [0, 0, 255]
                    self.robot["leds.bottom.right"] = [0, 0, 255]
                    if angle > 180:
                        self.robot["leds.bottom.right"] = [0, 0, 255]
                        self.robot["leds.bottom.left"] = [0, 0, 255]
                        self.robot["leds.top"] = [0, 0, 255]
                        if angle >= 360 - self.attraction_angle_range:
                            # self.rotate_right(360 - self.turning_angle)
                            self.rotate_left(self.turning_angle)

                    else:
                        self.robot["leds.bottom.right"] = [0, 0, 255]
                        self.robot["leds.bottom.left"] = [0, 0, 255]
                        self.robot["leds.top"] = [0, 0, 255]
                        if angle <= self.attraction_angle_range:
                            self.rotate_right(self.turning_angle)

                elif (
                    level is not None
                    and angle is not None
                    and level > self.critical_level
                ):
                    self.robot["leds.top"] = [255, 0, 0]
                    self.robot["leds.bottom.right"] = [255, 0, 0]
                    self.robot["leds.bottom.left"] = [255, 0, 0]
                    if angle > 180:
                        self.robot["leds.bottom.right"] = [255, 0, 0]
                        self.robot["leds.bottom.left"] = [255, 0, 0]
                        self.robot["leds.top"] = [255, 0, 0]
                        if angle >= 360 - self.repulsion_angle_range:
                            # self.rotate_right(360 - self.turning_angle)
                            self.rotate_right(self.turning_angle)

                    else:
                        self.robot["leds.bottom.right"] = [255, 0, 0]
                        self.robot["leds.bottom.left"] = [255, 0, 0]
                        self.robot["leds.top"] = [255, 0, 0]
                        if angle <= self.repulsion_angle_range:
                            self.rotate_left(self.turning_angle)

                else:
                    pass
                # After executing a turn, go back to moving straight.
                self.move_forward()
                level = None
                angle = None
                time.sleep(self.loop_period)

            except Exception as e:
                import traceback

                print(f"[ERROR in audio_move_360]: {e}")
                traceback.print_exc()
                time.sleep(0.1)

            except KeyboardInterrupt:
                self.stop()
        else:
            self.stop()
            led_controller.stop()

    def move_forward(self):
        """Move the robot forward at the specified forward speed.

        Attributes Used
        ----------------
        self.forward_speed : int
            Target speed for forward motor movement.
        self.robot : object
            Thymio robot instance for motor and sensor control.

        Notes
        -----
        -Top and bottom LEDs illuminate green and the robot moves forward

        """
        if not self.running or self.robot is None:
            return

        if self.check_stop_all_motion():
            self.stop_motion()
            return

        self.robot["leds.top"] = [0, 255, 0]
        self.robot["leds.bottom.right"] = [0, 255, 0]
        self.robot["leds.bottom.left"] = [0, 255, 0]
        self.robot["motor.left.target"] = self.forward_speed
        self.robot["motor.right.target"] = self.forward_speed

    def rotate_right(self, angle, arena=False):
        """Rotate the robot to the right by a specified angle.

        Parameters
        ----------
        angle : float
            The angle in degrees to rotate the robot.
        arena : bool, optional
            If True turn at arena_angular_speed, otherwise at behaviour_angular_speed.

        Attributes Used
        ----------------
        self.robot : object
            Thymio robot instance for motor and sensor control.

        Notes
        -----
        - Gives priority to the line detection to avoid the robot exiting the arena

        """
        if self.robot is not None and self.running:
            angular_speed, wheel = self._turn_speeds(arena)
            counter = self.angle_to_time(angle, angular_speed)
            print("angle counter =", counter)
            start_time = time.time()
            while time.time() - start_time < counter:
                if not self.running or self.check_stop_all_motion():
                    self.stop_motion()
                    break
                if (
                    not getattr(self, "_avoiding_line", False)
                    and self.is_line_detected()
                ):
                    self.avoid_line()
                    break
                self.robot["motor.left.target"] = int(
                    max(-500, min(500, self.forward_speed + wheel))
                )
                self.robot["motor.right.target"] = int(
                    max(-500, min(500, self.forward_speed - wheel))
                )
                time.sleep(self.loop_period)
            # print(f"Rotating right with angle {angle}")
            clear_queue(
                angle_queue
            )  # Clear the angle queue to avoid processing old angles during rotation
        else:
            self.stop_motion()

    def rotate_left(self, angle, arena=False):
        """Rotate the robot to the left by a specified angle.

        Parameters
        ----------
        angle : float
            The angle in degrees to rotate the robot.
        arena : bool, optional
            If True turn at arena_angular_speed, otherwise at behaviour_angular_speed.

        Attributes Used
        ----------------
        self.robot : object
            Thymio robot instance for motor and sensor control.

        Notes
        -----
        - Gives priority to the line detection to avoid the robot exiting the arena

        """
        if self.robot is not None and self.running:
            angular_speed, wheel = self._turn_speeds(arena)
            counter = self.angle_to_time(angle, angular_speed)
            start_time = time.time()
            while time.time() - start_time < counter:
                if not self.running or self.check_stop_all_motion():
                    self.stop_motion()
                    break
                if (
                    not getattr(self, "_avoiding_line", False)
                    and self.is_line_detected()
                ):
                    self.avoid_line()
                    break
                self.robot["motor.left.target"] = int(
                    max(-500, min(500, self.forward_speed - wheel))
                )
                self.robot["motor.right.target"] = int(
                    max(-500, min(500, self.forward_speed + wheel))
                )
                time.sleep(self.loop_period)
            # print(f"Rotating left with angle {angle}")
            clear_queue(
                angle_queue
            )  # Clear the angle queue to avoid processing old angles during rotation
        else:
            self.stop_motion()

    def smooth_rotate_right(self, angle, arena=False):
        """Rotate the robot to the right by a specified angle but keeps some forward speed while turning.

        Parameters
        ----------
        angle : float
            The angle in degrees to rotate the robot to the right.
        arena : bool, optional
            If True turn at arena_angular_speed, otherwise at behaviour_angular_speed.

        Attributes Used
        ----------------
        self.robot : object
            Thymio robot instance for motor and sensor control.

        Notes
        -----
        - Gives priority to the line detection to avoid the robot exiting the arena

        """
        if self.robot is not None and self.running:
            angular_speed, wheel = self._turn_speeds(arena)
            counter = self.angle_to_time(angle, angular_speed)
            start_time = time.time()
            while time.time() - start_time < counter:
                if not self.running or self.check_stop_all_motion():
                    self.stop_motion()
                    break
                if (
                    not getattr(self, "_avoiding_line", False)
                    and self.is_line_detected()
                ):
                    self.avoid_line()
                    break
                self.robot["motor.left.target"] = int(
                    max(-500, min(500, self.forward_speed + wheel))
                )
                self.robot["motor.right.target"] = int(
                    max(-500, min(500, self.forward_speed))
                )
                time.sleep(self.loop_period)
            # print(f"Smooth rotating right with angle {angle}")
            clear_queue(
                angle_queue
            )  # Clear the angle queue to avoid processing old angles during rotation

        else:
            self.stop_motion()

    def smooth_rotate_left(self, angle, arena=False):
        """Rotate the robot to the left by a specified angle but keeps some forward speed while turning.

        Parameters
        ----------
        angle : float
            The angle in degrees to rotate the robot to the right.
        arena : bool, optional
            If True turn at arena_angular_speed, otherwise at behaviour_angular_speed.

        Attributes Used
        ----------------
        self.robot : object
            Thymio robot instance for motor and sensor control.

        Notes
        -----
        - Gives priority to the line detection to avoid the robot exiting the arena

        """
        if self.robot is not None and self.running:
            angular_speed, wheel = self._turn_speeds(arena)
            counter = self.angle_to_time(angle, angular_speed)
            start_time = time.time()
            while time.time() - start_time < counter:
                if not self.running or self.check_stop_all_motion():
                    self.stop_motion()
                    break
                if (
                    not getattr(self, "_avoiding_line", False)
                    and self.is_line_detected()
                ):
                    self.avoid_line()
                    break
                self.robot["motor.left.target"] = int(
                    max(-500, min(500, self.forward_speed))
                )
                self.robot["motor.right.target"] = int(
                    max(-500, min(500, self.forward_speed + wheel))
                )
                time.sleep(self.loop_period)
            # print(f"smooth Rotating left with angle {angle}")
            clear_queue(
                angle_queue
            )  # Clear the angle queue to avoid processing old angles during rotation
        else:
            self.stop_motion()

    def move_back(self, counter=0.2):
        """Move the robot backward when an obstacle is detected in front.

        Parameters
        ----------
        counter : float, optional
            Time in seconds to move backward (default 0.2 s).

        Attributes Used
        ---------------
        self.robot : object
            Thymio robot instance for motor and sensor control.

        """

        if self.robot is not None:
            start_time = time.time()
            while time.time() - start_time < counter:
                self.robot["motor.left.target"] = -self.forward_speed
                self.robot["motor.right.target"] = -self.forward_speed
                time.sleep(self.loop_period)
        else:
            self.stop_bool = True
            self.stop()

    def check_stop_all_motion(self):
        """Check if the robot should stop all motion based on ground proximity sensors.

        Attributes Used
        ---------------
        self.robot : object
            Thymio robot instance for motor and sensor control.

        Returns
        -------
        bool
            True if the robot is lifted (proximity delta < 10 on either sensor),
            False if robot object is None (after calling stop()).

        """
        if self.robot is not None:
            if (
                self.robot["prox.ground.delta"][0] < 10
                or self.robot["prox.ground.delta"][1] < 10
            ):
                return True
        else:
            self.stop_bool = True
            self.stop()
            return False

    def is_stuck(self):
        """Check if the measured wheel speeds do not match the target speeds.

        Returns
        -------
        bool
            True if, for at least self.stuck_time seconds, either wheel's measured
            speed differs from its target by more than self.stuck_speed_threshold.

        Notes
        -----
        - Requiring a sustained mismatch ignores normal acceleration after a
          target change and single noisy speed readings.

        """
        if self.robot is None or not self.running:
            self._stuck_since = None
            return False
        try:
            targets = (
                self.robot["motor.left.target"],
                self.robot["motor.right.target"],
            )
            speeds = (
                self.robot["motor.left.speed"],
                self.robot["motor.right.speed"],
            )
        except Exception:
            return False

        mismatch = any(t != 0 for t in targets) and any(
            abs(t - s) > self.stuck_speed_threshold for t, s in zip(targets, speeds)
        )
        if not mismatch:
            self._stuck_since = None
            return False

        now = time.time()
        if self._stuck_since is None:
            self._stuck_since = now
        return now - self._stuck_since >= self.stuck_time

    def disentangle(self):
        """Free the robot when stuck: move backward, then turn away in a random direction.

        Attributes Used
        ---------------
        self.unstuck_back_time : float
            Time in seconds to move backward.
        self.unstuck_angle : float
            Angle in degrees to turn after backing up.

        """
        if self.robot is None or not self.running:
            return
        print("[Stuck] Wheel speeds not matching targets, disentangling")
        self.move_back(self.unstuck_back_time)
        self.random_turn(self.unstuck_angle, arena=True)
        self._stuck_since = None
        # Discard audio received while stuck
        clear_queue(angle_queue)
        clear_queue(level_queue)

    def is_line_detected(self):
        """Check if either ground proximity sensor detects a line (black border).

        Returns
        -------
        bool
            True if line detected by left or right sensor, False otherwise.

        """
        if self.robot is None or not self.running:
            return False
        try:
            delta = self.robot["prox.ground.delta"]
            return bool(
                delta[0] < self.left_sensor_threshold
                or delta[1] < self.right_sensor_threshold
            )
        except Exception:
            return False

    def avoid_line(self, angle=None):
        """Avoid line detection and response.

        If both ground sensors detect a line, the robot performs a random turn.
        If only the left sensor detects a line, the robot turns right.
        If only the right sensor detects a line, the robot turns left.

        Parameters
        ----------
        angle : float, optional
            The angle in degrees to turn for line avoidance (defaults to self.line_avoid_angle or 80).

        Returns
        -------
        bool
            True if a line was detected and avoided, False otherwise.

        Attributes Used
        ---------------
        self.left_sensor_threshold : int
            Threshold value for left ground sensor to detect lines.
        self.right_sensor_threshold : int
            Threshold value for right ground sensor to detect lines.
        self.robot : object
            Thymio robot instance for motor and sensor control.
        self.stop_bool : bool
            Set to True when stop condition is met

        """
        if not self.running or self.robot is None:
            return False

        if angle is None:
            angle = getattr(self, "line_avoid_angle", 80)

        line_detected = False
        self._avoiding_line = True
        try:
            if self.robot is not None and self.running:
                delta = self.robot["prox.ground.delta"]
                left_on_line = delta[0] < self.left_sensor_threshold
                right_on_line = delta[1] < self.right_sensor_threshold

                if left_on_line and right_on_line:
                    line_detected = True
                    if self.check_stop_all_motion():
                        self.stop_motion()
                        return True
                    self.random_turn(angle, arena=True)

                elif left_on_line:
                    line_detected = True
                    # Left sensor detects line -> turn right by specified angle (80 deg)
                    counter = self.angle_to_time(angle, self.arena_angular_speed)
                    start_time = time.time()
                    while time.time() - start_time < counter:
                        if not self.running or self.check_stop_all_motion():
                            self.stop_motion()
                            break
                        self.robot["motor.left.target"] = int(
                            max(
                                -500,
                                min(
                                    500, self.forward_speed + self.arena_angular_speed_thymio
                                ),
                            )
                        )
                        self.robot["motor.right.target"] = int(
                            max(
                                -500,
                                min(
                                    500, self.forward_speed - self.arena_angular_speed_thymio
                                ),
                            )
                        )
                        time.sleep(self.loop_period)

                elif right_on_line:
                    line_detected = True
                    # Right sensor detects line -> turn left by specified angle (80 deg)
                    counter = self.angle_to_time(angle, self.arena_angular_speed)
                    start_time = time.time()
                    while time.time() - start_time < counter:
                        if not self.running or self.check_stop_all_motion():
                            self.stop_motion()
                            break
                        self.robot["motor.left.target"] = int(
                            max(
                                -500,
                                min(
                                    500, self.forward_speed - self.arena_angular_speed_thymio
                                ),
                            )
                        )
                        self.robot["motor.right.target"] = int(
                            max(
                                -500,
                                min(
                                    500, self.forward_speed + self.arena_angular_speed_thymio
                                ),
                            )
                        )
                        time.sleep(self.loop_period)

                if line_detected:
                    # Line avoidance always takes strict priority over audio movement.
                    # Clear stale audio queues so robot does not turn based on audio heard before/during line avoidance.
                    clear_queue(angle_queue)
                    clear_queue(level_queue)
                    clear_queue(angle_queue_led)
                    clear_queue(level_queue_led)
            else:
                self.stop_motion()
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            print(f"[ERROR in avoid_line]: {e}")
        finally:
            self._avoiding_line = False

        return line_detected

    def stop_motion(self):
        """Stop robot motors without terminating the main loop."""
        try:
            if self.robot is not None:
                self.robot["motor.left.target"] = 0
                self.robot["motor.right.target"] = 0
        except Exception:
            pass

    def stop(self):
        """Stop all robot movement, turn off Thymio LEDs, and turn ReSpeaker array red at low brightness."""
        self.running = False
        self.stop_bool = True
        try:
            if self.robot is not None:
                self.robot.invalidate()  # always send the stop commands
                self.robot["motor.left.target"] = 0
                self.robot["motor.right.target"] = 0
                self.robot["leds.top"] = [0, 0, 0]
                self.robot["leds.bottom.right"] = [0, 0, 0]
                self.robot["leds.bottom.left"] = [0, 0, 0]
                self.robot["leds.circle"] = [0, 0, 0, 0, 0, 0, 0, 0]
        except Exception as e:
            print(f"Error setting Thymio stop state: {e}")

        if hasattr(self, "led_controller") and self.led_controller is not None:
            try:
                self.led_controller.set_all_red_min_brightness()
            except Exception as e:
                print(f"Error turning ReSpeaker red: {e}")

        import time

        time.sleep(0.3)  # Give thymiodirect time to send the commands over serial

    def random_turn(self, angle, arena=False):
        """Perform a random movement of the robot in a random direction.

        Parameters
        ----------
        angle : float
            The rotation angle in degrees
        arena : bool, optional
            If True turn at arena_angular_speed, otherwise at behaviour_angular_speed.

        Attributes Used
        ---------------
        self.robot : object
            Thymio robot instance for motor and sensor control.
        self.stop_bool : bool
            Set to True when stop condition is met

        """
        if not self.running or self.robot is None:
            return
        if self.check_stop_all_motion():
            self.stop_motion()
            return
        if bool(random.getrandbits(1)):
            self.rotate_right(angle, arena)
        else:
            self.rotate_left(angle, arena)
