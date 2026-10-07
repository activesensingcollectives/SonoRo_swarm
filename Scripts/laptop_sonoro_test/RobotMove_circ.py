#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created: 2025-12-17
Author: Alberto Doimo
email: alberto.doimo@uni-konstanz.de

"""

from thymiodirect import Connection
from thymiodirect import Thymio
from .shared_queues import angle_queue, level_queue, angle_queue_led, level_queue_led

import time
import random
from .SoundLEDController import SoundLEDController


class RobotMove:
    """This class manages robot motor control, LED indicators, ground sensors,
    and implements various behavioral modes

    Parameters
    ----------
    forward_speed : int
        Target speed for forward motor movement.
    turn_speed : in
        Target speed for rotational motor movement.
    left_sensor_threshold : int
        Threshold value for left ground sensor to detect white lines.
    right_sensor_threshold : int
        Threshold value for right ground sensor to detect white lines.
    trigger_level : float
        Sound pressure level (dB SPL) threshold to trigger detection.
    critical_level : float
        Critical sound pressure level (dB SPL) for avoiding collisions.
    ground_sensors_bool : bool, optional
        If True, initializes and monitors ground sensor values at start.

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
        turn_speed,
        left_sensor_threshold,
        right_sensor_threshold,
        critical_level,
        trigger_level,
        ground_sensors_bool=True,
    ):
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.left_sensor_threshold = left_sensor_threshold
        self.right_sensor_threshold = right_sensor_threshold
        self.ground_sensors_bool = ground_sensors_bool
        self.critical_level = critical_level
        self.trigger_level = trigger_level

        self.running = True
        self.turning_angle = 1  # degrees
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
        self.robot = th[th.first_node()]

        # Print all variables
        # print(th.variables(th.first_node()))
        # time.sleep(1)

        print("Robot connected")

        if self.ground_sensors_bool:
            print("\nGround sensors values...\n")
            time.sleep(0.2)  # wait for sensors to initialize
            # print("ground.delta  L R = ", self.robot["prox.ground.delta"])
            print("ground.reflected  L R = ", self.robot["prox.ground.reflected"])
            print("\n")

    def angle_to_time(self, angle, speed):
        """Calculate the time needed to turn the robot by a specified angle.

        Parameters
        ----------
        angle : float
            The rotation angle in degrees.
        speed : float
            The rotational speed parameter.

        Returns
        -------
        float
            The time required to turn by the specified angle, in seconds.

        Notes
        -----
        A = 612.33 and B = -0.94 are empirically determined constants.

        """
        # calculate the time needed to turn the robot by a certain angle
        A = 612.33
        B = -0.94
        t = A * speed**B
        return t * abs(angle) / 360  # time to turn by angle in seconds

    def audio_move_360(self):
        """Execute attraction and repulsion-based movement behaviour for the robot.
        Based on given angles over 360 degrees.

        Attributes Used
        ----------------
        self.running : bool
            Flag indicating if the robot's main loop is running.
        critical_level : float
            Critical sound pressure level (dB SPL) for avoiding collisions.
        trigger_level : float
            Sound pressure level (dB SPL) threshold to trigger detection.
        angle_queue : queue.Queue
            Inter-thread queue for sharing detected angles from AudioProcessor.
        level_queue : queue.Queue
            Inter-thread queue for sharing SPL levels from AudioProcessor.
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
        led_controller = SoundLEDController(
            angle_queue=angle_queue_led,
            level_queue=level_queue_led,
            trigger_threshold=self.trigger_level,
            critical_threshold=self.critical_level,
            high_intensity=0x1F,  # Max brightness
            low_intensity=0x01,  # Min brightness
        )
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

                self.avoid_white_line()

                # Flush the angle queue to obtain the latest angle value.
                while not angle_queue.empty():
                    angle = angle_queue.get()
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

                # Flush the level queue similarly to get the latest level value.
                while not level_queue.empty():
                    level = level_queue.get()

                # Make a decision based on the latest values.
                if (
                    level is not None
                    and angle is not None
                    and level < self.critical_level
                    and level > self.trigger_level
                ):
                    self.robot["leds.top"] = [0, 0, 255]
                    self.robot["leds.bottom.right"] = [0, 0, 255]
                    self.robot["leds.bottom.left"] = [0, 0, 255]
                    if angle > 180:
                        self.robot["leds.bottom.right"] = [0, 0, 255]
                        self.robot["leds.bottom.left"] = [0, 0, 255]
                        self.robot["leds.top"] = [0, 0, 255]
                        if angle >= 240:
                            # self.rotate_right(360 - self.turning_angle)
                            self.smooth_rotate_left(self.turning_angle)

                    else:
                        self.robot["leds.bottom.right"] = [0, 0, 255]
                        self.robot["leds.bottom.left"] = [0, 0, 255]
                        self.robot["leds.top"] = [0, 0, 255]
                        if angle <= 120:
                            self.smooth_rotate_right(self.turning_angle)

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
                        if angle >= 260:
                            # self.rotate_right(360 - self.turning_angle)
                            self.smooth_rotate_right(self.turning_angle)

                    else:
                        self.robot["leds.bottom.right"] = [255, 0, 0]
                        self.robot["leds.bottom.left"] = [255, 0, 0]
                        self.robot["leds.top"] = [255, 0, 0]
                        if angle <= 100:
                            self.smooth_rotate_left(self.turning_angle)

                else:
                    pass
                # After executing a turn, go back to moving straight.
                self.move_forward()
                level = None
                angle = None

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

    def rotate_right(self, angle):
        """Rotate the robot to the right by a specified angle.

        Parameters
        ----------
        angle : float
            The angle in degrees to rotate the robot.

        Attributes Used
        ----------------
        self.robot : object
            Thymio robot instance for motor and sensor control.
        self.turn_speed : int
            Target speed for rotational motor movement.

        Notes
        -----
        - Gives priority to the line detection to avoid the robot exiting the arena

        """
        if self.robot is not None:
            counter = self.angle_to_time(angle, self.turn_speed)
            start_time = time.time()
            while time.time() - start_time < counter:
                if self.check_stop_all_motion():
                    self.stop_bool = True
                    break
                self.robot["motor.left.target"] = self.turn_speed
                self.robot["motor.right.target"] = -self.turn_speed
            # print(f"Rotating right with angle {angle}")
            angle_queue.clear()  # Clear the angle queue to avoid processing old angles during rotation
        else:
            self.stop()

    def rotate_left(self, angle):
        """Rotate the robot to the left by a specified angle.

        Parameters
        ----------
        angle : float
            The angle in degrees to rotate the robot.

        Attributes Used
        ----------------
        self.robot : object
            Thymio robot instance for motor and sensor control.
        self.turn_speed : int
            Target speed for rotational motor movement.

        Notes
        -----
        - Gives priority to the line detection to avoid the robot exiting the arena

        """
        if self.robot is not None:
            counter = self.angle_to_time(angle, self.turn_speed)
            start_time = time.time()
            while time.time() - start_time < counter:
                if self.check_stop_all_motion():
                    self.stop_bool = True
                    break
                self.robot["motor.left.target"] = -self.turn_speed
                self.robot["motor.right.target"] = self.turn_speed
            # print(f"Rotating left with angle {angle}")
            angle_queue.clear()  # Clear the angle queue to avoid processing old angles during rotation
        else:
            self.stop()

    def smooth_rotate_right(self, angle):
        """Rotate the robot to the right by a specified angle but keeps some forward speed while turning.

        Parameters
        ----------
        angle : float
            The angle in degrees to rotate the robot to the right.

        Attributes Used
        ----------------
        self.robot : object
            Thymio robot instance for motor and sensor control.
        self.turn_speed : int
            Target speed for rotational motor movement.

        Notes
        -----
        - Gives priority to the line detection to avoid the robot exiting the arena

        """
        if self.robot is not None:
            counter = self.angle_to_time(angle, self.turn_speed)
            start_time = time.time()
            while time.time() - start_time < counter:
                if self.check_stop_all_motion():
                    self.stop_bool = True
                    break
                self.robot["motor.left.target"] = self.turn_speed + self.forward_speed
                self.robot["motor.right.target"] = -self.turn_speed + self.forward_speed
            # print(f"Smooth rotating right with angle {angle}")
            angle_queue.clear()  # Clear the angle queue to avoid processing old angles during rotation

        else:
            self.stop()

    def smooth_rotate_left(self, angle):
        """Rotate the robot to the left by a specified angle but keeps some forward speed while turning.

        Parameters
        ----------
        angle : float
            The angle in degrees to rotate the robot to the right.

        Attributes Used
        ----------------
        self.robot : object
            Thymio robot instance for motor and sensor control.
        self.turn_speed : int
            Target speed for rotational motor movement.

        Notes
        -----
        - Gives priority to the line detection to avoid the robot exiting the arena

        """
        if self.robot is not None:
            counter = self.angle_to_time(angle, self.turn_speed)
            start_time = time.time()
            while time.time() - start_time < counter:
                if self.check_stop_all_motion():
                    self.stop_bool = True
                    break
                self.robot["motor.left.target"] = -self.turn_speed + self.forward_speed
                self.robot["motor.right.target"] = self.turn_speed + self.forward_speed
            # print(f"smooth Rotating left with angle {angle}")
            angle_queue.clear()  # Clear the angle queue to avoid processing old angles during rotation
        else:
            self.stop()

    def move_back(self):
        """Move the robot backward when an obstacle is detected in front.

        Attributes Used
        ---------------
        self.robot : object
            Thymio robot instance for motor and sensor control.

        """

        if self.robot is not None:
            counter = 0.2  # sec
            start_time = time.time()
            while time.time() - start_time < counter:
                self.robot["motor.left.target"] = -self.forward_speed
                self.robot["motor.right.target"] = -self.forward_speed
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

    def avoid_white_line(self):
        """Avoid white line detection and response.

        If both ground sensors detect a white line, the robot performs a random turn.
        If only the left sensor detects a white line, the robot turns right.
        If only the right sensor detects a white line, the robot turns left.

        Attributes Used
        ---------------
        self.left_sensor_threshold : int
            Threshold value for left ground sensor to detect white lines.
        self.right_sensor_threshold : int
            Threshold value for right ground sensor to detect white lines.
        self.robot : object
            Thymio robot instance for motor and sensor control.
        self.stop_bool : bool
            Set to True when stop condition is met

        """
        if not self.running or self.robot is None:
            return
        try:
            if self.robot is not None and self.running:
                # check if the white line is detected
                if (
                    self.robot["prox.ground.reflected"][0] > self.left_sensor_threshold
                    and self.robot["prox.ground.reflected"][1]
                    > self.right_sensor_threshold
                ):
                    # Both sensors detect the line
                    if self.check_stop_all_motion():
                        self.stop_motion()
                        return
                    self.random_turn(30)

                elif (
                    self.robot["prox.ground.reflected"][0] > self.left_sensor_threshold
                ):
                    # Left sensor detects the line
                    counter = self.angle_to_time(20, self.turn_speed)
                    start_time = time.time()
                    while time.time() - start_time < counter:
                        if not self.running or self.check_stop_all_motion():
                            self.stop_motion()
                            break
                        self.robot["motor.left.target"] = self.turn_speed
                        self.robot["motor.right.target"] = -self.turn_speed

                elif (
                    self.robot["prox.ground.reflected"][1] > self.right_sensor_threshold
                ):
                    # Right sensor detects the line
                    counter = self.angle_to_time(20, self.turn_speed)
                    start_time = time.time()
                    while time.time() - start_time < counter:
                        if not self.running or self.check_stop_all_motion():
                            self.stop_motion()
                            break
                        self.robot["motor.left.target"] = -self.turn_speed
                        self.robot["motor.right.target"] = self.turn_speed

            else:
                self.stop_motion()
        except KeyboardInterrupt:
            self.stop()

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

    def random_turn(self, angle):
        """Perform a random movement of the robot in a random direction.

        Parameters
        ----------
        angle : float
            The rotation angle in degrees

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
            self.rotate_right(angle)
        else:
            self.rotate_left(angle)
