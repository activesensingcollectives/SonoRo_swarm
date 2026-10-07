import time
import queue
import threading
import usb.core
import usb.util

from usb_pixel_ring_v2 import PixelRing


class SoundLEDController:
    def __init__(
        self,
        angle_queue,
        level_queue,
        trigger_threshold,
        critical_threshold,
        high_intensity=0x1F,
        low_intensity=0x01,
        angle_shift=-45,
    ):
        # We now track two separate queues
        self.angle_queue = angle_queue
        self.level_queue = level_queue

        self.trigger_threshold = trigger_threshold
        print(f"Trigger Threshold set to: {self.trigger_threshold} dB SPL")
        self.critical_threshold = critical_threshold
        print(f"Critical Threshold set to: {self.critical_threshold} dB SPL")

        # Cap intensity bounds between 0 (0x00) and 31 (0x1F)
        self.high_intensity = max(0x00, min(high_intensity, 0x1F))
        self.low_intensity = max(0x00, min(low_intensity, 0x1F))
        self.angle_shift = angle_shift

        self.running = False

        # Locate the ReSpeaker Mic Array v2.0 hardware
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if not self.dev:
            raise RuntimeError(
                "ReSpeaker Mic Array v2.0 not found. Check USB connection or run with sudo."
            )

        self.pixel_ring = PixelRing(self.dev)

    def _angle_to_led_index(self, angle):
        """
        Maps a 0-360 degree DOA angle to a 0-11 LED index on the ReSpeaker soundcard.
        - shift = 45: angle 0 -> LED 11
        - shift = -45: angle 0 -> LED 2
        """
        base_index = int(((float(angle) + 15) % 360) / 30)
        slot_offset = int(round(2.0 - (float(self.angle_shift) + 45.0) / 30.0))
        return (base_index + slot_offset) % 12

    def _build_custom_led_array(self, target_led_index, r, g, b):
        """
        Creates an array to light up ONLY the dynamically targeted LED.
        The remaining 11 LEDs are kept dark (0, 0, 0).
        """
        data = []
        for i in range(12):
            if i == target_led_index:
                data.extend([r, g, b, 0])  # Light up target LED
            else:
                data.extend([0, 0, 0, 0])  # Keep others dark
        return data

    def start(self):
        """Starts the dual-queue monitoring thread."""
        self.running = True
        self.thread = threading.Thread(target=self._process_queues, daemon=True)
        self.thread.start()
        print("LED Controller started.")

    def set_all_red_min_brightness(self):
        """Sets all 12 ReSpeaker LEDs to solid red at low brightness (0x01)."""
        try:
            self.pixel_ring.set_brightness(self.low_intensity)  # Low brightness (0x01)
            self.pixel_ring.mono(0xFF0000)
        except Exception as e:
            print(f"Error setting ReSpeaker LEDs red: {e}")

    def set_all_red_full_brightness(self):
        """Alias to set_all_red_min_brightness."""
        self.set_all_red_min_brightness()

    def stop(self):
        """Stops the controller safely and turns all array LEDs red at low brightness."""
        self.running = False

        # Send poison pills to unblock BOTH queues
        self.angle_queue.put(None)
        self.level_queue.put(None)

        if hasattr(self, "thread") and self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

        # Turn all LEDs red at low brightness upon stopping
        self.set_all_red_min_brightness()
        print("LED Controller stopped. Array LEDs turned red at low brightness.")

    def _process_queues(self):
        while self.running:
            try:
                # Retrieve latest values from queues, or use last known values
                try:
                    angle = self.angle_queue.get_nowait()
                except queue.Empty:
                    angle = self.angle_queue.get(timeout=1)

                try:
                    level = self.level_queue.get_nowait()
                except queue.Empty:
                    level = self.level_queue.get(timeout=1)

                # If stop() was called, exit loop immediately without touching LEDs
                if not self.running:
                    break

                # Figure out which LED points toward the sound
                if angle is None or level is None:
                    self.pixel_ring.set_brightness(self.low_intensity)
                    # Show a subtle green when no angle/level is available
                    self.pixel_ring.mono(0x00FF00)

                else:
                    target_led = self._angle_to_led_index(angle)

                    # Condition 1: Critical Threshold (Red, High Intensity, Directional LED)
                    if level > self.critical_threshold:
                        self.pixel_ring.set_brightness(self.high_intensity)
                        led_data = self._build_custom_led_array(target_led, 255, 0, 0)
                        self.pixel_ring.customize(led_data)

                    # Condition 2: Trigger Threshold (Blue, High Intensity, Directional LED)
                    elif level > self.trigger_threshold:
                        self.pixel_ring.set_brightness(self.high_intensity)
                        led_data = self._build_custom_led_array(target_led, 0, 0, 255)
                        self.pixel_ring.customize(led_data)

                # print(f"LED Processing -> Angle: {angle}, dB SPL: {level}")

            except queue.Empty:
                pass
            except Exception as e:
                print(f"Encountered an error in LED processing: {e}")
