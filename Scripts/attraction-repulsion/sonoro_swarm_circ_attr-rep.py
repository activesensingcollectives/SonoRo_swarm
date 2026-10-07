#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created: 2025-12-16
Author: Alberto Doimo
email: alberto.doimo@uni-konstanz.de

Description
--------
SonoRo Swarm main file
This module implements direction-of-arrival (DOA) detection using microphone arrays and
controls a Thymio robot based on acoustic stimuli.

Features
--------
- Direction-of-arrival (DOA) computation using DAS or CC algorithms
- Sound pressure level (SPL) calculation in decibels
- Trigger-based computation for efficient processing
- Data export
- Real-time robot movement control based on acoustic detection

"""

#############################################################################
# Libraries import
#############################################################################

print("import libraries...")

from tracemalloc import start

from omegaconf import OmegaConf, DictConfig
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import signal as std_signal
import scipy.signal as signal
import sounddevice as sd
import soundfile as sf
import argparse
import queue
import datetime
import time
import os
import subprocess
import threading
import pandas as pd

# import psutil
import socket
import netifaces as ni
import hydra

import utilities

from AudioProcessor_circ_array import AudioProcessor
from RobotMove_circ import RobotMove
from shared_queues import (
    angle_queue,
    level_queue,
    angle_queue_led,
    level_queue_led,
)

print("imports done")


def get_local_ip(interface: str = "wlan0") -> str:
    """Safely get local IPv4 address with multiple fallbacks."""
    try:
        addrs = ni.ifaddresses(interface)
        if ni.AF_INET in addrs and addrs[ni.AF_INET]:
            return addrs[ni.AF_INET][0]["addr"]
    except Exception:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def get_temperature_celsius() -> float | None:
    try:
        result = subprocess.run(
            ["vcgencmd", "measure_temp"],
            check=True,
            capture_output=True,
            text=True,
        )
        temp_str = result.stdout.strip()
        # Output format: "temp=XX.X'C"
        if "temp=" in temp_str:
            temp_value = float(temp_str.split("=")[1].split("'")[0])
            return temp_value
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError, IndexError):
        return None

    return None


def get_pi_throttled_status() -> str:
    throttle_paths = (
        "/sys/devices/platform/soc/soc:firmware/get_throttled",
        "/sys/devices/platform/soc/soc0/get_throttled",
    )

    raw_value = None
    for throttle_path in throttle_paths:
        if os.path.exists(throttle_path):
            with open(throttle_path, "r", encoding="utf-8") as throttle_file:
                raw_value = throttle_file.read().strip()
            break

    if raw_value is None:
        try:
            result = subprocess.run(
                ["vcgencmd", "get_throttled"],
                check=True,
                capture_output=True,
                text=True,
            )
            raw_value = result.stdout.strip().split("=", 1)[-1]
        except (FileNotFoundError, subprocess.CalledProcessError, IndexError):
            return "N/A"

    try:
        throttle_flags = int(raw_value, 16)
    except ValueError:
        return raw_value

    status_labels = []
    if throttle_flags & 0x1:
        status_labels.append("under-voltage now")
    if throttle_flags & 0x2:
        status_labels.append("freq capped now")
    if throttle_flags & 0x4:
        status_labels.append("throttled now")
    if throttle_flags & 0x8:
        status_labels.append("soft temp limit now")
    if throttle_flags & 0x10000:
        status_labels.append("under-voltage occurred")
    if throttle_flags & 0x20000:
        status_labels.append("freq capped occurred")
    if throttle_flags & 0x40000:
        status_labels.append("throttled occurred")
    if throttle_flags & 0x80000:
        status_labels.append("soft temp limit occurred")

    if not status_labels:
        status_labels.append("not throttled")

    return f"{raw_value} ({', '.join(status_labels)})"


###############################################################################
# SETUP PARAMETERS
###############################################################################
# Create queues for storing data
timestamp_queue = queue.Queue()

timestamp_bool = True  # Set to True to save timestamps, False to not save timestamps
if timestamp_bool == True:
    print("Timestamps saving is ENABLED")
else:
    print("Timestamps saving is DISABLED")

recording_bool = (
    True  # Set to True to record audio, False to just process audio without recording
)
if recording_bool == True:
    print("Recording is ENABLED")
else:
    print("Recording is DISABLED")


# Get the index of the USB card
soundcard_index = utilities.get_card(sd.query_devices(), "ReSpeaker")
print(sd.query_devices())
print("soundcard_index=", soundcard_index)

raspi_local_ip = get_local_ip("wlan0")
print("raspi_local_ip =", raspi_local_ip)


#######################################################################################
# MAIN FUNCTION
#######################################################################################


@hydra.main(
    version_base=None,
    config_path=".",
    config_name="config_attr-rep",
)
def main(cfg: DictConfig) -> None:
    # Convert to standard dictionary (strips away OmegaConf overhead)
    native_cfg = OmegaConf.to_container(cfg, resolve=True)

    # Set up timestamps and paths for saving data
    startime = datetime.datetime.now()
    time1 = startime.strftime("%Y-%m-%d")
    time2 = startime.strftime("_%Y-%m-%d__%H-%M-%S")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up 2 levels to reach the main SonoRo_swarm folder
    save_path = os.path.dirname(os.path.dirname(script_dir))
    # folder_name = str(raspi_local_ip)
    folder_name = str("test")
    cfg_folder_path = os.path.join(save_path, "Data", folder_name, time1, time2)
    os.makedirs(cfg_folder_path, exist_ok=True)
    OmegaConf.save(cfg, os.path.join(cfg_folder_path, "config" + time2 + ".yaml"))

    ICI = 1 / native_cfg["call_rate"]  # inter-call interval in seconds

    input_buffer_time = (
        ICI - native_cfg["duration_out"] - 50e-3 - 50e-3 - 20e-3
    )  # seconds
    print("input_buffer_time =", input_buffer_time)
    block_size = int(
        input_buffer_time * native_cfg["fs"]
    )  # used for the shared queue from which the doa is computed, not anymore for the output stream

    # Parameters for the DAS algorithm
    theta_das = np.arange(
        native_cfg["theta_das_start"],
        native_cfg["theta_das_end"],
        native_cfg["theta_das_step"],
    )  # (start, end, step)

    silence_post = (
        native_cfg["duration_out"] * 1000 + 30
    )  # [ms](duration of the chirp + 30ms of estimated time for the DAC to output the signal)

    t = np.linspace(
        0,
        native_cfg["duration_out"],
        int(native_cfg["fs"] * native_cfg["duration_out"]),
    )
    sweep = signal.chirp(t, native_cfg["start_f"], t[-1], native_cfg["end_f"])
    sweep *= signal.windows.tukey(sweep.size, 0.2)
    sweep *= 0.8

    silence_samples_post = int(silence_post * native_cfg["fs"] / 1000)
    silence_vec_post = np.zeros((silence_samples_post,))
    post_silence_sig = np.concatenate((sweep, silence_vec_post))
    full_sig = post_silence_sig

    stereo_sig = np.hstack([full_sig.reshape(-1, 1), full_sig.reshape(-1, 1)])
    data = native_cfg["amplitude"] * np.float32(stereo_sig)

    out_blocksize = int(len(data))  # Length of the output signal
    print("out_blocksize =", out_blocksize)

    # Set the path for the sensitivity CSV file relative to the current script location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # sensitivity_path = os.path.join(script_dir, "Knowles_SPH0645LM4H-B_sensitivity.csv")
    sensitivity_path = os.path.join(
        script_dir, "Null_sensitivity.csv"
    )  # no sensitivity compensation applied for the laptop test
    sensitivity = pd.read_csv(sensitivity_path)
    # print(sensitivity.head())  # Print the first few rows of the sensitivity DataFrame

    block_size_analyzed_buffer = int(
        native_cfg["analyzed_buffer_time"] * native_cfg["fs"]
    )
    sos = signal.butter(
        1, native_cfg["cutoff_freq"], "hp", fs=native_cfg["fs"], output="sos"
    )

    in_sig = np.zeros(
        block_size_analyzed_buffer, dtype=np.float32
    )  # Initialize the buffer for the audio input stream
    print("in_sig shape:", np.shape(in_sig))
    centrefreqs, freqrms = utilities.calc_native_freqwise_rms(in_sig, native_cfg["fs"])

    freqs = np.array(sensitivity.iloc[:, 0])  # first column contains frequencies
    sens_freqwise_rms = np.array(
        sensitivity.iloc[:, 1]
    )  # Last column contains sensitivity values

    interp_sensitivity = utilities.interpolate_freq_response(
        [freqs, sens_freqwise_rms], centrefreqs
    )

    # print("Interpolated sensitivity values for the center frequencies:")
    # for freq, sens in zip(centrefreqs, interp_sensitivity):
    #     print(f"Frequency: {freq:.2f} Hz, Sensitivity: {sens:.4f}")

    frequency_band = [
        native_cfg["doa_freq_range"][0],
        native_cfg["doa_freq_range"][1],
    ]  # min, max frequency to do the compensation Hz
    tgtmic_relevant_freqs = np.logical_and(
        centrefreqs >= frequency_band[0], centrefreqs <= frequency_band[1]
    )
    # Thymio movement parameters
    speed = utilities.perc_to_speed(
        native_cfg["speed_percent"]
    )  # Map 0-100 perc speed to thymio range (0-500)
    turn_speed = utilities.perc_to_speed(native_cfg["turn_speed_percent"])

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Parse arguments and pass them to main function
    args = parser.parse_args()
    args.buffer = np.zeros((block_size, native_cfg["channels"]))

    # Set initial parameters for the audio processing
    args.samplerate = native_cfg["fs"]
    args.rec_samplerate = native_cfg["rec_samplerate"]
    args.angle = 0
    args.subtype = None

    # Set general save path
    # Note: Using exist_ok=True ensures that the Data folder structure is preserved
    # and only missing subdirectories are created (no data is overwritten)
    folder_path = os.path.join(save_path, "Data", folder_name, time1, time2)

    # Create folder structure once (with exist_ok=True to preserve existing data)
    os.makedirs(folder_path, exist_ok=True)

    if recording_bool == True:
        # Create filename for saving recordings
        name = "MULTIWAV_" + str(raspi_local_ip) + str(time2) + ".wav"
        args.filename = os.path.join(folder_path, name)

        if args.samplerate is None:
            print("error!: no samplerate set! Using default")
            device_info = sd.query_devices(args.device, "input")
            args.samplerate = int(device_info["default_samplerate"])
        if args.filename is None:
            timenow = datetime.datetime.now()
            args.filename = name
        print(args.samplerate)

    if timestamp_bool == True:
        # Create filename for saving timestamps
        name = "TIMESTAMPS_" + str(raspi_local_ip) + str(time2) + ".csv"
        args.timestamp_filename = os.path.join(folder_path, name)

    # Create instances of the AudioProcessor and RobotMove classes

    robot_move = RobotMove(
        speed,
        turn_speed,
        native_cfg["turn_angle"],
        native_cfg["left_sensor_threshold"],
        native_cfg["right_sensor_threshold"],
        native_cfg["critical_level"],
        native_cfg["trigger_level"],
        ground_sensors_bool=True,
        angle_shift=native_cfg.get("shift", -45),
    )

    def _sig_shutdown_handler(signum, frame):
        print(f"\n[Shutdown] Received signal {signum}, stopping robot cleanly...")
        try:
            robot_move.stop()
        except Exception:
            pass
        raise KeyboardInterrupt

    std_signal.signal(std_signal.SIGTERM, _sig_shutdown_handler)
    std_signal.signal(std_signal.SIGINT, _sig_shutdown_handler)
    try:
        std_signal.signal(std_signal.SIGHUP, _sig_shutdown_handler)
    except AttributeError:
        pass

    # # ========================================================================
    # # NOISE CALIBRATION STEP
    # # Robot spins in place with varying speed while recording gear noise.
    # # After calibration, robot stops and whitening matrices are computed.
    # # ========================================================================

    # print("\n" + "=" * 60)
    # print("STARTING NOISE CALIBRATION")
    # print("Robot will rotate in place with varying speed...")
    # print("=" * 60 + "\n")

    # calib_duration = native_cfg.get("calib_duration_s", 12.0)  # seconds
    # calib_wlen = native_cfg.get("calib_wlen", 64)

    # calib_buffer_list = []

    # def _calib_audio_callback(indata, frames, time_info, status):
    #     if status:
    #         print("Calibration stream status:", status)
    #     calib_buffer_list.append(indata.copy())

    # def _spin_robot_variable_speed(robot_move_obj, duration, base_turn_speed):
    #     start = time.time()
    #     while time.time() - start < duration:
    #         elapsed = time.time() - start
    #         speed_factor = 0.4 + 0.6 * (0.5 * (1 + np.sin(2 * np.pi * elapsed / 4.0)))
    #         current_speed = int(base_turn_speed * speed_factor)
    #         robot_move_obj.robot["motor.right.target"] = current_speed
    #         robot_move_obj.robot["motor.left.target"] = -current_speed
    #         time.sleep(0.1)
    #     robot_move_obj.stop()

    # calib_stream = sd.InputStream(
    #     samplerate=native_cfg["fs"],
    #     channels=native_cfg["channels"],
    #     device=soundcard_index,
    #     callback=_calib_audio_callback,
    # )

    # with calib_stream:
    #     spin_thread = threading.Thread(
    #         target=_spin_robot_variable_speed,
    #         args=(robot_move, calib_duration, utilities.perc_to_speed(100)),
    #     )
    #     spin_thread.start()
    #     spin_thread.join()
    #     time.sleep(0.2)

    # robot_move.stop()
    # print("Calibration rotation complete, robot stopped.\n")

    # noise_calib_audio = np.concatenate(calib_buffer_list, axis=0)
    # noise_calib_audio_doa = noise_calib_audio[:, 1:5]  # match runtime exactly

    # print(f"Calibration audio captured: shape = {noise_calib_audio_doa.shape}")

    # beamformer = utilities.WhitenedUCABeamformer(
    #     fs=native_cfg["fs"],
    #     nch=4,
    #     r=native_cfg["radius"],
    #     shift=native_cfg["shift"],
    #     bw=native_cfg["doa_freq_range"],
    #     theta=theta_das,
    #     wlen=native_cfg.get("wlen"),
    #     nfft=native_cfg.get("nfft"),
    # )
    # beamformer.calibrate(noise_calib_audio_doa)

    # beamformer.plot_calibration_diagnostics(save_dir=folder_path, prefix="calib")

    # print("=" * 60)
    # print("NOISE CALIBRATION COMPLETE — starting experiment")
    # print("=" * 60 + "\n")

    # # ========================================================================
    # # END NOISE CALIBRATION STEP
    # # ========================================================================

    audio_processor = AudioProcessor(
        native_cfg["fs"],
        native_cfg["channels"],
        native_cfg["radius"],
        native_cfg["shift"],
        block_size,
        native_cfg["analyzed_buffer_time"],
        data,
        args,
        native_cfg["trigger_level"],
        native_cfg["critical_level"],
        native_cfg["ref"],
        theta_das,
        native_cfg["N_peaks"],
        soundcard_index,
        args.subtype,
        interp_sensitivity,
        tgtmic_relevant_freqs,
        args.filename,
        args.rec_samplerate,
        native_cfg["doa_freq_range"],
        sos,
    )

    # Create threads for the audio input and recording
    if recording_bool == True:
        inputstream_thread = threading.Thread(
            target=audio_processor.continuos_recording, daemon=True
        )
        inputstream_thread.start()
    else:
        inputstream_thread = threading.Thread(
            target=audio_processor.input_stream, daemon=True
        )
        inputstream_thread.start()
        print("Input stream thread started")

    move_thread = threading.Thread(target=robot_move.audio_move_360, daemon=True)
    move_thread.start()

    print("Waiting for the first input buffer to fill...")
    time.sleep(1)  # Wait a moment to ensure threads are properly started

    event = threading.Event()
    # process = psutil.Process(os.getpid())
    # process.cpu_percent(None)
    last_resource_report = time.time()

    time2 = startime.strftime("_%Y-%m-%d__%H-%M-%S")
    os.makedirs(folder_path, exist_ok=True)
    name = "TIMESTAMPS_" + str(raspi_local_ip) + str(time2)
    npy_data = os.path.join(folder_path, name)

    # Synchronized swarm start barrier
    target_start_epoch = float(os.environ.get("SWARM_START_TIME", 0))
    if target_start_epoch > 0:
        now = datetime.datetime.now(datetime.timezone.utc).timestamp()
        wait_seconds = target_start_epoch - now
        if wait_seconds > 0:
            print(
                f"[Sync] Waiting {wait_seconds:.2f}s for synchronized start at UTC epoch {target_start_epoch:.2f}..."
            )
            time.sleep(wait_seconds)
            print(
                f"[Sync] Synchronized start triggered at UTC {datetime.datetime.now(datetime.timezone.utc).strftime('%H:%M:%S.%f')[:-3]}!"
            )
        else:
            print(
                f"[Sync Warning] Initialization took {-wait_seconds:.2f}s longer than planned start time."
            )

    # timestamp_queue.put([audio_processor.ts_queue.get(), 0, 'recording_start'])
    timestamp_queue.put([audio_processor.ts_queue.get(), 0, 0, "recording_start"])
    end_time = time.time()
    time40 = time.time()
    try:
        while True:
            start_time = time.time()
            event.clear()
            timestamp = datetime.datetime.timestamp(
                datetime.datetime.now(datetime.timezone.utc)
            )  # POSIX timestamp
            timestamp_queue.put(
                [timestamp, 0, 0, "output_start"]
            )  # Put the timestamp in the queue (no block=False, keeps all values)

            with sd.OutputStream(
                samplerate=native_cfg["fs"],
                blocksize=0,
                device=soundcard_index,
                channels=2,
                callback=audio_processor.callback_out,
                finished_callback=event.set,
                latency="low",
            ) as out_stream:
                with out_stream:
                    event.wait()
            # print(sd.check_output_settings())
            # print("call duration: ", time.time() - start_time)
            # print("Total loop time: ", time.time() - end_time)
            # add a small random jitter to avoid strict periodic locking
            wait_time = ICI - (time.time() - end_time) + np.random.uniform(0, 0.05)
            # print("wait time", wait_time)
            if wait_time > 0:
                time.sleep(wait_time)
            else:
                print(
                    f"Warning: computation took more than ICI = {ICI}! Approx: {(time.time() - end_time)} sec"
                )

            end_time = time.time()

            # timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S:%f')[:-3]  # Format timestamp to milliseconds
            timestamp = datetime.datetime.timestamp(
                datetime.datetime.now(datetime.timezone.utc)
            )  # POSIX timestamp
            # print("step 1", time.time() - end_time)
            time10 = time.time()
            args.angle, dB_SPL_level = audio_processor.update_das_UCA()
            # print("step 2 = update das", time.time() - end_time)
            timestamp_queue.put([timestamp, dB_SPL_level[0], args.angle])
            time40 = time.time()
            angle_temp = args.angle[0]
            level_temp = dB_SPL_level[0]
            angle_queue.put(angle_temp)
            level_queue.put(level_temp)
            angle_queue_led.put(angle_temp)
            level_queue_led.put(level_temp)
            print("\nAngle:", angle_temp, "dB SPL:", level_temp, "\n")

            if dB_SPL_level[0] >= native_cfg["trigger_level"]:
                print("REACTION:\nAngle:", angle_temp, "dB SPL:", level_temp, "\n")

            if time.time() - last_resource_report >= 1:
                # cpu_percent = process.cpu_percent(None)
                # ram_mb = process.memory_info().rss / (1024**2)
                # ram_percent = process.memory_percent()
                # cpu_freq = psutil.cpu_freq()
                # cpu_freq_text = (
                #     f"{cpu_freq.current:.0f}/{cpu_freq.max:.0f} MHz"
                #     if cpu_freq is not None
                #     else "N/A"
                # # )
                temperature_c = get_temperature_celsius()
                temperature_text = (
                    f"{temperature_c:.1f} C" if temperature_c is not None else "N/A"
                )
                throttle_text = get_pi_throttled_status()
                # print(f"CPU: | TEMP: {temperature_text} | THROTTLE: {throttle_text}")
                #     f"CPU: {cpu_percent:.1f}% | FREQ: {cpu_freq_text} | RAM: {ram_mb:.1f} MB ({ram_percent:.1f}%) | TEMP: {temperature_text} | THROTTLE: {throttle_text}"
                # )
                last_resource_report = time.time()

            # # print("step queue", time.time() - time40)

    except Exception as e:
        robot_move.stop()

        # Collect all remaining items from the queue and append to existing npy data
        remaining_data = []
        while not timestamp_queue.empty():
            remaining_data.append(timestamp_queue.get())
        if remaining_data:
            # Load existing data if file exists
            if os.path.exists(npy_data + ".npy"):
                existing_data = np.load(npy_data + ".npy", allow_pickle=True)
                # Ensure existing_data is always a list of records
                if existing_data.ndim == 1 and isinstance(
                    existing_data[0], (list, np.ndarray)
                ):
                    combined_data = np.concatenate(
                        [existing_data, remaining_data], axis=0
                    )
                else:
                    combined_data = np.array(
                        [existing_data] + remaining_data, dtype=object
                    )
            else:
                combined_data = np.array(remaining_data, dtype=object)
            np.save(npy_data, combined_data)
        print(f"\nMatrix has been saved to npy at {npy_data}\n")

        robot_move.running = False
        audio_processor.stop()
        inputstream_thread.join(timeout=1.0)

        print("\n Recording finished \n\nFilename: " + repr(args.filename) + "\n")
        move_thread.join(timeout=1.0)
        parser.exit(type(e).__name__ + ": " + str(e))

    except KeyboardInterrupt:
        robot_move.stop()
        robot_move.running = False
        audio_processor.stop()

        # Collect all remaining items from the queue and append to existing npy data
        remaining_data = []
        while not timestamp_queue.empty():
            remaining_data.append(timestamp_queue.get())
        if remaining_data:
            # Load existing data if file exists
            if os.path.exists(npy_data + ".npy"):
                existing_data = np.load(npy_data + ".npy", allow_pickle=True)
                # Ensure existing_data is always a list of records
                if existing_data.ndim == 1 and isinstance(
                    existing_data[0], (list, np.ndarray)
                ):
                    combined_data = np.concatenate(
                        [existing_data, remaining_data], axis=0
                    )
                else:
                    combined_data = np.array(
                        [existing_data] + remaining_data, dtype=object
                    )
            else:
                combined_data = np.array(remaining_data, dtype=object)
            np.save(npy_data, combined_data)
        print(f"\nMatrix has been saved to npy at {npy_data}\n")

        print("\n Recording finished \n\nFilename: " + repr(args.filename) + "\n")
        inputstream_thread.join(timeout=1.0)
        move_thread.join(timeout=1.0)
        parser.exit("KeyboardInterrupt")


if __name__ == "__main__":
    main()
