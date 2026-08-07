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
import matplotlib.pyplot as plt
import scipy.signal as signal
import sounddevice as sd
import soundfile as sf
import argparse
import queue
import datetime
import time
import os
import threading
import pandas as pd
import netifaces as ni
import hydra

from utilities import *

from AudioProcessor_circ_array import AudioProcessor
from RobotMove_circ import RobotMove
from shared_queues import angle_queue, level_queue, angle_queue_led, level_queue_led

print("imports done")

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
soundcard_index = get_card(sd.query_devices(), "ReSpeaker")
print(sd.query_devices())
print("soundcard_index=", soundcard_index)

ni.ifaddresses("wlan0")
raspi_local_ip = ni.ifaddresses("wlan0")[2][0]["addr"]
print("raspi_local_ip =", raspi_local_ip)


#######################################################################################
# MAIN FUNCTION
#######################################################################################


@hydra.main(
    version_base=None,
    config_path=".",
    config_name="config_rep",
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
    folder_name = str(raspi_local_ip)
    cfg_folder_path = os.path.join(save_path, "Data", folder_name, time1, time2)
    os.makedirs(cfg_folder_path, exist_ok=True)
    OmegaConf.save(cfg, os.path.join(cfg_folder_path, "config" + time2 + ".yaml"))

    ICI = 1 / native_cfg["call_rate"]  # inter-call interval in seconds

    input_buffer_time = (
        ICI - native_cfg["duration_out"] - 50e-3 - 50e-3 - 40e-3
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
        native_cfg["duration_out"] * 1000 + 40
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
    sensitivity_path = os.path.join(script_dir, "Knowles_SPH0645LM4H-B_sensitivity.csv")
    sensitivity = pd.read_csv(sensitivity_path)

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
    centrefreqs, freqrms = calc_native_freqwise_rms(in_sig, native_cfg["fs"])

    freqs = np.array(sensitivity.iloc[:, 0])  # first column contains frequencies
    sens_freqwise_rms = np.array(
        sensitivity.iloc[:, 1]
    )  # Last column contains sensitivity values

    interp_sensitivity = interpolate_freq_response(
        [freqs, sens_freqwise_rms], centrefreqs
    )

    frequency_band = [2e3, 20e3]  # min, max frequency to do the compensation Hz
    tgtmic_relevant_freqs = np.logical_and(
        centrefreqs >= frequency_band[0], centrefreqs <= frequency_band[1]
    )
    # Thymio movement parameters
    speed = perc_to_speed(
        native_cfg["speed_percent"]
    )  # Map 0-100 perc speed to thymio range (0-500)
    turn_speed = perc_to_speed(native_cfg["turn_speed_percent"])

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

    robot_move = RobotMove(
        speed,
        turn_speed,
        native_cfg["left_sensor_threshold"],
        native_cfg["right_sensor_threshold"],
        native_cfg["critical_level"],
        native_cfg["trigger_level"],
        ground_sensors_bool=True,
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

    time2 = startime.strftime("_%Y-%m-%d__%H-%M-%S")
    os.makedirs(folder_path, exist_ok=True)
    name = "TIMESTAMPS_" + str(raspi_local_ip) + str(time2)
    npy_data = os.path.join(folder_path, name)

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

            # print(" step 5", time.time() - time40)
            # print("Total loop time: ", time.time() - end_time)
            # add a small random jitter to avoid strict periodic locking
            wait_time = ICI - (time.time() - end_time) + np.random.uniform(0, 0.05)
            print("wait time", wait_time)
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
            # print("step 2 = update das", time.time() - time10)
            time20 = time.time()
            # print("step 3", time.time() - time20)
            timestamp_queue.put([timestamp, dB_SPL_level[0], args.angle])
            time30 = time.time()
            # print("step 4", time.time() - time30)
            time40 = time.time()
            angle_temp = args.angle[0]
            level_temp = dB_SPL_level[0]
            angle_queue.put(angle_temp)
            level_queue.put(level_temp)
            angle_queue_led.put(angle_temp)
            level_queue_led.put(level_temp)
            print("\nAngle:", angle_temp, "dB SPL:", level_temp, "\n")

            # # print("step queue", time.time() - time40)
            # if dB_SPL_level[0] >= trigger_level:

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

        inputstream_thread.join()

        print("\n Recording finished \n\nFilename: " + repr(args.filename) + "\n")
        move_thread.join()
        parser.exit(type(e).__name__ + ": " + str(e))

    except KeyboardInterrupt:
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

        print("\n Recording finished \n\nFilename: " + repr(args.filename) + "\n")
        inputstream_thread.join()
        move_thread.join()
        parser.exit("KeyboardInterrupt")
        if "led_controller" in locals():
            led_controller.stop()


if __name__ == "__main__":
    main()
