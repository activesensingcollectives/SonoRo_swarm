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

Parameters
----------
behaviour : {'attraction', 'repulsion', 'dynamic_movement'}
    Defines the robot movement behaviour in response to acoustic stimuli.
    Default is 'dynamic_movement'.
trigger_level : int
    Sound pressure level threshold (dB SPL) that triggers DOA computation.
critical_level : int
    Critical sound pressure level (dB SPL) for avoiding collisions.
c : int, default=343
    Speed of sound in air (m/s) at standard conditions.
fs : int, default=48000
    Audio sampling frequency (Hz).
rec_samplerate : int, default=48000
    Recording sampling frequency (Hz).
input_buffer_time : float, default=0.04
    Duration of audio buffer for DOA computation (seconds).
channels : int
    Number of microphones in the array.
mic_spacing : float, default=0.018
    Physical spacing between adjacent microphones (meters).
ref : int
    Reference microphone index (center mic for odd arrays).
method : {'DAS', 'CC'}, default='DAS'
    DOA algorithm selection. DAS uses Delay-and-Sum beamforming, CC cross correlation.
theta_das : ndarray
    Angular resolution range for DAS spectrum computation (-90 to 90 degrees).
N_peaks : int, default=1
    Number of peaks to detect in the DOA spectrum.
duration_out : float
    Duration of output chirp signal (seconds).
silence_post : int
    Post-chirp silence duration (milliseconds).
amplitude : float
    Amplitude of output chirp signal
auto_hipas_freq : int
    Highpass filter cutoff frequency (Hz), auto-calculated from array geometry.
auto_lopas_freq : int
    Lowpass filter cutoff frequency (Hz), auto-calculated from array geometry.
speed : int
    Robot forward movement speed.
turn_speed : int
    Robot rotation speed.
left_sensor_threshold : int
    Proximity ground sensor threshold for left obstacle detection.
right_sensor_threshold : int
    Proximity ground sensor threshold for right obstacle detection.
Flags
-----
recording_bool : bool, default=True
    Enable/disable audio recording to file.

Attributes
----------
angle_queue : queue.Queue
    Inter-thread queue for sharing detected angles with robot controller.
level_queue : queue.Queue
    Inter-thread queue for sharing SPL levels with robot controller.

Notes
-----
The input signals are filtered using a Butterworth HPF at hipas_freq in Hz
calculated from the array geometry.
The script uses multi-threading to handle simultaneous:
- Audio input/recording from USB mchstreamer sound card
- DOA computation
- Robot movement control
Audio processing uses the AudioProcessor class with continuous buffer
updates. Robot control is handled by the RobotMove class which implements
different behaviours based on detected acoustic parameters.
Thymio robot connectivity is established via thymiodirect library,
and network connectivity information is retrieved for data organization.

See Also
--------
AudioProcessor : Handles audio input, recording, and DOA computation
RobotMove : Controls robot movement behaviours

"""

#############################################################################
# Libraries import
#############################################################################

print("import libraries...")

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

from utilities import *

from AudioProcessor import AudioProcessor
from RobotMove import RobotMove
from shared_queues import angle_queue, level_queue

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
soundcard_index = get_card(sd.query_devices(), "MCHStreamer")  # MCHStreamer, ReSpeaker
print(sd.query_devices())
print("soundcard_index=", soundcard_index)

ni.ifaddresses("wlan0")
raspi_local_ip = ni.ifaddresses("wlan0")[2][0]["addr"]
print("raspi_local_ip =", raspi_local_ip)

##############################################################################
# EXPERIMENT PARAMETERS
##############################################################################

behaviour = "dynamic_movement"  # Options: 'attraction', 'repulsion', 'dynamic_movement'

# Parameters for the DOA algorithm
trigger_level = 70  # dB SPL
critical_level = 80  # dB SPL
c = 343  # speed of sound
fs = 48000
radius = 0.0323  # radius of the UCA in meters
shift = 45  # degrees of shift of mics wrt 0 angle

rec_samplerate = fs
input_buffer_time = 0.03  # seconds
block_size = int(
    input_buffer_time * fs
)  # used for the shared queue from which the doa is computed, not anymore for the output stream
channels = 5
mic_spacing = 0.018  # m
ref = channels // 2  # central mic in odd array as ref
# ref= 0 #left most mic as reference
critical = []

# Possible algorithms for computing DOA:CC, DAS
method = "DAS"

# Parameters for the CC algorithm
avar_theta = None
theta_values = []

# Parameters for the DAS algorithm
theta_das = np.linspace(-90, 90, 61)  # angles resolution for DAS spectrum
N_peaks = 1  # Number of peaks to detect in DAS spectrum

# Parameters for the chirp signal
duration_out = 10e-3  # Duration in seconds

if behaviour == "attraction":
    silence_post = 10  # [ms] can probably pushed to 20
elif behaviour == "repulsion":
    silence_post = 10  # [ms] can probably pushed to 20
elif behaviour == "dynamic_movement":
    silence_post = 100  # [ms] can probably pushed to 20

if behaviour == "attraction":
    amplitude = 0  # Amplitude of the chirp
elif behaviour == "repulsion":
    amplitude = 0  # Amplitude of the chirp
elif behaviour == "dynamic_movement":
    amplitude = 0.5  # Amplitude of the chirp

t = np.linspace(0, duration_out, int(fs * duration_out))
start_f, end_f = 8e3, 2e2
sweep = signal.chirp(t, start_f, t[-1], end_f)
sweep *= signal.windows.tukey(sweep.size, 0.2)
sweep *= 0.8

silence_samples_post = int(silence_post * fs / 1000)
silence_vec_post = np.zeros((silence_samples_post,))
post_silence_sig = np.concatenate((sweep, silence_vec_post))
full_sig = post_silence_sig

stereo_sig = np.hstack([full_sig.reshape(-1, 1), full_sig.reshape(-1, 1)])
data = amplitude * np.float32(stereo_sig)

out_blocksize = int(len(data))  # Length of the output signal
print("out_blocksize =", out_blocksize)

# Calculate highpass and lowpass frequencies based on the array geometry
auto_hipas_freq = int(343 / (2 * (mic_spacing * (channels - 1))))
print("HP frequency:", auto_hipas_freq)
auto_lowpas_freq = int(343 / (2 * mic_spacing))
print("LP frequency:", auto_lowpas_freq)
highpass_freq, lowpass_freq = [auto_hipas_freq, auto_lowpas_freq]

cutoff = auto_hipas_freq  # [Hz] highpass filter cutoff frequency
sos = signal.butter(1, cutoff, "hp", fs=fs, output="sos")

# Set the path for the sensitivity CSV file relative to the current script location
script_dir = os.path.dirname(os.path.abspath(__file__))
sensitivity_path = os.path.join(script_dir, "Knowles_SPH0645LM4H-B_sensitivity.csv")
sensitivity = pd.read_csv(sensitivity_path)

analyzed_buffer_time = 0.01  # Portion of the input buffer used for analysis in seconds
block_size_analyzed_buffer = int(analyzed_buffer_time * fs)

in_sig = np.zeros(
    block_size_analyzed_buffer, dtype=np.float32
)  # Initialize the buffer for the audio input stream
print("in_sig shape:", np.shape(in_sig))
centrefreqs, freqrms = calc_native_freqwise_rms(in_sig, fs)

freqs = np.array(sensitivity.iloc[:, 0])  # first column contains frequencies
sens_freqwise_rms = np.array(
    sensitivity.iloc[:, 1]
)  # Last column contains sensitivity values

interp_sensitivity = interpolate_freq_response([freqs, sens_freqwise_rms], centrefreqs)

frequency_band = [2e3, 20e3]  # min, max frequency to do the compensation Hz
tgtmic_relevant_freqs = np.logical_and(
    centrefreqs >= frequency_band[0], centrefreqs <= frequency_band[1]
)
# Thymio movement parameters

# Straight speed
speed = 150
turn_speed = 150

left_sensor_threshold = 400
right_sensor_threshold = 400

#######################################################################################
# MAIN FUNCTION
#######################################################################################

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Parse arguments and pass them to main function
    args = parser.parse_args()
    args.buffer = np.zeros((block_size, channels))

    # Set initial parameters for the audio processing
    startime = datetime.datetime.now()
    args.samplerate = fs
    args.rec_samplerate = rec_samplerate
    args.angle = 0
    args.subtype = None

    # Set general save path
    time1 = startime.strftime("%Y-%m-%d")
    save_path = os.path.dirname(__file__)
    folder_name = str(raspi_local_ip)
    folder_path = os.path.join(save_path, "Data", folder_name, time1)

    if recording_bool == True:
        # Create folder for saving recordings
        time2 = startime.strftime("_%Y-%m-%d__%H-%M-%S")
        os.makedirs(folder_path, exist_ok=True)

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
        # Create folder for saving timestamps
        time2 = startime.strftime("_%Y-%m-%d__%H-%M-%S")
        os.makedirs(folder_path, exist_ok=True)
        folder_path_data = os.path.join(save_path, "Data", folder_name, time1)
        os.makedirs(folder_path_data, exist_ok=True)

        name = "TIMESTAMPS_" + str(raspi_local_ip) + str(time2) + ".csv"
        args.timestamp_filename = os.path.join(folder_path_data, name)

    # Create instances of the AudioProcessor and RobotMove classes
    audio_processor = AudioProcessor(
        fs,
        channels,
        radius,
        shift,
        block_size,
        analyzed_buffer_time,
        data,
        args,
        trigger_level,
        critical_level,
        mic_spacing,
        ref,
        highpass_freq,
        lowpass_freq,
        theta_das,
        N_peaks,
        soundcard_index,
        args.subtype,
        interp_sensitivity,
        tgtmic_relevant_freqs,
        args.filename,
        args.rec_samplerate,
        sos,
    )

    robot_move = RobotMove(
        speed,
        turn_speed,
        left_sensor_threshold,
        right_sensor_threshold,
        critical_level,
        trigger_level,
        ground_sensors_bool=False,
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
    if behaviour == "attraction":
        attraction_thread = threading.Thread(
            target=robot_move.attraction_only, daemon=True
        )
        attraction_thread.start()
    elif behaviour == "repulsion":
        repulsion_thread = threading.Thread(
            target=robot_move.repulsion_only, daemon=True
        )
        repulsion_thread.start()
    elif behaviour == "dynamic_movement":
        move_thread = threading.Thread(target=robot_move.audio_move_360, daemon=True)
        move_thread.start()
    else:
        print("No valid behaviour provided")

    event = threading.Event()

    time2 = startime.strftime("_%Y-%m-%d__%H-%M-%S")
    os.makedirs(folder_path, exist_ok=True)
    name = "TIMESTAMPS_" + str(raspi_local_ip) + str(time2)
    npy_data = os.path.join(folder_path, name)

    # timestamp_queue.put([audio_processor.ts_queue.get(), 0, 'recording_start'])
    timestamp_queue.put([audio_processor.ts_queue.get(), 0, 0, "recording_start"])
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
                samplerate=fs,
                blocksize=0,
                device=soundcard_index,
                channels=2,
                callback=audio_processor.callback_out,
                finished_callback=event.set,
                latency="low",
            ) as out_stream:
                with out_stream:
                    event.wait()

                    if method == "CC":
                        time.sleep(input_buffer_time * 2.3)

            if method == "DAS":
                # timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S:%f')[:-3]  # Format timestamp to milliseconds
                timestamp = datetime.datetime.timestamp(
                    datetime.datetime.now(datetime.timezone.utc)
                )  # POSIX timestamp
                args.angle, dB_SPL_level = audio_processor.update_das()
                timestamp_queue.put(
                    [timestamp, dB_SPL_level[0], args.angle]
                )  # Put the timestamp in the queue (no block=False, keeps all values)
                # args.angle, dB_SPL_level = audio_processor.update_das_UCA()
                # #!!!! USING NOT PARAMETRIC VALUES INSIDE update_das_UCA() !!!!!

                angle_queue.put(args.angle)
                level_queue.put(dB_SPL_level)
                print("Angle:", args.angle, "dB SPL:", dB_SPL_level[0])

            elif method == "CC":
                # timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S:%f')[:-3]  # Format timestamp to milliseconds
                timestamp = datetime.datetime.timestamp(
                    datetime.datetime.now(datetime.timezone.utc)
                )  # POSIX timestamp

                args.angle, dB_SPL_level = audio_processor.update_das()
                timestamp_queue.put(
                    [timestamp, dB_SPL_level[0], args.angle]
                )  # Put the timestamp in the queue (no block=False, keeps all values)
                np.save(
                    npy_data,
                    np.array([timestamp, dB_SPL_level[0], args.angle], dtype=object),
                )
                args.angle, dB_SPL_level = audio_processor.update_CC()
                angle_queue.put(args.angle)
                level_queue.put(dB_SPL_level)

                if isinstance(args.angle, (int, float, np.number)):
                    if np.isnan(args.angle):
                        angle_queue.put(None)

            else:
                print("No valid method provided")
                robot_move.stop()

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
        attraction_thread.join()
        repulsion_thread.join()
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
        attraction_thread.join()
        repulsion_thread.join()
        parser.exit("KeyboardInterrupt")
