#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created: 2026-1-13
Author: Alberto Doimo
email: alberto.doimo@uni-konstanz.de

Description:

Real-time visualization of DOA estimation using Delay And Sum (DAS) method
with a Uniform Circular Array (UCA) of microphones.
"""

import numpy as np
import matplotlib.pyplot as plt
import sounddevice as sd
import scipy.signal as signal
import queue
from matplotlib.animation import FuncAnimation

# "functions" directory must be in the same path of this file
from utilities import *

# Define DAS filter function
# Constants
global fs, channels, radius, sos

c = 343.0  # speed of sound
fs = 16000  # sampling frequency
channels = 5
radius = 0.0323  # radius of the microphone array in meters

cutoff = 1000  # high-pass filter cutoff frequency in Hz
sos = signal.butter(1, cutoff, "hp", fs=fs, output="sos")


def get_card(device_list):
    for i, each in enumerate(device_list):
        if "ReSpeaker" in each["name"]:
            return i


global q
q = queue.Queue()

usb_card_index = get_card(sd.query_devices())
print(sd.query_devices())
print(f"Using device index: {usb_card_index}")


def audio_callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""

    # Fancy indexing with mapping creates a (necessary!) copy:
    q.put(indata[::1, :])


def update_das(frame):
    """Calculates DOA using Delay And Sum (DAS) method.

    Returns
    -------
        peak_angles:
            Angles of detected peaks in degrees.
        spatial_resp:
            Spatial response of the DAS beamformer.
    """

    try:
        in_sig = q.get_nowait()
    except queue.Empty:
        return None

    in_sig = signal.sosfiltfilt(sos, in_sig, axis=0)
    in_sig = in_sig[
        :, 1:5
    ]  # ReSpeaker 4-mic array: use channels 1 to 4 for the processing; ch0=summed signal from all mics, ch5 = speaker out signal

    # # Filter the input with its envelope on ref channel
    # filtered_envelope = np.abs(signal.hilbert(in_sig[:, 1], axis=0))

    # max_envelope_idx = np.argmax(filtered_envelope)

    # # Trim all channels around the max
    # trim_ms = 10  # ms
    # trim_samples = int(fs * trim_ms)
    # half_trim = trim_samples // 2
    # trimmed_signal = np.zeros((trim_samples, in_sig.shape[1]), dtype=in_sig.dtype)

    # # Ensure trimmed_signal always has exactly trim_samples rows (matching trim_ms duration)
    # if max_envelope_idx - half_trim < 0:
    #     start_idx = 0
    #     end_idx = trim_samples
    # elif max_envelope_idx + half_trim > in_sig.shape[0]:
    #     end_idx = in_sig.shape[0]
    #     start_idx = end_idx - trim_samples
    # else:
    #     start_idx = max_envelope_idx - half_trim
    #     end_idx = start_idx + trim_samples

    # trimmed_signal = in_sig[start_idx:end_idx, :]

    theta, spatial_resp, f_spec_axis, spectrum, bands = das_filter_UCA(
        in_sig,
        fs,
        4,
        radius,
        45,
        [1000, 3000],
        np.linspace(0, 360, 361),
        show=False,
    )

    peaks, _ = signal.find_peaks(spatial_resp)

    peak_angles = theta[peaks]
    N = 1  # Number of peaks to keep

    # Sort peaks by their height and keep the N largest ones
    peak_heights = spatial_resp[peaks]
    top_n_peak_indices = np.argsort(peak_heights)[-N:]
    top_n_peak_indices = top_n_peak_indices[::-1]
    peak_angles = theta[peaks[top_n_peak_indices]]

    return peak_angles, spatial_resp


#####################################333
# # --- 1. Setup the Linear Plot ---
# fig1, ax1 = plt.subplots(figsize=(10, 4))

# # Initialize plot objects
# # Your theta is defined as np.linspace(0, 360, 361) in the original function
# theta_deg = np.linspace(0, 360, 36)

# (line,) = ax1.plot([], [], lw=1, color="blue", label="Spatial Response")
# (peak_dots,) = ax1.plot([], [], "r+", markersize=5, label="Detected Peaks")

# # Configure Axes
# ax1.set_xlim(0, 360)
# ax1.set_ylim(0, 1.1)  # Adjust based on your expected spatial_resp amplitude
# ax1.set_xlabel("Angle (Degrees)")
# ax1.set_ylabel("Amplitude")
# ax1.set_title("DAS Spatial Response (Linear View)")
# ax1.grid(True, linestyle="--", alpha=0.7)
# ax1.legend(loc="upper right")

# --- 1. Setup the Plotting Environment ---
fig2 = plt.figure(figsize=(8, 8))
ax2 = fig2.add_subplot(111, projection="polar")

# Standard orientation for DOA: 0° at Top (North), Clockwise
ax2.set_theta_zero_location("N")
ax2.set_theta_direction(-1)
ax2.set_ylim(0, 1.1)  # Assuming normalized response; adjust if necessary

# Initialize plot objects
(line,) = ax2.plot([], [], lw=1.5, color="blue", label="Spatial Response")
(peak_dots,) = ax2.plot([], [], "r+", markersize=8, label="Detected Peaks")
ax2.legend(loc="lower right")


def init():
    global line, peak_dots
    line.set_data([], [])
    peak_dots.set_data([], [])
    return line, peak_dots


def animate_polar(frame):

    # It returns: (peak_angles, spatial_resp)
    result = update_das(frame)

    # Handle the case where the queue is empty or returns None
    if result is None:
        return line, peak_dots

    global peak_angles, spatial_resp
    peak_angles, spatial_resp = result
    spatial_resp = (
        spatial_resp / np.max(spatial_resp)
        if np.max(spatial_resp) > 0
        else spatial_resp
    )

    # Define theta (must match the one inside update_das)
    theta_deg = np.linspace(0, 360, 361)
    theta_rad = np.deg2rad(theta_deg)

    # Update the spatial response line
    line.set_data(theta_rad, spatial_resp)

    # Update peak dots (if peaks exist)
    if peak_angles is not None and len(peak_angles) > 0:
        # We need the magnitude at the peak angle to place the dot correctly
        # Finding indices in theta that match peak_angles
        peak_indices = [np.argmin(np.abs(theta_deg - ang)) for ang in peak_angles]
        peak_mags = spatial_resp[peak_indices]
        peak_dots.set_data(np.deg2rad(peak_angles), peak_mags)
    else:
        peak_dots.set_data([], [])

    return line, peak_dots


# def animate_lin(frame):
#     # Call your exact function
#     # result = (peak_angles, spatial_resp)
#     result = update_das(frame)

#     if result is None:
#         return line, peak_dots

#     peak_angles, spatial_resp = result
#     spatial_resp = (
#         spatial_resp / np.max(spatial_resp)
#         if np.max(spatial_resp) > 0
#         else spatial_resp
#     )

#     # Update the spatial response line (X = degrees, Y = amplitude)
#     line.set_data(theta_deg, spatial_resp)

#     # Update peak dots (if peaks were detected)
#     if peak_angles is not None and len(peak_angles) > 0:
#         # Find the amplitude at the peak angles to place the dots correctly
#         peak_indices = [np.argmin(np.abs(theta_deg - ang)) for ang in peak_angles]
#         peak_mags = spatial_resp[peak_indices]
#         peak_dots.set_data(peak_angles, peak_mags)
#     else:
#         peak_dots.set_data([], [])

#     return line, peak_dots


# --- 3. Execute Animation ---
# Start the audio stream
stream = sd.InputStream(
    device=usb_card_index,
    samplerate=fs,
    channels=channels,
    blocksize=512,
    callback=audio_callback,
)

stream.start()

# ani = FuncAnimation(fig1, animate_lin, init_func=init, interval=100, blit=True)
ani = FuncAnimation(fig2, animate_polar, init_func=init, interval=100, blit=True)

try:
    plt.show()
finally:
    stream.stop()
    stream.close()
