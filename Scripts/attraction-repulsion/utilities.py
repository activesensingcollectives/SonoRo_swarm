#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created: 2025-12-16
Author: Alberto Doimo
email: alberto.doimo@uni-konstanz.de

Description
Utility functions used to run SonoRo_swarm.py

"""

import numpy as np
import scipy.signal as signal
from scipy.interpolate import interp1d
from scipy.signal import stft, hilbert
import time
import os
import matplotlib.pyplot as plt

# Audio processing functions


def pascal_to_dbspl(X):
    """Converts Pascals to dB SPL re 20 uPa"""
    return dB(X / 20e-6)


def rms(X):
    """Calculates the root mean square of the input signal X

    Parameters
    ----------
    X : np.array
        Input signal

    Returns
    -------
    rms_value :
        The RMS value of the input signal

    """
    return np.sqrt(np.mean(X**2))


def calc_dBrms(one_channel_buffer):
    """Calculate the dB RMS of a single channel buffer.

    Parameters
    -----------
    one_channel_buffer : 1D np.array.
        Single channel buffer.

    Returns
    -----------
    dB_rms : float.
        dB RMS value of the buffer.

    """
    squared = np.square(one_channel_buffer)
    mean_squared = np.mean(squared)
    root_mean_squared = np.sqrt(mean_squared)
    # print('rms',root_mean_squared)
    try:
        dB_rms = 20.0 * np.log10(root_mean_squared)
    except ValueError:
        dB_rms = -np.inf
    return dB_rms


dB = lambda X: 20 * np.log10(abs(np.array(X).flatten()))

db_to_linear = lambda X: 10 ** (X / 20)


# bin width clarification https://stackoverflow.com/questions/10754549/fft-bin-width-clarification
def get_rms_from_fft(freqs, spectrum, **kwargs):
    """Use Parseval's theorem to get the RMS level of each frequency component
    This only works for RFFT spectrums!!!

    Parameters
    ----------
    freqs : (Nfreqs,) np.array >0 values
    spectrum : (Nfreqs,) np.array (complex)
    freq_range : (2,) array-like
        Min and max values

    Returns
    -------
    root_mean_squared : float
        The RMS of the signal within the min-max frequency range

    """
    minfreq, maxfreq = kwargs["freq_range"]
    relevant_freqs = np.logical_and(freqs >= minfreq, freqs <= maxfreq)
    spectrum_copy = spectrum.copy()
    spectrum_copy[~relevant_freqs] = 0
    mean_sigsquared = np.sum(abs(spectrum_copy) ** 2) / spectrum.size
    root_mean_squared = np.sqrt(mean_sigsquared / (2 * spectrum.size - 1))
    return root_mean_squared


def calc_native_freqwise_rms(X, fs):
    """Converts the FFT spectrum into a band-wise rms output.
    The frequency-resolution of the spectrum/audio size decides
    the frequency resolution in general.

    Parameters
    ----------
    X : np.array
        Audio
    fs : int
        Sampling rate in Hz

    Returns
    -------
    fftfreqs, freqwise_rms : np.array
        fftfreqs holds the frequency bins from the RFFT
        freqwise_rms is the RMS value of each frequency bin.

    """
    rfft = np.fft.rfft(X)
    fftfreqs = np.fft.rfftfreq(X.size, 1 / fs)
    # now calculate the rms per frequency-band
    abs_rfft_squared = np.abs(rfft) ** 2
    mean_sq_freq = abs_rfft_squared / rfft.size
    freqwise_rms = np.sqrt(mean_sq_freq / (2 * rfft.size - 1))

    return fftfreqs, freqwise_rms


def interpolate_freq_response(mic_freq_response, new_freqs):
    """
    Parameters
    ----------
    mic_freq_response : tuple/list
        A tuple/list with two entries: (centrefreqs, centrefreq_RMS).

    new_freqs : list/array-like
        A set of new centre frequencies that need to be interpolated to.

    Returns
    -------
    tgtmicsens_interp :

    Note
    ---------
    Any frequencies outside of the calibration range will automatically be
    assigned to the lowest sensitivity values measured in the input centrefreqs

    """
    centrefreqs, mic_sensitivity = mic_freq_response
    tgtmic_sens_interpfn = interp1d(
        centrefreqs,
        mic_sensitivity,
        kind="cubic",
        bounds_error=False,
        fill_value=np.min(mic_sensitivity),
    )
    # interpolate the sensitivity of the mic to intermediate freqs
    tgtmicsens_interp = tgtmic_sens_interpfn(new_freqs)
    return tgtmicsens_interp


def bandpass(rec_buffer, highpass_freq, lowpass_freq, fs):
    """Applies a bandpass filter to the input buffer.

    Parameters
    ----------
    rec_buffer : ndarray
        Input buffer to be filtered.
    highpass_freq : float
        Highpass frequency in Hz.
    lowpass_freq : float
        Lowpass frequency in Hz.
    fs : float
        Sampling frequency in Hz.

    Returns
    -------
    rec_buffer_bp : ndarray
        Bandpass filtered buffer with the same shape as the input.

    """
    nyq_freq = fs / 2.0
    b, a = signal.butter(
        4, [highpass_freq / nyq_freq, lowpass_freq / nyq_freq], btype="bandpass"
    )  # to be 'allowed' in Hz.
    rec_buffer_bp = np.apply_along_axis(
        lambda X: signal.lfilter(b, a, X), 0, rec_buffer
    )
    return rec_buffer_bp


def matched_filter(recording, chirp_template):
    """Apply a matched filter to the signal using the provided template.

    Parameters
    -----------
    recording : np.ndarray
        The audio signal to be filtered.
    chirp_template : np.ndarray
        The chirp template used for filtering.

    Returns
    --------
    np.ndarray
        The filtered output, which is the envelope of the matched filter response.

    """
    # Apply correlation and shift
    filtered_output = np.roll(
        signal.correlate(recording, chirp_template, "same", method="direct"),
        -len(chirp_template) // 2,
    )
    # Apply Tukey window
    filtered_output *= signal.windows.tukey(filtered_output.size, 0.1)
    # Get the envelope using Hilbert transform
    filtered_envelope = np.abs(signal.hilbert(filtered_output))

    return filtered_envelope


def detect_peaks(filtered_output, sample_rate, prominence, distance):
    """Detect peaks in the matched filter output.

    Parameters
    ----------
    filtered_output : np.ndarray
        The output of the matched filter.
    sample_rate : int
        The sample rate of the audio signal.
    prominence : float
        Minimum prominence of peaks to be detected.
    distance : float
        Minimum distance between peaks in seconds.

    Returns
    -------
    peaks : np.ndarray
        Indices of detected peaks in the filtered output.

    """
    peaks, properties = signal.find_peaks(
        filtered_output, prominence=prominence, distance=distance * sample_rate
    )
    return peaks


def find_peaks_circular(mag_p, wrap=3):
    """
    Circular-aware peak detection: wraps `wrap` samples from each end
    around the array so peaks at the 0/360 boundary are correctly
    detected by scipy's find_peaks (which cannot flag edge elements).
    """
    n = len(mag_p)
    # Extend array: [last `wrap` samples] + [full array] + [first `wrap` samples]
    extended = np.concatenate([mag_p[-wrap:], mag_p, mag_p[:wrap]])

    peaks_ext, _ = signal.find_peaks(extended)

    # Map back to original indices, keeping only peaks in the "real" middle region,
    # but converting wrapped detections back to their true original index
    peaks = []
    for p in peaks_ext:
        real_idx = p - wrap
        if 0 <= real_idx < n:
            peaks.append(real_idx)
        elif real_idx < 0:  # wrapped from the end
            peaks.append(real_idx + n)
        elif real_idx >= n:  # wrapped from the start
            peaks.append(real_idx - n)

    return np.unique(peaks)


def pow_two_pad_and_window(vec, fs, show=True):
    """Pad a vector with zeros to the next power of two and apply a Tukey window.

    Parameters
    ----------
    vec : np.ndarray
        Input vector
    fs : float
        Sampling rate in Hz
    show : bool, optional
        If True, plot the windowed vector and its spectrogram (default: True)

    Returns
    -------
    padded_windowed_vec : np.ndarray
        Padded and windowed vector normalized by its maximum value

    """
    window = signal.windows.tukey(len(vec), alpha=0.2)
    windowed_vec = vec * window
    padded_windowed_vec = np.pad(
        windowed_vec,
        (0, 2 ** int(np.ceil(np.log2(len(windowed_vec)))) - len(windowed_vec)),
    )

    if show:
        dur = len(padded_windowed_vec) / fs
        t = np.linspace(0, dur, len(windowed_vec))
        plt.figure()
        plt.title("Windowed Vector and Spectrogram")
        plt.subplot(2, 1, 1)
        plt.plot(t, windowed_vec)
        plt.subplot(2, 1, 2)
        plt.specgram(windowed_vec, NFFT=256, Fs=fs)
        plt.show()

    return padded_windowed_vec / max(padded_windowed_vec)


# Direction of Arrival estimation functions


def avar_angle(delay_set, nchannels, mic_spacing, ref_channel):
    """Calculate the mean angle of arrival to the array with respect to reference channel.

    Parameters
    ----------
    delay_set : array-like
        The time delay between signals
    nchannels : int
        Number of mics in the array
    mic_spacing : float
        Inter-distance between the mics
    ref_channel : int
        Reference channel

    Returns
    -------
    avar_theta : float
        The mean angle of arrival to the array in radians

    """
    theta = []
    if ref_channel != 0:  # centered reference that works with odd mics
        for each in range(0, nchannels // 2):
            theta.append(
                -np.arcsin(
                    (delay_set[each] * 343) / ((nchannels // 2 - each) * mic_spacing)
                )
            )  # rad
            i = nchannels // 2 - each
        for each in range(nchannels // 2, nchannels - 1):
            theta.append(
                np.arcsin((delay_set[each] * 343) / ((i) * mic_spacing))
            )  # rad
            i += 1
    else:
        for each in range(0, nchannels - 1):
            theta.append(
                np.arcsin((delay_set[each] * 343) / ((each + 1) * mic_spacing))
            )  # rad
    avar_theta = np.mean(theta)

    return avar_theta


def calc_delay(two_ch, fs):
    """Calculate the delay between two channels using cross-correlation.

    Parameters
    ----------
    two_ch : (Nsamples, 2) np.array
        Input audio buffer with 2 channels
    fs : float
        Sampling frequency in Hz

    Returns
    -------
    delay : float
        The time-delay in seconds between the arriving audio across the
        channels

    """
    for each_column in range(2):
        two_ch[:, each_column] = two_ch[:, each_column]

    cc = np.correlate(two_ch[:, 0], two_ch[:, 1], "same")
    midpoint = cc.size / 2.0
    delay = np.argmax(cc) - midpoint
    # convert delay to seconds
    delay *= 1 / float(fs)
    return delay


def calc_multich_delays(multich_audio, ref_sig, fs, ref_channel):
    """Calculate delays between channels using GCC-PHAT with a reference signal.

    Parameters
    ----------
    multich_audio : (Nsamples, Nchannels) np.ndarray
        Multichannel audio signal
    ref_sig : (Nsamples,) np.ndarray
        Reference signal
    fs : float
        Sampling frequency in Hz
    ref_channel : int
        Reference channel index

    Returns
    -------
    delay_set : np.ndarray
        Array of time delays in seconds for each channel relative to reference

    """
    nchannels = multich_audio.shape[1]
    delay_set = []
    i = 0
    while i < nchannels:
        if i != ref_channel:
            delay_set.append(
                gcc_phat(multich_audio[:, i], ref_sig, fs)
            )  # gcc phat correlation
            i += 1
        else:
            i += 1
            pass

    return np.array(delay_set)


def check_if_above_level(mic_inputs, trigger_level, critical_level):
    """Checks if the dB rms level of the input recording buffer is above
    threshold. If any of the microphones are above the given level then
    recording is initiated.

    Parameters
    ----------
    mic_inputs : (Nsamples, Nchannels) np.ndarray
        Data from soundcard
    trigger_level : float
        If the input data buffer has a dB rms >= this value then True will be returned
    critical_level : float
        Critical level threshold

    Returns
    -------
    trigger_bool : bool
        True if the buffer dB rms is >= the trigger_level
    critical_bool : bool
        True if the buffer dB rms is >= the critical_level
    dBrms_channel : np.ndarray
        dB RMS values for each channel

    """
    dBrms_channel = np.apply_along_axis(calc_dBrms, 0, mic_inputs)
    trigger_bool = np.any(dBrms_channel >= trigger_level)
    critical_bool = np.any(dBrms_channel >= critical_level)
    if critical_bool:
        trigger_bool = False
    max_dBrms = np.max(dBrms_channel)

    return (trigger_bool, critical_bool, dBrms_channel)


def gcc_phat(sig, refsig, fs):
    """Compute the cross-correlation between two signals using GCC-PHAT.

    Parameters
    ----------
    sig : (Nsamples, Nchannels) np.ndarray
        First signal
    refsig : (Nsamples,) np.ndarray
        Reference signal
    fs : float
        Sampling frequency in Hz

    Returns
    -------
    float
        Time delay in seconds between the two signals

    """
    n = sig.shape[0] + refsig.shape[0]
    SIG = np.fft(sig, n=n)
    REFSIG = np.fft(refsig, n=n)
    R = SIG * np.conj(REFSIG)
    cc = np.fft.ifft(R / np.abs(R))
    max_shift = int(np.floor(n / 2))
    cc = np.concatenate((cc[-max_shift:], cc[: max_shift + 1]))
    shift = np.argmax(np.abs(cc)) - max_shift

    return -shift / float(fs)  # time delay


def music(
    y, fs, nch, d, bw, theta=np.linspace(-90, 90, 73), c=343, wlen=64, ns=1, show=False
):
    """Simple multiband MUltiple SIgnal Classification spatial filter implementation.

    Parameters
    ----------
    y : (Nsamples, Nchannels) np.ndarray
        Mic array signals
    fs : float
        Sampling rate in Hz
    nch : int
        Number of mics in the array
    d : float
        Mic spacing in meters
    bw : tuple
        Frequency band (low_freq, high_freq) in Hz
    theta : np.ndarray, optional
        Angle axis in degrees. Defaults to 2.5[deg] resolution
    c : float, optional
        Sound speed in m/s. Defaults to 343
    wlen : int, optional
        Window length for STFT. Defaults to 64
    ns : int, optional
        Expected number of sources. Defaults to 1
    show : bool, optional
        Plot the pseudospectrum for each band. Defaults to False

    Returns
    -------
    theta : np.ndarray
        Angle axis in degrees
    mag_p : np.ndarray
        Magnitude of spatial energy distribution estimate, averaged across bands

    """
    f_spec_axis, _, spectrum = stft(
        y, fs=fs, window=np.ones((wlen,)), nperseg=wlen, noverlap=wlen - 1, axis=0
    )
    bands = f_spec_axis[(f_spec_axis >= bw[0]) & (f_spec_axis <= bw[1])]
    p = np.zeros_like(theta, dtype=complex)
    p_i = np.zeros((len(theta), 1), dtype=complex)

    for f_c in bands:
        w_s = 2 * np.pi * f_c * d * np.sin(np.deg2rad(theta)) / c
        a = np.exp(np.outer(np.linspace(0, nch - 1, nch), -1j * w_s))
        a_H = a.T.conj()
        spec = spectrum[f_spec_axis == f_c, :, :].squeeze()
        cov_est = np.cov(spec, bias=True)
        lambdas, V = np.linalg.eig(cov_est)
        indices = np.argsort(lambdas)[::-1]
        V_sorted = V[indices]
        V_n = V_sorted[:, ns:]
        V_n_H = V_n.T.conj()
        for i, _ in enumerate(theta):
            p_i[i] = 1 / (a_H[i, :] @ V_n @ V_n_H @ a[:, i])
            p[i] += p_i[i]
        if show:
            plt.figure()
            plt.polar(np.deg2rad(theta), 20 * np.log10(np.abs(p_i)))
            plt.xlim((-np.pi / 2, np.pi / 2))
            plt.title(str(f_c))
            plt.show()
    mag_p = np.abs(p) / len(bands)

    return theta, mag_p


def das_filter(y, fs, nch, d, bw, theta, c=343, wlen=64, show=False):
    """Simple multiband Delay and Sum spatial filter implementation.

    Parameters
    ----------
    y : (Nsamples, Nchannels) np.ndarray
        Mic array signals
    fs : float
        Sampling rate in Hz
    nch : int
        Number of mics in the array
    d : float
        Mic spacing in meters
    bw : tuple
        Frequency band (low_freq, high_freq) in Hz
    theta : np.ndarray
        Angle vector in degrees
    c : float, optional
        Sound speed in m/s. Defaults to 343
    wlen : int, optional
        Window length for STFT. Defaults to 64
    show : bool, optional
        Plot the pseudospectrum for each band. Defaults to False

    Returns
    -------
    theta : np.ndarray
        Angle axis in degrees
    mag_p : np.ndarray
        Magnitude of average spatial energy distribution estimation across bands
    f_spec_axis : np.ndarray
        Frequency axis from STFT
    spectrum : np.ndarray
        STFT spectrum
    bands : np.ndarray
        Frequency bands within the specified bandwidth

    """
    f_spec_axis, _, spectrum = stft(
        y, fs=fs, window=np.ones((wlen,)), nperseg=wlen, noverlap=wlen - 1, axis=0
    )
    bands = f_spec_axis[(f_spec_axis >= bw[0]) & (f_spec_axis <= bw[1])]
    p = np.zeros_like(theta, dtype=complex)
    p_i = np.zeros_like(theta, dtype=complex)
    band_idxs = np.where((f_spec_axis >= bw[0]) & (f_spec_axis <= bw[1]))[0]
    nch_range = np.linspace(
        nch - 1, 0, nch
    )  # np.linspace( nch-1,0, nch) = -90>0>90; np.linspace( 0,nch-1, nch) = 90>0>-90
    theta_rad = np.deg2rad(theta)

    # Spatial frequencies matrix
    w_s_mat = (
        2 * np.pi * bands[:, None] * d * np.sin(theta_rad) / c
    )  # shape: (n_bands, len(theta))

    # propagation vector (or steering vector) matrix
    a_mat = np.exp(np.outer(nch_range, -1j * w_s_mat.ravel())).reshape(
        nch, len(bands), len(theta)
    )  # (nch, n_bands, len(theta))

    # Hermitian transpose of a_mat
    a_H_mat = np.conj(a_mat.transpose(1, 2, 0))  # (n_bands, len(theta), nch)

    for b_idx, f_idx in enumerate(band_idxs):
        spec = spectrum[f_idx, :, :]  # (nch, n_frames)
        # print("spec shape", spectrum.shape)
        cov_est = np.cov(spec, bias=True)
        a = a_mat[:, b_idx, :]  # (nch, len(theta))
        a_H = a_H_mat[b_idx, :, :]  # (len(theta), nch)
        p_i = np.einsum("ij,jk,ki->i", a_H, cov_est, a) / (nch**2)
        p += p_i

        if show:
            ax.plot(
                theta_rad, 10 * np.log10(np.abs(p_i)), label=f"{bands[b_idx]:.1f} Hz"
            )
    mag_p = np.abs(p) / len(bands)

    if show:
        fig, ax = plt.subplots(subplot_kw={"projection": "polar"})
        ax.set_xlim((-np.pi / 2, np.pi / 2))
        plt.ylabel("Magnitude (dB)")
        ax.set_title("Pseudospectra DAS")
        ax.set_theta_offset(np.pi / 2)
        plt.legend()
        # plt.savefig("das_pseudospectra.png")
        plt.show()

    return theta, mag_p, f_spec_axis, spectrum, bands


def das_filter_UCA(y, fs, nch, r, shift, bw, theta, c=343, wlen=64, show=False):
    """
    Multiband Delay and Sum for a Uniform Circular Array (UCA).

    Parameters:
    r : float
        Radius of the circular array in meters.
    theta : np.ndarray
        Scanning angles (azimuth) in degrees [0, 360].
    """
    # 1. Spectral Decomposition
    f_spec_axis, _, spectrum = stft(
        y, fs=fs, window=np.ones((wlen,)), nperseg=wlen, noverlap=wlen - 1, axis=0
    )

    band_idxs = np.where((f_spec_axis >= bw[0]) & (f_spec_axis <= bw[1]))[0]
    bands = f_spec_axis[band_idxs]
    theta_rad = np.deg2rad(theta)

    # 2. UCA Geometry Logic
    # Angular positions of the sensors in the array
    shift_rad = np.deg2rad(shift)
    phi_n = np.linspace(2 * np.pi, 0, nch, endpoint=False) + shift_rad

    # Coordinates of microphones
    mic_x = r * np.cos(phi_n)
    mic_y = r * np.sin(phi_n)

    # 3. Steering Vector Matrix (a_mat)
    # The phase shift for UCA is exp(j * k * r * cos(theta - phi_n))
    # Shape: (nch, n_bands, n_theta)
    a_mat = np.zeros((nch, len(bands), len(theta)), dtype=complex)

    for b_idx, f in enumerate(bands):
        k = 2 * np.pi * f / c  # Wavenumber
        for t_idx, t_start in enumerate(theta_rad):
            # Projection of the plane wave onto the circular geometry
            phase = k * (mic_x * np.cos(t_start) + mic_y * np.sin(t_start))
            a_mat[:, b_idx, t_idx] = np.exp(1j * phase)

    # Hermitian transpose for the beamforming calculation
    a_H_mat = np.conj(a_mat.transpose(1, 2, 0))  # (n_bands, n_theta, nch)

    # 4. Spatial Filtering
    p = np.zeros_like(theta, dtype=complex)

    if show:
        fig, ax = plt.subplots()

    for b_idx, f_idx in enumerate(band_idxs):
        spec = spectrum[f_idx, :, :]  # (bands, nch, n_frames(time))
        cov_est = np.cov(spec, bias=True)
        # print("spec shape", spec.shape)

        a = a_mat[:, b_idx, :]  # (nch, n_theta)
        a_H = a_H_mat[b_idx, :, :]  # (n_theta, nch)

        # P = a^H * R * a / N^2
        p_i = np.einsum("ij,jk,ki->i", a_H, cov_est, a) / (nch**2)
        p += p_i

        if show:
            linestyles = ["-", "--", "-.", ":"]
            ax.plot(
                np.rad2deg(theta_rad),
                np.abs(p_i) + 1e-12,
                label=f"{bands[b_idx]:.1f} Hz",
                linestyle=linestyles[b_idx % len(linestyles)],
            )

    mag_p = np.abs(p) / len(bands)

    if show:
        ax.set_title("UCA DAS Pseudospectra")
        ax.plot(
            np.rad2deg(theta_rad),
            mag_p + 1e-12,
            label=f"full spectrum",
            linewidth=4,
            color="black",
        )
        # ax.set_theta_direction(-1)  # Clockwise
        # ax.set_theta_offset(np.pi / 2)  # North is 0 degrees
        plt.grid()
        ax.set_xlabel("Angle (Degrees)")
        ax.set_ylabel("Magnitude (dB)")
        plt.legend()
        plt.show()

    if show:
        plt.figure()
        plt.title(f"Spectrum at f={bands[8]:.1f} Hz")
        plt.imshow(np.abs(spectrum[8, :, :]), aspect="auto", origin="lower")
        plt.colorbar(label="Magnitude")
        plt.xlabel("Frames")
        plt.ylabel("Channels")
        plt.show()

    return theta, mag_p, f_spec_axis, spectrum, bands


def das_filter_CA(y, fs, nch, r, shift, bw, theta, c=343, wlen=64, show=False):
    """
    Multiband Delay and Sum for a Uniform Circular Array (UCA).

    Parameters:
    r : float
        Radius of the circular array in meters.
    theta : np.ndarray
        Scanning angles (azimuth) in degrees [0, 360].
    """
    # 1. Spectral Decomposition
    f_spec_axis, _, spectrum = stft(
        y, fs=fs, window=np.ones((wlen,)), nperseg=wlen, noverlap=wlen - 1, axis=0
    )

    band_idxs = np.where((f_spec_axis >= bw[0]) & (f_spec_axis <= bw[1]))[0]
    bands = f_spec_axis[band_idxs]
    theta_rad = np.deg2rad(theta)

    # 2. UCA Geometry Logic
    # Angular positions of the sensors in the array
    shift_rad = np.deg2rad(shift)
    custom_angles = np.deg2rad(
        [0, 90, 150, 210, 300, 330]
    )  # Custom angles for 5-mic circular array
    # phi_n = np.array(custom_angles) + shift_rad
    phi_n = np.linspace(0, 2 * np.pi, nch, endpoint=False) + shift_rad

    # Coordinates of microphones
    mic_x = r * np.cos(phi_n)
    mic_y = r * np.sin(phi_n)

    # 3. Steering Vector Matrix (a_mat)
    # The phase shift for UCA is exp(j * k * r * cos(theta - phi_n))
    # Shape: (nch, n_bands, n_theta)
    a_mat = np.zeros((nch, len(bands), len(theta)), dtype=complex)

    for b_idx, f in enumerate(bands):
        k = 2 * np.pi * f / c  # Wavenumber
        for t_idx, t_start in enumerate(theta_rad):
            # Projection of the plane wave onto the circular geometry
            phase = k * (mic_x * np.cos(t_start) + mic_y * np.sin(t_start))
            a_mat[:, b_idx, t_idx] = np.exp(1j * phase)

    # Hermitian transpose for the beamforming calculation
    a_H_mat = np.conj(a_mat.transpose(1, 2, 0))  # (n_bands, n_theta, nch)

    # 4. Spatial Filtering
    p = np.zeros_like(theta, dtype=complex)

    if show:
        fig, ax = plt.subplots(subplot_kw={"projection": "polar"})

    for b_idx, f_idx in enumerate(band_idxs):
        spec = spectrum[f_idx, :, :]  # (nch, n_frames)
        cov_est = np.cov(spec, bias=True)

        a = a_mat[:, b_idx, :]  # (nch, n_theta)
        a_H = a_H_mat[b_idx, :, :]  # (n_theta, nch)

        # P = a^H * R * a / N^2
        p_i = np.einsum("ij,jk,ki->i", a_H, cov_est, a) / (nch**2)
        p += p_i

        if show:
            ax.plot(
                theta_rad,
                10 * np.log10(np.abs(p_i) + 1e-12),
                label=f"{bands[b_idx]:.1f} Hz",
            )

    mag_p = np.abs(p) / len(bands)

    if show:
        ax.set_title("UCA DAS Pseudospectra")
        ax.set_theta_direction(-1)  # Clockwise
        ax.set_theta_offset(np.pi / 2)  # North is 0 degrees
        plt.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1))
        plt.show()

    return theta, mag_p, f_spec_axis, spectrum, bands


# Other functions


def get_card(device_list, device_name):
    """Get the index of the ASIO card in the device list.

    Parameters
    ----------
    device_list : list
        List of devices (usually from sd.query_devices())

    Returns
    -------
    int or None
        Index of the MCHStreamer card in the device list, or None if not found

    """
    for i, each in enumerate(device_list):
        dev_name = each["name"]
        name = device_name in dev_name
        if name:
            return i
    return None


def dmas_filter_UCA(y, fs, nch, r, shift, bw, theta, c=343, wlen=64, show=False):
    """
    Multiband Delay Multiply and Sum (DMAS) for a Uniform Circular Array (UCA).

    Parameters:
    r : float
        Radius of the circular array in meters.
    theta : np.ndarray
        Scanning angles (azimuth) in degrees [0, 360].
    """
    # 1. Spectral Decomposition
    f_spec_axis, _, spectrum = stft(
        y, fs=fs, window=np.ones((wlen,)), nperseg=wlen, noverlap=wlen - 1, axis=0
    )

    band_idxs = np.where((f_spec_axis >= bw[0]) & (f_spec_axis <= bw[1]))[0]
    bands = f_spec_axis[band_idxs]
    theta_rad = np.deg2rad(theta)

    # 2. UCA Geometry Logic
    # Angular positions of the sensors in the array
    shift_rad = np.deg2rad(shift)
    phi_n = np.linspace(2 * np.pi, 0, nch, endpoint=False) + shift_rad

    # Coordinates of microphones
    mic_x = r * np.cos(phi_n)
    mic_y = r * np.sin(phi_n)

    # 3. Steering Vector Matrix (a_mat)
    # The phase shift for UCA is exp(j * k * r * cos(theta - phi_n))
    # Shape: (nch, n_bands, n_theta)
    a_mat = np.zeros((nch, len(bands), len(theta)), dtype=complex)

    for b_idx, f in enumerate(bands):
        k = 2 * np.pi * f / c  # Wavenumber
        for t_idx, t_start in enumerate(theta_rad):
            # Projection of the plane wave onto the circular geometry
            phase = k * (mic_x * np.cos(t_start) + mic_y * np.sin(t_start))
            a_mat[:, b_idx, t_idx] = np.exp(1j * phase)

    # Hermitian transpose for the beamforming calculation
    a_H_mat = np.conj(a_mat.transpose(1, 2, 0))  # (n_bands, n_theta, nch)

    # 4. Spatial Filtering
    p = np.zeros_like(theta, dtype=complex)

    if show:
        fig, ax = plt.subplots()

    for b_idx, f_idx in enumerate(band_idxs):
        spec = spectrum[f_idx, :, :]  # (nch, n_frames)
        cov_est = np.cov(spec, bias=True)

        # --- DMAS MODIFICATIONS START ---
        # 1. Take the square root of the magnitude while preserving the complex phase
        cov_dmas = np.sqrt(np.abs(cov_est)) * (cov_est / (np.abs(cov_est)) + 1e-12)

        # print("cov_est", np.sqrt(np.abs(cov_est)))
        # print("cov_dmas", cov_dmas)

        # 2. Remove auto-correlations (main diagonal) to improve spatial resolution
        np.fill_diagonal(cov_dmas, 0)

        # 3. Adjust normalization factor for the missing diagonal terms
        num_cross_terms = (nch**2) - nch
        # --- DMAS MODIFICATIONS END ---

        a = a_mat[:, b_idx, :]  # (nch, n_theta)
        a_H = a_H_mat[b_idx, :, :]  # (n_theta, nch)

        # P = a^H * R_dmas * a / (N^2 - N)
        p_i = np.einsum("ij,jk,ki->i", a_H, cov_dmas, a) / num_cross_terms
        p += p_i

        if show:
            linestyles = ["-", "--", "-.", ":"]
            ax.plot(
                np.rad2deg(theta_rad),
                np.abs(p_i) + 1e-12,
                label=f"{bands[b_idx]:.1f} Hz",
                linestyle=linestyles[b_idx % len(linestyles)],
            )

    mag_p = np.abs(p) / len(bands)

    if show:
        ax.set_title("UCA DMAS Pseudospectra")
        ax.plot(
            np.rad2deg(theta_rad),
            mag_p + 1e-12,
            label=f"full spectrum",
            linewidth=4,
            color="black",
        )
        plt.grid()
        ax.set_xlabel("Angle (Degrees)")
        ax.set_ylabel("Magnitude (dB)")
        plt.legend()
        plt.show()

    return theta, mag_p, f_spec_axis, spectrum, bands


def uca_steering_vectors(fs, nch, r, shift, bw, theta, c=343, wlen=64):
    """Precompute the UCA steering vectors used by higher_order_dmas_UCA.

    They depend only on the array geometry and the analysis settings, so they can be
    computed once at start-up instead of on every call.

    Parameters
    ----------
    fs : int
        Sampling rate in Hz.
    nch : int
        Number of microphones in the circular array.
    r : float
        Radius of the circular array in meters.
    shift : float
        Angular offset of the first microphone in degrees.
    bw : (2,) array-like
        Min and max frequency (Hz) of the analysed bands.
    theta : np.ndarray
        Scanning angles (azimuth) in degrees.
    c : float, optional
        Speed of sound in m/s.
    wlen : int, optional
        STFT window length (must match the one used in higher_order_dmas_UCA).

    Returns
    -------
    band_idxs : np.ndarray
        Indices of the STFT frequency bins inside bw.
    bands : np.ndarray
        Frequencies (Hz) of those bins.
    a_mat : np.ndarray
        Steering vectors, shape (nch, n_bands, n_theta).

    """
    r = float(np.squeeze(r))
    shift = float(np.squeeze(shift))
    f_spec_axis = np.fft.rfftfreq(wlen, 1 / fs)  # same bins as stft(nperseg=wlen)
    band_idxs = np.where((f_spec_axis >= bw[0]) & (f_spec_axis <= bw[1]))[0]
    if band_idxs.size == 0:
        raise ValueError(
            f"No frequency bin inside the DOA range {list(bw)} Hz: with fs={fs} Hz and "
            f"wlen={wlen} the bins are {fs / wlen:g} Hz apart (0 to {fs / 2:g} Hz). "
            "Widen the range so it contains at least one bin."
        )
    bands = f_spec_axis[band_idxs]
    theta_rad = np.deg2rad(theta)

    phi_n = np.linspace(2 * np.pi, 0, nch, endpoint=False) + np.deg2rad(shift)
    mic_x = r * np.cos(phi_n)
    mic_y = r * np.sin(phi_n)

    # Projection of each mic position on each look direction, shape (nch, n_theta)
    proj = np.outer(mic_x, np.cos(theta_rad)) + np.outer(mic_y, np.sin(theta_rad))
    k = 2 * np.pi * bands / c
    a_mat = np.exp(1j * k[None, :, None] * proj[:, None, :])
    return band_idxs, bands, a_mat


_stft_bins_phases = {}


def stft_bins(y, wlen, bins):
    """STFT of the columns of y for selected frequency bins only.

    Gives the same values as
    ``scipy.signal.stft(y, window=np.ones(wlen), nperseg=wlen, noverlap=wlen - 1, axis=0)[2][bins]``
    (rectangular window, hop of 1 sample, zero boundary padding, 1/wlen scaling)
    for even wlen, but only computes the requested bins, as sliding sums.

    Parameters
    ----------
    y : np.ndarray
        Signals, shape (n_samples, n_channels).
    wlen : int
        Window length (even).
    bins : np.ndarray
        Indices of the frequency bins to compute.

    Returns
    -------
    np.ndarray
        Complex spectrum, shape (n_bins, n_channels, n_samples + 1).

    """
    if wlen % 2:
        raise ValueError(f"stft_bins needs an even window length, got wlen={wlen}")
    half = wlen // 2
    padded = np.pad(y, ((half, half), (0, 0)))  # boundary="zeros"
    n_pad = padded.shape[0]
    key = (n_pad, wlen, tuple(bins))
    if key not in _stft_bins_phases:
        # Demodulate each bin to DC, sum over the window, then restore the frame phase
        _stft_bins_phases[key] = (
            np.exp(-2j * np.pi * np.outer(bins, np.arange(n_pad)) / wlen)[:, None, :],
            np.exp(2j * np.pi * np.outer(bins, np.arange(n_pad - wlen + 1)) / wlen)[
                :, None, :
            ]
            / wlen,
        )
    phase_in, phase_out = _stft_bins_phases[key]
    csum = np.cumsum(phase_in * padded.T[None], axis=2)
    win_sum = csum[:, :, wlen - 1 :].copy()
    win_sum[:, :, 1:] -= csum[:, :, :-wlen]
    return win_sum * phase_out


class SosFiltFilt:
    """Zero-phase filter, same as ``scipy.signal.sosfiltfilt(sos, x, axis=0)``.

    The constant set-up (pad length, initial conditions) is computed once instead
    of on every call, which is most of the cost for short signals.
    """

    def __init__(self, sos):
        self.sos = np.asarray(sos, dtype=float)
        n_sections = self.sos.shape[0]
        ntaps = 2 * n_sections + 1 - min(
            (self.sos[:, 2] == 0).sum(), (self.sos[:, 5] == 0).sum()
        )
        self.padlen = 3 * ntaps  # scipy's default padding
        self.zi = signal.sosfilt_zi(self.sos)

    def __call__(self, x):
        p = self.padlen
        if x.shape[0] <= p:
            return signal.sosfiltfilt(self.sos, x, axis=0)
        # Odd extension at both ends (scipy's padtype="odd")
        ext = np.concatenate(
            (2 * x[:1] - x[p:0:-1], x, 2 * x[-1:] - x[-2 : -(p + 2) : -1])
        )
        zi = self.zi.reshape(self.zi.shape + (1,) * (x.ndim - 1))
        y, _ = signal.sosfilt(self.sos, ext, axis=0, zi=zi * ext[:1])
        y, _ = signal.sosfilt(self.sos, y[::-1], axis=0, zi=zi * y[-1:])
        return y[::-1][p:-p]


def higher_order_dmas_UCA(
    y,
    fs,
    nch,
    r,
    shift,
    bw,
    theta,
    c=343,
    wlen=64,
    order=2,
    apply_cf=True,
    show=False,
    steering=None,
):
    """
    Multiband Higher-Order Delay Multiply and Sum (p-DMAS) for a Uniform Circular Array,
    with optional Standard Coherence Factor (CF) weighting.

    Parameters:
    r : float
        Radius of the circular array in meters.
    theta : np.ndarray
        Scanning angles (azimuth) in degrees [0, 360].
    order : float, optional
        The dimensionality order (p). order=2 is standard DMAS. order > 2 is higher-order.
    apply_cf : bool, optional
        Whether to calculate and apply the Standard Coherence Factor (CF). Defaults to True.
    steering : tuple, optional
        Output of uca_steering_vectors() for the same fs, nch, r, shift, bw, theta, c
        and wlen. Pass it to avoid recomputing the steering vectors on every call;
        then only the analysed frequency bins are computed and the returned
        `spectrum` is None.
    """
    if steering is None:
        # 1. Spectral Decomposition
        f_spec_axis, _, spectrum = stft(
            y, fs=fs, window=np.ones((wlen,)), nperseg=wlen, noverlap=wlen - 1, axis=0
        )
        # 2-3. UCA geometry and steering vectors (nch, n_bands, n_theta)
        steering = uca_steering_vectors(fs, nch, r, shift, bw, theta, c=c, wlen=wlen)
        band_idxs, bands, a_mat = steering
        spec = spectrum[band_idxs]
    else:
        # 1-3. Only the analysed bins of the spectrum, precomputed steering vectors
        band_idxs, bands, a_mat = steering
        f_spec_axis = np.fft.rfftfreq(wlen, 1 / fs)
        spectrum = None
        spec = stft_bins(y, wlen, band_idxs)
    theta_rad = np.deg2rad(theta)

    # 4. Spatial Filtering, all bands at once
    num_cross_terms = (nch**2) - nch

    # Covariance per band, same as np.cov(spec, bias=True); shape (n_bands, nch, nch)
    spec = spec - spec.mean(axis=2, keepdims=True)
    cov_est = spec @ spec.conj().transpose(0, 2, 1) / spec.shape[2]

    a_bt = a_mat.transpose(1, 0, 2)  # (n_bands, nch, n_theta)

    def _beam_power(cov):
        # a^H cov a for every band and look direction, shape (n_bands, n_theta)
        return np.sum(a_bt.conj() * (cov @ a_bt), axis=1)

    # ==========================================
    # COHERENCE FACTOR (CF) CALCULATION
    # ==========================================
    if apply_cf:
        # Numerator: Coherent Power (Standard DAS output)
        p_das = np.real(_beam_power(cov_est))
        # Denominator: Total Incoherent Power (N * sum of auto-powers)
        incoherent_power = nch * np.real(np.trace(cov_est, axis1=1, axis2=2))
        # CF Weight (Bounded between 0 and 1)
        cf = np.clip(p_das / (incoherent_power[:, None] + 1e-12), 0.0, 1.0)
    else:
        cf = 1.0  # No CF weighting applied

    # ==========================================
    # HIGHER ORDER DMAS MODIFICATIONS
    # ==========================================
    # 1. Take the p-th root (1/order) of the magnitude, preserve phase
    abs_cov = np.abs(cov_est)
    cov_hdmas = np.power(abs_cov, 1.0 / order) * (
        cov_est / (abs_cov + 1e-12)  # added 1e-12 to avoid div by zero
    )

    # 2. Remove auto-correlations
    diag = np.arange(nch)
    cov_hdmas[:, diag, diag] = 0

    # 3. Compute pseudo-power
    p_pseudo = _beam_power(cov_hdmas) / num_cross_terms

    # 4. Dimensionality restoration: raise absolute value back to the power of `order`
    # 5. Apply Coherence Factor
    p_final = np.abs(p_pseudo) ** order * cf  # (n_bands, n_theta)

    mag_p = p_final.sum(axis=0) / len(bands)

    if show:
        fig, ax = plt.subplots()
        linestyles = ["-", "--", "-.", ":"]
        for b_idx in range(len(bands)):
            ax.plot(
                np.rad2deg(theta_rad),
                10
                * np.log10(
                    p_final[b_idx] + 1e-12
                ),  # Added log10 back for better dB visualization
                label=f"{bands[b_idx]:.1f} Hz",
                linestyle=linestyles[b_idx % len(linestyles)],
            )
        ax.set_title(f"UCA {order}-Order DMAS-CF Pseudospectra")
        ax.plot(
            np.rad2deg(theta_rad),
            10
            * np.log10(mag_p + 1e-12),  # Added log10 back for better dB visualization
            label="Full spectrum",
            linewidth=4,
            color="black",
        )
        plt.grid()
        ax.set_xlabel("Angle (Degrees)")
        ax.set_ylabel("Magnitude (dB)")
        plt.legend()
        plt.show()

    return theta, mag_p, f_spec_axis, spectrum, bands


# Thymio robot control functions


def perc_to_speed(value):
    """Map a value in range [0,100] = percentage speed to [0,500] = [min, max] values that Thymio takes as speed control. Clamp input to [0,100]. Returns int."""
    try:
        v = float(value)
    except Exception:
        v = 0.0
    v = max(0.0, min(100.0, v))
    return int(round((v / 100.0) * 500.0))


def degs_to_speed(value, max_deg_s=170):
    """Angular speed in deg/s -> per-wheel speed offset in [-500, 500]. Keeps the sign
    (positive = counter-clockwise / left). Clamps input. Returns int."""
    try:
        v = float(value)
    except Exception:
        v = 0.0
    v = max(-max_deg_s, min(max_deg_s, v))
    return int(round((v / max_deg_s) * 500))


def speed_to_degs(value, max_deg_s=170):
    """Inverse of degs_to_speed: motor units [-500, 500] -> deg/s."""
    try:
        v = float(value)
    except Exception:
        v = 0.0
    v = max(-500, min(500, v))
    return (v / 500) * max_deg_s


class WhitenedUCABeamformer:
    """
    Precomputes noise whitening + steering vectors ONCE at startup,
    using a long calibration recording. Per-chirp processing is then
    fast (only new FFT + whitening + beamform + CFAR).
    """

    def __init__(self, fs, nch, r, shift, bw, theta, wlen=16, nfft=128, c=343):
        self.fs = fs
        self.nch = nch
        self.r = r
        self.shift = shift
        self.bw = bw
        self.theta = theta
        self.theta_rad = np.deg2rad(theta)
        self.wlen = wlen
        self.nfft = nfft
        self.c = c

        # Placeholders filled in by calibrate()
        self.R_n = None
        self.W = None
        self.band_idxs = None
        self.bands = None
        self.a_mat_white = None
        self.a_H_mat = None
        self.norm_theta = None
        self.is_calibrated = False

    def calibrate(self, y_noise_long, eps=0.02):
        """
        Run ONCE at startup with a long (10-15s) noise-only recording.
        Precomputes R_n, W, and whitened steering vectors for all
        subsequent chirp processing.

        Parameters
        ----------
        y_noise_long : ndarray, shape (n_samples, nch)
            Long calibration recording (gear noise only, no chirp).
            Recommend sweeping wheel speed during this recording.
        """
        wlen = self.wlen
        nfft = self.nfft
        fs = self.fs
        nch = self.nch

        # --- Estimate R_n(f) from long recording ---
        f_spec_axis, _, spectrum = stft(
            y_noise_long,
            fs=fs,
            window=np.ones((wlen,)),
            nperseg=wlen,
            noverlap=wlen - 1,
            nfft=nfft,
            axis=0,
        )
        # spectrum shape: (n_bins, nch, n_frames)  -- n_frames will be
        # large (~ n_samples) since noverlap = wlen-1, giving a very
        # stable covariance estimate -- this is fine, it's a one-time cost

        n_bins = spectrum.shape[0]
        R_n_full = np.zeros((n_bins, nch, nch), dtype=complex)

        for f_idx in range(n_bins):
            seg = spectrum[f_idx, :, :]  # (nch, n_frames)
            R_n_full[f_idx] = np.cov(seg, bias=True)
            trace_val = np.trace(R_n_full[f_idx]).real
            R_n_full[f_idx] += (eps * trace_val / nch) * np.eye(nch)

        self.R_n = R_n_full

        # --- Restrict to usable band ---
        self.band_idxs = np.where(
            (f_spec_axis >= self.bw[0]) & (f_spec_axis <= self.bw[1])
        )[0]
        self.bands = f_spec_axis[self.band_idxs]

        # --- Compute whitening matrices W(f) for used bands only ---
        W_full = np.zeros((len(self.band_idxs), nch, nch), dtype=complex)
        for i, f_idx in enumerate(self.band_idxs):
            L = np.linalg.cholesky(self.R_n[f_idx])
            W_full[i] = np.linalg.inv(L)
        self.W = W_full  # shape (n_bands_used, nch, nch)

        # --- UCA geometry ---
        shift_rad = np.deg2rad(self.shift)
        phi_n = np.linspace(2 * np.pi, 0, nch, endpoint=False) + shift_rad
        mic_x = self.r * np.cos(phi_n)
        mic_y = self.r * np.sin(phi_n)

        # --- Build + whiten steering vectors ONCE ---
        n_bands = len(self.bands)
        n_theta = len(self.theta)
        a_mat = np.zeros((nch, n_bands, n_theta), dtype=complex)

        for b_idx, f in enumerate(self.bands):
            k = 2 * np.pi * f / self.c
            for t_idx, t_start in enumerate(self.theta_rad):
                phase = k * (mic_x * np.cos(t_start) + mic_y * np.sin(t_start))
                a_mat[:, b_idx, t_idx] = np.exp(1j * phase)

        a_mat_white = np.zeros_like(a_mat)
        for b_idx in range(n_bands):
            a_mat_white[:, b_idx, :] = self.W[b_idx] @ a_mat[:, b_idx, :]

        self.a_mat_white = a_mat_white
        self.a_H_mat = np.conj(
            a_mat_white.transpose(1, 2, 0)
        )  # (n_bands, n_theta, nch)

        # --- Precompute per-angle normalization (fixed, since calibration is fixed) ---
        norm_theta = np.zeros((n_bands, n_theta))
        for b_idx in range(n_bands):
            a = a_mat_white[:, b_idx, :]
            norm_theta[b_idx] = np.maximum(
                np.real(np.sum(np.conj(a) * a, axis=0)), 1e-12
            )
        self.norm_theta = norm_theta

        self.is_calibrated = True
        print(f"Calibration complete: {n_bands} bands, {n_theta} angles.")

    def das_filter_UCA_motor_whitened(self, y, show=False):
        """
        Multiband whitened DAS for a UCA, using precomputed calibration
        data (self.W, self.a_mat_white, self.a_H_mat, self.norm_theta,
        self.band_idxs, self.bands, self.theta). Only new work per call
        is the STFT + whitening + beamform of the incoming buffer `y`.
        """
        if not self.is_calibrated:
            raise RuntimeError("Call calibrate() once before process().")

        # 1. Spectral Decomposition (uses calibration-time fs/wlen for consistency)
        f_spec_axis, _, spectrum = stft(
            y,
            fs=self.fs,
            window=np.ones((self.wlen,)),
            nperseg=self.wlen,
            noverlap=self.wlen - 1,
            nfft=self.nfft,
            axis=0,
        )

        theta_rad = self.theta_rad  # use calibration-time angle grid, always

        # 4. Spatial Filtering (reuses precomputed geometry/whitening — no recompute)
        p = np.zeros(len(self.theta), dtype=complex)

        if show:
            fig, ax = plt.subplots()
            linestyles = ["-", "--", "-.", ":"]

        for b_idx, f_idx in enumerate(self.band_idxs):
            spec = spectrum[f_idx, :, :]  # (nch, n_frames)

            spec_white = self.W[b_idx] @ spec  # whiten with calibration W

            cov_est = np.cov(spec_white, bias=True)

            a = self.a_mat_white[:, b_idx, :]
            a_H = self.a_H_mat[b_idx, :, :]
            norm = self.norm_theta[b_idx]

            p_i = np.einsum("ij,jk,ki->i", a_H, cov_est, a) / norm
            p += p_i

            if show:
                ax.plot(
                    np.rad2deg(theta_rad),
                    np.abs(p_i) + 1e-12,
                    label=f"{self.bands[b_idx]:.1f} Hz",
                    linestyle=linestyles[b_idx % len(linestyles)],
                )

        mag_p = np.abs(p) / len(self.bands)

        if show:
            ax.set_title("Whitened UCA DAS Pseudospectra")
            ax.plot(
                np.rad2deg(theta_rad),
                mag_p + 1e-12,
                label="full spectrum",
                linewidth=4,
                color="black",
            )
            ax.grid()
            ax.set_xlabel("Angle (Degrees)")
            ax.set_ylabel("Magnitude (linear)")
            ax.legend()
            plt.show()

            plt.figure()
            debug_bin = self.band_idxs[8]
            plt.title(f"Spectrum at f={self.bands[8]:.1f} Hz")
            plt.imshow(np.abs(spectrum[debug_bin, :, :]), aspect="auto", origin="lower")
            plt.colorbar(label="Magnitude")
            plt.xlabel("Frames")
            plt.ylabel("Channels")
            plt.show()

        return self.theta, mag_p, f_spec_axis, spectrum, self.bands

    def plot_calibration_diagnostics(self, save_dir=".", prefix="calib"):
        """
        Generates diagnostic plots after calibrate() has been run:
        1. Noise power per mic vs frequency (R_n diagonal)
        2. Inter-mic coherence vs frequency (R_n off-diagonal, normalized)
        3. Eigenvalues / condition number of R_n(f) vs frequency
        4. Whitening matrix W(f) magnitude (heatmap)
        5. Steering vector norm vs angle, before vs after whitening
            (shows directional bias introduced/corrected by whitening)
        """
        if not self.is_calibrated:
            raise RuntimeError("Run calibrate() before plotting diagnostics.")

        os.makedirs(save_dir, exist_ok=True)
        nch = self.nch
        bands = self.bands
        n_bands = len(bands)

        # ---------------------------------------------------------------
        # 1. Noise power per mic vs frequency (diagonal of R_n)
        # ---------------------------------------------------------------
        fig, ax = plt.subplots(figsize=(8, 4))
        for ch in range(nch):
            power_db = 10 * np.log10(np.abs(self.R_n[self.band_idxs, ch, ch]) + 1e-12)
            ax.plot(bands, power_db, label=f"Mic {ch}")
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("Noise power (dB)")
        ax.set_title("Per-mic gear noise power spectrum")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(
            os.path.join(save_dir, f"{prefix}_noise_power_per_mic.png"), dpi=150
        )
        plt.close(fig)

        # ---------------------------------------------------------------
        # 2. Inter-mic coherence vs frequency (normalized off-diagonal)
        #    coherence_ij(f) = |R_n[i,j]| / sqrt(R_n[i,i] * R_n[j,j])
        #    Value near 1 => highly correlated noise between that pair
        #    Value near 0 => spatially uncorrelated at that frequency
        # ---------------------------------------------------------------
        fig, ax = plt.subplots(figsize=(8, 4))
        pair_list = [(i, j) for i in range(nch) for j in range(i + 1, nch)]
        for i, j in pair_list:
            Rii = np.abs(self.R_n[self.band_idxs, i, i])
            Rjj = np.abs(self.R_n[self.band_idxs, j, j])
            Rij = np.abs(self.R_n[self.band_idxs, i, j])
            coherence = Rij / (np.sqrt(Rii * Rjj) + 1e-12)
            ax.plot(bands, coherence, label=f"Mic {i}-{j}")
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("Coherence (0-1)")
        ax.set_title("Inter-mic noise coherence (spatial correlation of gear noise)")
        ax.set_ylim(0, 1.05)
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(save_dir, f"{prefix}_coherence.png"), dpi=150)
        plt.close(fig)

        # ---------------------------------------------------------------
        # 3. Eigenvalues / condition number vs frequency
        #    High condition number => near-singular R_n => whitening will
        #    aggressively boost that direction (check for instability)
        # ---------------------------------------------------------------
        eigvals_all = np.zeros((n_bands, nch))
        cond_numbers = np.zeros(n_bands)
        for b_idx, f_idx in enumerate(self.band_idxs):
            ev = np.linalg.eigvalsh(self.R_n[f_idx])
            ev = np.sort(np.abs(ev))[::-1]
            eigvals_all[b_idx] = ev
            cond_numbers[b_idx] = ev[0] / (ev[-1] + 1e-12)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
        for k in range(nch):
            ax1.plot(bands, 10 * np.log10(eigvals_all[:, k] + 1e-12), label=f"λ{k+1}")
        ax1.set_ylabel("Eigenvalue (dB)")
        ax1.set_title("R_n(f) eigenvalue spread")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        ax2.plot(bands, cond_numbers)
        ax2.set_xlabel("Frequency (Hz)")
        ax2.set_ylabel("Condition number")
        ax2.set_title(
            "R_n(f) condition number (high = near-singular, watch for instability)"
        )
        ax2.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(
            os.path.join(save_dir, f"{prefix}_eigenvalues_condition.png"), dpi=150
        )
        plt.close(fig)

        # ---------------------------------------------------------------
        # 4. Whitening matrix W(f) magnitude, shown as heatmap
        #    (frequency on y-axis, flattened 4x4 matrix entries on x-axis)
        # ---------------------------------------------------------------
        W_mag = np.abs(self.W).reshape(n_bands, nch * nch)  # (n_bands, 16)
        fig, ax = plt.subplots(figsize=(6, 6))
        im = ax.imshow(
            W_mag,
            aspect="auto",
            origin="lower",
            extent=[0, nch * nch, bands[0], bands[-1]],
            cmap="viridis",
        )
        ax.set_xlabel("W matrix entry (flattened row-major: [0,0],[0,1],...,[3,3])")
        ax.set_ylabel("Frequency (Hz)")
        ax.set_title("Whitening matrix |W(f)| magnitude")
        fig.colorbar(im, ax=ax, label="|W| magnitude")
        fig.tight_layout()
        fig.savefig(
            os.path.join(save_dir, f"{prefix}_whitening_matrix_heatmap.png"), dpi=150
        )
        plt.close(fig)

        # ---------------------------------------------------------------
        # 5. Steering vector norm vs angle, before vs after whitening
        #    Shows if whitening introduces angle-dependent bias, and
        #    confirms the norm_theta correction is doing its job
        # ---------------------------------------------------------------
        # Recompute unwhitened steering vectors for comparison
        shift_rad = np.deg2rad(self.shift)
        phi_n = np.linspace(2 * np.pi, 0, nch, endpoint=False) + shift_rad
        mic_x = self.r * np.cos(phi_n)
        mic_y = self.r * np.sin(phi_n)

        norm_unwhitened = np.zeros(len(self.theta))
        for t_idx, t_start in enumerate(self.theta_rad):
            total = 0
            for b_idx, f in enumerate(bands):
                k = 2 * np.pi * f / self.c
                phase = k * (mic_x * np.cos(t_start) + mic_y * np.sin(t_start))
                a = np.exp(1j * phase)
                total += np.real(np.sum(np.conj(a) * a))
            norm_unwhitened[t_idx] = total
        norm_unwhitened /= len(bands)

        norm_whitened_avg = np.mean(self.norm_theta, axis=0)  # avg over bands

        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(
            self.theta,
            norm_unwhitened / norm_unwhitened.max(),
            label="Unwhitened (should be flat/uniform)",
        )
        ax.plot(
            self.theta,
            norm_whitened_avg / norm_whitened_avg.max(),
            label="Whitened (shows directional bias from noise)",
        )
        ax.set_xlabel("Angle (deg)")
        ax.set_ylabel("Normalized steering vector norm")
        ax.set_title("Whitening-induced directional sensitivity change")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(
            os.path.join(save_dir, f"{prefix}_steering_norm_comparison.png"), dpi=150
        )
        plt.close(fig)

        print(f"Calibration diagnostic plots saved to: {save_dir}")
