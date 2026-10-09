#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created: 2025-12-16
Author: Alberto Doimo
email: alberto.doimo@uni-konstanz.de

"""

import utilities
import numpy as np
import scipy.signal as signal
import sounddevice as sd
import soundfile as sf
import queue
import threading
import time


class AudioProcessor:
    """AudioProcessor class for real-time audio signal processing and analysis."""

    def __init__(
        self,
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
        ref,
        theta_das,
        N_peaks,
        soundcard_index,
        subtype,
        interp_sensitivity,
        tgtmic_relevant_freqs,
        filename,
        rec_samplerate,
        DOA_freq_range,
        sos,
        beamformer=None,
        input_block_ms=1,
        dmas_wlen=64,
        self_call_onset=0.0095,
        self_call_end=0.0263,
        call_guard_time=0.03,
    ):
        self.fs = fs
        self.channels = channels
        self.radius = radius
        self.shift = shift
        self.block_size = block_size
        self.analyzed_buffer_time = analyzed_buffer_time
        self.data = data
        self.args = args
        self.trigger_level = trigger_level
        self.critical_level = critical_level
        self.ref = ref
        self.theta_das = theta_das
        self.N_peaks = N_peaks
        self.soundcard_index = soundcard_index
        self.subtype = subtype
        self.interp_sensitivity = interp_sensitivity
        self.tgtmic_relevant_freqs = tgtmic_relevant_freqs
        self.filename = filename
        self.rec_samplerate = rec_samplerate
        self.DOA_freq_range = DOA_freq_range
        self.sos = sos
        self.filtfilt = utilities.SosFiltFilt(sos)  # sosfiltfilt with set-up done once
        self.beamformer = beamformer
        self.shared_audio_queue = queue.Queue()
        self.ts_queue = queue.Queue()
        self.recording = False  # only queue audio for the file writer when recording
        self.running = True

        # Input blocks: PortAudio callbacks run in Python and take the GIL, so very
        # small blocks (1 ms = 1000 callbacks/s) slow down the DOA processing itself.
        self.input_blocksize = max(1, int(round(self.fs * input_block_ms / 1000)))

        # Analysis window: the gap between two own calls as heard by the mics, i.e. from
        # the end of the previous own call to the start of the current one (each with a
        # guard time), at most block_size samples. Before the first call the newest
        # block_size samples are used.
        self.call_guard_time = call_guard_time  # s, kept from own calls (echoes, timing errors)
        # Measured on the robot, relative to the emission time from emit_call:
        self.self_call_onset = self_call_onset  # s until own call is heard (device latency)
        self.self_call_end = self_call_end  # s until own call (with echoes) has died out
        self.min_window_rows = int(self.fs * self.analyzed_buffer_time)
        self.filter_margin = 128  # samples filtered around the trimmed part (see update_das_UCA)
        self.window_rows = 0  # length (samples) of the last analysed window

        # Preallocated ring buffer holding the most recent input samples
        self.ring_size = int(self.block_size * 2)
        self.ring = np.zeros((self.ring_size, self.channels), dtype=np.float32)
        self.ring_pos = 0  # next write index
        self.samples_written = 0
        self.last_sample_time = None  # wall-clock (ADC-corrected) time of newest sample
        self.window_end_time = None  # wall-clock time of the end of the last analysed window
        self.ring_lock = threading.Lock()
        self.step_times = {}  # ms per step of the last update_das_UCA call
        self.input_status_count = 0

        # Persistent output stream state (see start_output_stream / emit_call)
        self.out_stream = None
        self.out_latency = 0.0  # s, reported output latency (fallback for DAC timing)
        self.in_latency = None  # s, reported input latency (fallback for ADC timing)
        self.current_frame = 0
        self.call_requested = False
        self.call_started = threading.Event()
        self.call_done = threading.Event()
        self.call_done.set()  # no call playing
        self.last_emission_time = None
        self.prev_emission_time = None

        # The DMAS steering vectors depend only on fixed parameters: compute them once
        self.dmas_wlen = dmas_wlen
        self.steering = utilities.uca_steering_vectors(
            self.fs,
            self.channels - 1,
            self.radius,
            self.shift,
            self.DOA_freq_range,
            self.theta_das,
            wlen=self.dmas_wlen,
        )

    def stop(self):
        """Stop audio recording/streaming gracefully."""
        self.running = False
        try:
            self.shared_audio_queue.put_nowait(None)
        except Exception:
            pass
        if self.out_stream is not None:
            try:
                self.out_stream.stop()
                self.out_stream.close()
            except Exception:
                pass
            self.out_stream = None
        self.call_done.set()  # release a caller waiting in emit_call

    def _open_input_stream(self):
        """Create the input stream feeding callback_in (not started)."""
        stream = sd.InputStream(
            samplerate=self.fs,
            device=self.soundcard_index,
            channels=self.channels,
            dtype="float32",
            callback=self.callback_in,
            latency="low",
            blocksize=self.input_blocksize,
        )
        self.in_latency = stream.latency
        print("Input stream latency:", stream.latency, "s")
        return stream

    def continuos_recording(self):
        """Continuously records audio from the specified sound device and writes it to a file.

        Attributes Used
        ---------------
        self.filename : str
            Path to the output audio file where the recording will be saved.
        self.rec_samplerate : int
            Sample rate (Hz) for writing to the output file.
        self.channels : int
            Number of audio channels to record.
        self.shared_audio_queue : queue.Queue
            Thread-safe queue containing audio blocks from the input stream.

        Notes
        -----
        - All blocks waiting in the queue are written in one call to limit disk I/O.
        - The recording start time (wall clock of the first sample) is put in
          self.ts_queue by callback_in.

        """
        self.recording = True
        with sf.SoundFile(
            self.filename,
            mode="x",
            samplerate=self.rec_samplerate,
            channels=self.channels,
            subtype=self.subtype,
        ) as file:
            with self._open_input_stream():
                print("Recording started\n")
                while self.running:
                    try:
                        data = self.shared_audio_queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if data is None:
                        break
                    blocks = [data]
                    while True:
                        try:
                            more = self.shared_audio_queue.get_nowait()
                        except queue.Empty:
                            break
                        if more is None:
                            self.running = False
                            break
                        blocks.append(more)
                    file.write(np.concatenate(blocks) if len(blocks) > 1 else data)

    def input_stream(self):
        """Continuously captures audio from the specified sound device (no recording).

        Notes
        -----
        The stream stays open until stop() is called. The recording start time
        (wall clock of the first sample) is put in self.ts_queue by callback_in.

        """
        self.recording = False
        with self._open_input_stream():
            print("Input stream started\n")
            while self.running:
                time.sleep(0.1)

    def start_output_stream(self):
        """Open the output stream once and keep it running (silence between calls).

        Opening and closing a PortAudio stream for every call adds a variable delay
        (tens of ms on ALSA). With a persistent stream a call starts within one
        output buffer of emit_call().
        """
        self.out_stream = sd.OutputStream(
            samplerate=self.fs,
            blocksize=0,
            device=self.soundcard_index,
            channels=self.data.shape[1],
            dtype="float32",
            callback=self.callback_out,
            latency="low",
        )
        self.out_latency = self.out_stream.latency
        self.out_stream.start()
        print("Output stream started, latency:", self.out_stream.latency, "s")

    def emit_call(self, timeout=1.0):
        """Start playing the call once on the running output stream.

        Returns as soon as the first call block has been handed to the device, so that
        listening can go on while the call is played.

        Parameters
        ----------
        timeout : float, optional
            Maximum time (s) to wait for the previous call to finish and this one to start.

        Returns
        -------
        float or None
            Wall-clock time at which the first call sample reaches the DAC
            (corrected for the output latency), or None if it was not played.

        """
        if self.out_stream is None:
            self.start_output_stream()
        if not self.call_done.wait(timeout):  # previous call still playing
            print("Warning: previous call still playing after", timeout, "s")
        self.prev_emission_time = self.last_emission_time
        self.call_started.clear()
        self.call_done.clear()
        self.last_emission_time = None
        self.call_requested = True
        if not self.call_started.wait(timeout):
            print("Warning: call not played within", timeout, "s")
            self.call_requested = False
            self.call_done.set()
        return self.last_emission_time

    def listen_end_time(self):
        """Wall-clock time until which the current window can listen: just before the
        own call emitted last becomes audible (onset minus guard time)."""
        if self.last_emission_time is None:
            return None
        return self.last_emission_time + self.self_call_onset - self.call_guard_time

    def wait_for_samples(self, until, timeout=1.0):
        """Block until the input samples up to wall-clock time `until` have been received."""
        if until is None:
            return
        deadline = time.monotonic() + timeout
        while self.running and time.monotonic() < deadline:
            last = self.last_sample_time
            if last is not None and last >= until:
                return
            remaining = until - (last if last is not None else time.time())
            time.sleep(min(max(remaining, 0.0005), 0.01))

    @staticmethod
    def _stream_delay(later, earlier, fallback):
        """Delay between two PortAudio stream times, with a fallback when they are not provided."""
        delay = later - earlier
        if later <= 0 or earlier <= 0 or not (0 <= delay < 1):
            return fallback
        return delay

    def callback_out(self, outdata, frames, time_info, status):
        """Audio output callback: plays self.data once per emit_call(), silence otherwise.

        Parameters
        ----------
        outdata : numpy.ndarray
            Output buffer where audio samples should be written. Shape is (frames, channels).
        frames : int
            Number of audio frames requested by the audio stream.
        time_info : CData
            PortAudio time information (current time and DAC time of this buffer).
        status : sd.CallbackFlags
            Status flags indicating any issues with the audio stream (e.g., underflow).

        Attributes Used
        ---------------
        self.data : numpy.ndarray
            Audio data array to be played back.
        self.current_frame : int
            Current position in the audio data array for playback.

        """
        if self.current_frame == 0:
            if not self.call_requested:
                outdata.fill(0)
                return
            # First block of a call: time when it reaches the DAC (output latency)
            dac_delay = self._stream_delay(
                time_info.outputBufferDacTime, time_info.currentTime, self.out_latency
            )
            self.last_emission_time = time.time() + dac_delay
            self.call_requested = False
            self.call_started.set()

        chunksize = min(len(self.data) - self.current_frame, frames)
        outdata[:chunksize] = self.data[
            self.current_frame : self.current_frame + chunksize
        ]
        outdata[chunksize:] = 0
        self.current_frame += chunksize
        if self.current_frame >= len(self.data):
            self.current_frame = 0
            self.call_done.set()

    def callback_in(self, indata, frames, time_info, status):
        """
        Process incoming audio data from the audio stream.

        Parameters
        ----------
        indata : ndarray
            Audio data buffer containing the input samples for this block.
        frames : int
            Number of frames (samples) in the current audio block.
        time_info : CData
            PortAudio time information (current time and ADC time of this buffer).
        status : CallbackFlags
            Status flags indicating any errors or special conditions
            (e.g., input overflow, buffer underflow).

        Attributes Used
        ---------------
        self.ring : numpy.ndarray
            Ring buffer holding the most recent input samples.
        self.shared_audio_queue : queue.Queue
            Thread-safe queue of audio blocks for the file writer (only when recording).

        Notes
        -----
        Kept minimal (no allocation besides the recording copy, no printing),
        since it runs every input block while holding the GIL.

        """
        if status:
            self.input_status_count += 1

        # Wall-clock time of the first sample of this block (input latency corrected)
        now = time.time()
        adc_delay = self._stream_delay(
            time_info.currentTime,
            time_info.inputBufferAdcTime,
            self.in_latency if self.in_latency else frames / self.fs,
        )
        first_sample_time = now - adc_delay
        if self.samples_written == 0:
            self.ts_queue.put(first_sample_time)  # recording start time

        with self.ring_lock:
            pos = self.ring_pos
            end = pos + frames
            if end <= self.ring_size:
                self.ring[pos:end] = indata
            else:
                k = self.ring_size - pos
                self.ring[pos:] = indata[:k]
                self.ring[: frames - k] = indata[k:]
            self.ring_pos = end % self.ring_size
            self.samples_written += frames
            self.last_sample_time = first_sample_time + (frames - 1) / self.fs

        if self.recording:
            self.shared_audio_queue.put(indata.copy())

    def get_latest_chunk(self, start_time=None, end_time=None):
        """Return a copy of input samples between two times, oldest sample first.

        Parameters
        ----------
        start_time : float, optional
            Wall-clock time of the oldest sample to include (e.g. end of the robot's
            previous call). If None, the window is self.block_size samples long.
        end_time : float, optional
            Wall-clock time of the newest sample to include (e.g. just before the
            robot's current call is heard). If None, the newest sample.

        Returns
        -------
        numpy.ndarray or None
            Array of shape (n, channels) with n <= self.block_size, or None if fewer
            than self.min_window_rows samples are available.

        """
        with self.ring_lock:
            available = min(self.samples_written, self.ring_size)
            newest_time = self.last_sample_time
            skip = 0  # newest samples left out (after end_time)
            if end_time is not None and newest_time is not None:
                skip = min(max(int(round((newest_time - end_time) * self.fs)), 0), available)
                newest_time -= skip / self.fs
            num_rows = min(self.block_size, available - skip)
            if start_time is not None and newest_time is not None:
                num_rows = min(num_rows, int((newest_time - start_time) * self.fs) + 1)
            num_rows -= num_rows % 32  # FFT-friendly length for the Hilbert transform
            self.window_rows = max(num_rows, 0)
            if num_rows < self.min_window_rows:
                print(
                    f"Warning: only {max(num_rows, 0) / self.fs:.3f} s of audio available "
                    f"for analysis (need {self.min_window_rows / self.fs:.3f} s)"
                )
                return None
            stop = (self.ring_pos - skip) % self.ring_size
            start = (stop - num_rows) % self.ring_size
            if start < stop:
                chunk = self.ring[start:stop].copy()
            else:
                chunk = np.concatenate((self.ring[start:], self.ring[:stop]))
            self.window_end_time = newest_time
        return chunk

    def update_das_UCA(self):
        """Calculates DOA using Delay And Sum (DAS) method.

        The processing pipeline includes:
        1. High-pass filtering of the input buffer
        2. Max detection using Hilbert transform on the reference channel
        3. Full signal trimming around the maximum envelope peak
        4. Calculates frequency-wise RMS values for the reference channel
        5. Normalizes by mic sensitivity (from loaded calibration data)
        6. Computes total RMS across relevant frequency bands
        7. Converts to dB SPL level
        8. DAS beamforming (only in the filtered bandwidth) for spatial response and direction estimation
        9. Peak detection and angle extraction

        Attributes Used
        ---------------
        self.ring : numpy.ndarray
            Ring buffer with the latest input samples (newest block_size samples analysed).
        self.fs : int
            Sampling frequency of the audio data.
        self.ref : int
            Index of the reference microphone channel.
        self.sos : numpy.ndarray
            Second-order sections for the highpass filter.
        self.analyzed_buffer_time : float
            Duration (in seconds) of the buffer to analyze.
        self.interp_sensitivity : numpy.ndarray
            Interpolated sensitivity values for frequency normalization.
        self.tgtmic_relevant_freqs : list or numpy.ndarray
            Indices of relevant frequency bands for the target microphone.
        self.trigger_level : float
            Threshold level for triggering angle calculation.
        self.critical_level : float
            Critical threshold level for angle calculation.
        self.channels : int
            Number of audio channels.
        self.mic_spacing : float
            Spacing between microphones for delay calculation.
        self.highpass_freq : float
            High-pass filter cutoff frequency.
        self.lowpass_freq : float
            Low-pass filter cutoff frequency.
        self.theta_das : numpy.ndarray
            Array of angles (in degrees) for DAS beamforming.
        self.N_peaks : int
            Number of peaks to detect in the spatial response.

        Returns
        -------
        tuple
            If the detected dB SPL level exceeds the trigger level or critical level:
                (float, float): A tuple containing:
                    - peak_angle (float): The angle (in degrees) of the strongest
                      detected acoustic source
                    - dB_SPL_level (float): The sound pressure level in dB SPL of the
                      detected event
            Otherwise:
                (None, float): A tuple containing:
                    - None: No direction detected
                    - dB_SPL_level (float): The sound pressure level in dB SPL
        Notes
        -----
        - The trimming window of self.analyzed_buffer_time seconds is centered on
          the maximum envelope amplitude
        - The DAS beamforming is performed only within the specified highpass and lowpass bandwidth
        - Only the top N peaks (defined by N_peaks) sorted by their height are considered for direction
          estimation

        """
        t_prev = time.perf_counter()
        step_times = self.step_times = {}  # ms per processing step, for profiling

        def lap(step):
            nonlocal t_prev
            t_now = time.perf_counter()
            step_times[step] = (t_now - t_prev) * 1e3
            t_prev = t_now

        # Listen in the gap between own calls as heard by the mics: from the end of the
        # previous call to just before the current one (call this after emit_call)
        window_start = None
        if self.prev_emission_time is not None:
            window_start = (
                self.prev_emission_time + self.self_call_end + self.call_guard_time
            )
        in_buffer = self.get_latest_chunk(window_start, self.listen_end_time())
        lap("read")
        if in_buffer is None:
            no_angle = [None] if self.N_peaks == 1 else [None, None]
            return no_angle, np.array([-np.inf])

        mics = in_buffer[:, 1:5]
        # High-pass only the reference mic over the whole window, to find the loudest part
        ref_sig = self.filtfilt(mics[:, self.ref])
        lap("filter")

        # Filter the input with its envelope on ref channel
        filtered_envelope = np.abs(signal.hilbert(ref_sig))

        max_envelope_idx = np.argmax(filtered_envelope)

        # Trim all channels around the max
        trim_ms = self.analyzed_buffer_time  # ms
        trim_samples = int(self.fs * trim_ms)
        half_trim = trim_samples // 2

        # Ensure trimmed_signal always has exactly trim_samples rows (matching trim_ms duration)
        if max_envelope_idx - half_trim < 0:
            start_idx = 0
            end_idx = trim_samples
        elif max_envelope_idx + half_trim > mics.shape[0]:
            end_idx = mics.shape[0]
            start_idx = end_idx - trim_samples
        else:
            start_idx = max_envelope_idx - half_trim
            end_idx = start_idx + trim_samples

        # High-pass all mics only around the trimmed part. The filter's impulse response
        # dies out within a few samples, so with this margin the result is the same as
        # filtering the whole window.
        seg_start = max(start_idx - self.filter_margin, 0)
        seg_end = min(end_idx + self.filter_margin, mics.shape[0])
        seg = self.filtfilt(mics[seg_start:seg_end])
        trimmed_signal = seg[start_idx - seg_start : end_idx - seg_start]
        lap("envelope_trim")
        centrefreqs, freqrms = utilities.calc_native_freqwise_rms(
            trimmed_signal[:, self.ref], self.fs
        )
        freqwise_Parms = freqrms / self.interp_sensitivity
        # print(
        #     f"freqwise_Parms[self.tgtmic_relevant_freqs]: {freqwise_Parms[self.tgtmic_relevant_freqs]}"
        # )
        # print(f"freqrms : {freqrms}")

        total_rms_freqwise_Parms = np.sqrt(
            np.sum(freqwise_Parms[self.tgtmic_relevant_freqs] ** 2)
        )
        dB_SPL_level = utilities.pascal_to_dbspl(
            total_rms_freqwise_Parms
        )  # dB SPL level for reference channel
        lap("spl")
        # theta, spatial_resp, f_spec_axis, spectrum, bands = higher_order_dmas_UCA(
        #     trimmed_signal,
        #     self.fs,
        #     self.channels - 1,
        #     self.radius,
        #     self.shift,
        #     self.DOA_freq_range,
        #     theta=self.theta_das,
        #     show=False,
        # )

        theta, spatial_resp, f_spec_axis, spectrum, bands = (
            utilities.higher_order_dmas_UCA(
                trimmed_signal,
                self.fs,
                self.channels - 1,
                self.radius,
                self.shift,
                self.DOA_freq_range,
                theta=self.theta_das,
                wlen=self.dmas_wlen,
                show=False,
                steering=self.steering,
            )
        )
        lap("dmas")

        # theta, spatial_resp, f_spec_axis, spectrum, bands = (
        #     self.beamformer.das_filter_UCA_motor_whitened(trimmed_signal, show=False)
        # )

        # print("das time:", time.time() - start_time)
        # peaks, _ = signal.find_peaks(spatial_resp)  # Adjust height as needed
        peaks = utilities.find_peaks_circular(spatial_resp, wrap=3)

        peak_angles = theta[peaks]
        N = self.N_peaks  # Number of peaks to keep

        # Sort peaks by their height and keep the N largest ones
        peak_heights = spatial_resp[peaks]
        top_n_peak_indices = np.argsort(peak_heights)[
            -N:
        ]  # Indices of the N largest peaks # Indices of the N largest peaks
        top_n_peak_indices = top_n_peak_indices[::-1]
        peak_angles = theta[peaks[top_n_peak_indices]]  # Corresponding angles
        lap("peaks")

        if self.N_peaks == 1:
            if (
                dB_SPL_level > self.trigger_level or dB_SPL_level > self.critical_level
            ) and peak_angles.shape[0] != 0:
                return [peak_angles[0]], dB_SPL_level
            else:
                return [None], dB_SPL_level
        else:
            if (
                dB_SPL_level > self.trigger_level or dB_SPL_level > self.critical_level
            ) and peak_angles.shape[0] != 0:
                return peak_angles, dB_SPL_level
            else:
                peak_angles = [None, None]
                return peak_angles, dB_SPL_level
