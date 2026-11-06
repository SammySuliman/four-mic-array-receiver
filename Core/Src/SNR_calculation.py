import argparse
import wave
import numpy as np


def load_wav(filename):
    """
    Load a 16-bit PCM WAV file.

    The script will stop immediately if any channel:
    - contains only zeros
    - is stuck at one constant value
    - has zero AC power
    """

    with wave.open(filename, "rb") as wf:
        channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        sample_rate = wf.getframerate()
        num_frames = wf.getnframes()

        raw_audio = wf.readframes(num_frames)

    assert sample_width == 2, (
        f"Expected 16-bit PCM WAV, "
        f"but sample width is {sample_width} bytes."
    )

    audio = np.frombuffer(
        raw_audio,
        dtype="<i2"
    ).astype(np.float64)

    if channels > 1:
        audio = audio.reshape(-1, channels)

    print("=" * 50)
    print("CHANNEL DIAGNOSTICS")
    print("=" * 50)

    first_5_end = min(
        int(5.0 * sample_rate),
        len(audio)
    )

    for ch in range(channels):

        if channels == 1:
            channel_audio = audio
        else:
            channel_audio = audio[:, ch]

        first_5_seconds = channel_audio[:first_5_end]

        full_mean = np.mean(channel_audio)

        centered_full = (
            channel_audio - full_mean
        )

        full_ac_power = np.mean(
            centered_full ** 2
        )

        full_raw_power = np.mean(
            channel_audio ** 2
        )

        nonzero_samples = np.count_nonzero(
            channel_audio
        )

        first5_mean = np.mean(
            first_5_seconds
        )

        centered_first5 = (
            first_5_seconds - first5_mean
        )

        first5_ac_power = np.mean(
            centered_first5 ** 2
        )

        print()
        print(f"Channel {ch}")
        print(f"Total samples: {len(channel_audio)}")

        print(
            f"Full-file min: "
            f"{np.min(channel_audio):.0f}"
        )

        print(
            f"Full-file max: "
            f"{np.max(channel_audio):.0f}"
        )

        print(
            f"Full-file mean/DC offset: "
            f"{full_mean:.2f}"
        )

        print(
            f"Full-file mean abs: "
            f"{np.mean(np.abs(channel_audio)):.2f}"
        )

        print(
            f"Full-file raw power: "
            f"{full_raw_power:.2f}"
        )

        print(
            f"Full-file AC power: "
            f"{full_ac_power:.2f}"
        )

        print("First 5 seconds:")

        print(
            f"  Min: "
            f"{np.min(first_5_seconds):.0f}"
        )

        print(
            f"  Max: "
            f"{np.max(first_5_seconds):.0f}"
        )

        print(
            f"  Mean/DC offset: "
            f"{first5_mean:.2f}"
        )

        print(
            f"  Mean abs: "
            f"{np.mean(np.abs(first_5_seconds)):.2f}"
        )

        print(
            f"  Raw power: "
            f"{np.mean(first_5_seconds ** 2):.2f}"
        )

        print(
            f"  AC power after DC removal: "
            f"{first5_ac_power:.2f}"
        )

        print(
            f"  Non-zero samples: "
            f"{np.count_nonzero(first_5_seconds)} / "
            f"{len(first_5_seconds)}"
        )

        # --------------------------------------------------
        # HARD DEAD-CHANNEL CHECKS
        # --------------------------------------------------

        assert nonzero_samples > 0, (
            f"\nDEAD CHANNEL: "
            f"Channel {ch} contains only zeros."
        )

        assert np.min(channel_audio) != np.max(channel_audio), (
            f"\nDEAD CHANNEL: "
            f"Channel {ch} is stuck at constant value "
            f"{channel_audio[0]:.0f}."
        )

        assert full_ac_power > 0, (
            f"\nDEAD CHANNEL: "
            f"Channel {ch} has zero AC power."
        )

    print("=" * 50)

    return sample_rate, channels, audio


def calculate_snr(
    signal,
    sample_rate,
    noise_start,
    noise_end,
    speech_start,
    speech_end,
):
    """
    Estimate SNR using:

        noise window   = noise only
        speech window  = speech + noise

    DC offset is removed from each window before
    calculating signal power.
    """

    noise_start_idx = int(
        noise_start * sample_rate
    )

    noise_end_idx = int(
        noise_end * sample_rate
    )

    speech_start_idx = int(
        speech_start * sample_rate
    )

    speech_end_idx = int(
        speech_end * sample_rate
    )

    noise_segment = signal[
        noise_start_idx:noise_end_idx
    ].astype(np.float64)

    speech_noise_segment = signal[
        speech_start_idx:speech_end_idx
    ].astype(np.float64)

    assert len(noise_segment) > 0, (
        "Noise segment is empty."
    )

    assert len(speech_noise_segment) > 0, (
        "Speech segment is empty."
    )

    # --------------------------------------------------
    # REMOVE DC OFFSET
    # --------------------------------------------------

    noise_dc = np.mean(
        noise_segment
    )

    speech_noise_dc = np.mean(
        speech_noise_segment
    )

    noise_centered = (
        noise_segment - noise_dc
    )

    speech_noise_centered = (
        speech_noise_segment -
        speech_noise_dc
    )

    # --------------------------------------------------
    # AC POWER
    # --------------------------------------------------

    noise_power = np.mean(
        noise_centered ** 2
    )

    speech_noise_power = np.mean(
        speech_noise_centered ** 2
    )

    assert noise_power > 0, (
        "Noise power is zero."
    )

    # Estimate speech-only power
    speech_power = (
        speech_noise_power -
        noise_power
    )

    # If this fails, your speech window had less
    # AC power than your noise-only window.
    assert speech_power > 0, (
        f"Unable to estimate positive speech power.\n"
        f"Noise power: {noise_power:.2f}\n"
        f"Speech+noise power: {speech_noise_power:.2f}\n"
        f"Estimated speech power: {speech_power:.2f}"
    )

    snr_db = 10.0 * np.log10(
        speech_power / noise_power
    )

    return {
        "snr_db": snr_db,
        "noise_power": noise_power,
        "speech_noise_power": speech_noise_power,
        "speech_power": speech_power,
        "noise_dc": noise_dc,
        "speech_noise_dc": speech_noise_dc,
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Calculate estimated SNR from a "
            "16-bit PCM WAV file."
        )
    )

    parser.add_argument(
        "wav_file",
        help="Input WAV file"
    )

    parser.add_argument(
        "--noise-start",
        type=float,
        default=0.0,
        help="Noise-only start time in seconds"
    )

    parser.add_argument(
        "--noise-end",
        type=float,
        default=5.0,
        help="Noise-only end time in seconds"
    )

    parser.add_argument(
        "--speech-start",
        type=float,
        default=5.0,
        help="Speech+noise start time in seconds"
    )

    parser.add_argument(
        "--speech-end",
        type=float,
        default=15.0,
        help="Speech+noise end time in seconds"
    )

    parser.add_argument(
        "--channel",
        type=int,
        default=None,
        help=(
            "Optional channel index to analyze. "
            "If omitted, all channels are analyzed."
        )
    )

    return parser.parse_args()


def main():
    args = parse_args()

    sample_rate, channels, audio = load_wav(
        args.wav_file
    )

    duration = len(audio) / sample_rate

    print()
    print("=" * 50)
    print("WAV INFORMATION")
    print("=" * 50)

    print(
        f"File:        {args.wav_file}"
    )

    print(
        f"Sample rate: {sample_rate} Hz"
    )

    print(
        f"Channels:    {channels}"
    )

    print(
        f"Duration:    {duration:.2f} seconds"
    )

    print()
    print("Analysis windows:")

    print(
        f"Noise only:     "
        f"{args.noise_start:.2f} - "
        f"{args.noise_end:.2f} s"
    )

    print(
        f"Speech + noise: "
        f"{args.speech_start:.2f} - "
        f"{args.speech_end:.2f} s"
    )

    print()
    print("=" * 50)
    print("SNR RESULTS")
    print("=" * 50)

    if channels == 1:

        channels_to_analyze = [0]

    elif args.channel is not None:

        assert (
            0 <= args.channel < channels
        ), (
            f"Channel must be between "
            f"0 and {channels - 1}."
        )

        channels_to_analyze = [
            args.channel
        ]

    else:

        channels_to_analyze = range(
            channels
        )

    for ch in channels_to_analyze:

        if channels == 1:
            signal = audio
        else:
            signal = audio[:, ch]

        result = calculate_snr(
            signal,
            sample_rate,
            args.noise_start,
            args.noise_end,
            args.speech_start,
            args.speech_end,
        )

        print()
        print(f"Channel {ch}")

        print(
            f"  Noise DC offset: "
            f"{result['noise_dc']:.2f}"
        )

        print(
            f"  Speech-window DC offset: "
            f"{result['speech_noise_dc']:.2f}"
        )

        print(
            f"  Noise AC power: "
            f"{result['noise_power']:.2f}"
        )

        print(
            f"  Speech+noise AC power: "
            f"{result['speech_noise_power']:.2f}"
        )

        print(
            f"  Estimated speech power: "
            f"{result['speech_power']:.2f}"
        )

        print(
            f"  Estimated SNR: "
            f"{result['snr_db']:.2f} dB"
        )


if __name__ == "__main__":
    main()
