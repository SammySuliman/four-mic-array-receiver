import argparse
import wave

import numpy as np


def load_wav(filename):
    with wave.open(filename, "rb") as wf:
        channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        sample_rate = wf.getframerate()
        num_frames = wf.getnframes()

        raw_audio = wf.readframes(num_frames)

    if sample_width != 2:
        raise ValueError(
            f"Expected 16-bit PCM WAV, but sample width is {sample_width} bytes."
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

    noise_end_idx = min(
        int(5 * sample_rate),
        len(audio)
    )

    for ch in range(channels):

        if channels == 1:
            channel_audio = audio
        else:
            channel_audio = audio[:, ch]

        first_5_seconds = channel_audio[:noise_end_idx]

        print(f"\nChannel {ch}")
        print(f"Total samples: {len(channel_audio)}")

        print(
            f"Full-file min: {np.min(channel_audio):.0f}"
        )
        print(
            f"Full-file max: {np.max(channel_audio):.0f}"
        )
        print(
            f"Full-file mean abs: "
            f"{np.mean(np.abs(channel_audio)):.2f}"
        )
        print(
            f"Full-file power: "
            f"{np.mean(channel_audio ** 2):.2f}"
        )

        print("First 5 seconds:")

        print(
            f"  Min: {np.min(first_5_seconds):.0f}"
        )
        print(
            f"  Max: {np.max(first_5_seconds):.0f}"
        )
        print(
            f"  Mean abs: "
            f"{np.mean(np.abs(first_5_seconds)):.2f}"
        )
        print(
            f"  Power: "
            f"{np.mean(first_5_seconds ** 2):.2f}"
        )

        nonzero_samples = np.count_nonzero(
            first_5_seconds
        )

        print(
            f"  Non-zero samples: "
            f"{nonzero_samples} / "
            f"{len(first_5_seconds)}"
        )

        if nonzero_samples == 0:
            print(
                "  WARNING: first 5 seconds are "
                "completely zero on this channel."
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
        noise window      = noise only
        speech window     = speech + noise

    Assumes approximately stationary background noise.

    SNR = 10 * log10(P_speech / P_noise)

    where:
        P_speech ≈ P_speech+noise - P_noise
    """

    noise_start_idx = int(noise_start * sample_rate)
    noise_end_idx = int(noise_end * sample_rate)

    speech_start_idx = int(speech_start * sample_rate)
    speech_end_idx = int(speech_end * sample_rate)

    noise_segment = signal[noise_start_idx:noise_end_idx]
    speech_noise_segment = signal[speech_start_idx:speech_end_idx]

    if len(noise_segment) == 0:
        raise ValueError("Noise segment is empty.")

    if len(speech_noise_segment) == 0:
        raise ValueError("Speech segment is empty.")

    # Mean-square power
    noise_power = np.mean(noise_segment ** 2)
    speech_noise_power = np.mean(speech_noise_segment ** 2)

    # Estimate actual speech power
    speech_power = speech_noise_power - noise_power

    if noise_power <= 0:
        raise ValueError("Noise power is zero or invalid.")

    if speech_power <= 0:
        return {
            "snr_db": None,
            "noise_power": noise_power,
            "speech_noise_power": speech_noise_power,
            "speech_power": speech_power,
        }

    snr_db = 10.0 * np.log10(speech_power / noise_power)

    return {
        "snr_db": snr_db,
        "noise_power": noise_power,
        "speech_noise_power": speech_noise_power,
        "speech_power": speech_power,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Estimate SNR from a WAV file."
    )

    parser.add_argument(
        "wav_file",
        help="Path to WAV file"
    )

    parser.add_argument(
        "--noise-start",
        type=float,
        default=0.0,
        help="Start time of noise-only section in seconds"
    )

    parser.add_argument(
        "--noise-end",
        type=float,
        default=5.0,
        help="End time of noise-only section in seconds"
    )

    parser.add_argument(
        "--speech-start",
        type=float,
        default=5.0,
        help="Start time of speech+noise section in seconds"
    )

    parser.add_argument(
        "--speech-end",
        type=float,
        default=15.0,
        help="End time of speech+noise section in seconds"
    )

    parser.add_argument(
        "--channel",
        type=int,
        default=None,
        help=(
            "Channel index to analyze. "
            "If omitted, all channels are analyzed."
        )
    )

    args = parser.parse_args()

    sample_rate, channels, audio = load_wav(args.wav_file)

    duration = len(audio) / sample_rate

    print("=" * 50)
    print("WAV INFORMATION")
    print("=" * 50)

    print(f"File:        {args.wav_file}")
    print(f"Sample rate: {sample_rate} Hz")
    print(f"Channels:    {channels}")
    print(f"Duration:    {duration:.2f} seconds")

    print()
    print("Analysis windows:")
    print(
        f"Noise only:     "
        f"{args.noise_start:.2f} - {args.noise_end:.2f} s"
    )
    print(
        f"Speech + noise: "
        f"{args.speech_start:.2f} - {args.speech_end:.2f} s"
    )

    print()
    print("=" * 50)
    print("SNR RESULTS")
    print("=" * 50)

    if channels == 1:
        signal = audio

        result = calculate_snr(
            signal,
            sample_rate,
            args.noise_start,
            args.noise_end,
            args.speech_start,
            args.speech_end,
        )

        if result["snr_db"] is None:
            print(
                "Could not estimate a positive speech power.\n"
                "The speech+noise section may not be louder "
                "than the noise-only section."
            )
        else:
            print(f"Estimated SNR: {result['snr_db']:.2f} dB")

        print(f"Noise power:        {result['noise_power']:.2f}")
        print(
            f"Speech+noise power: "
            f"{result['speech_noise_power']:.2f}"
        )
        print(f"Estimated speech power: {result['speech_power']:.2f}")

    else:
        if args.channel is not None:
            if args.channel < 0 or args.channel >= channels:
                raise ValueError(
                    f"Channel must be between 0 and {channels - 1}."
                )

            channels_to_test = [args.channel]

        else:
            channels_to_test = range(channels)

        for channel in channels_to_test:
            signal = audio[:, channel]

            result = calculate_snr(
                signal,
                sample_rate,
                args.noise_start,
                args.noise_end,
                args.speech_start,
                args.speech_end,
            )

            print()
            print(f"Channel {channel}")

            if result["snr_db"] is None:
                print(
                    "  SNR: could not estimate positive speech power"
                )
            else:
                print(
                    f"  Estimated SNR: "
                    f"{result['snr_db']:.2f} dB"
                )

            print(
                f"  Noise power: "
                f"{result['noise_power']:.2f}"
            )

            print(
                f"  Speech+noise power: "
                f"{result['speech_noise_power']:.2f}"
            )

            print(
                f"  Estimated speech power: "
                f"{result['speech_power']:.2f}"
            )


if __name__ == "__main__":
    main()
