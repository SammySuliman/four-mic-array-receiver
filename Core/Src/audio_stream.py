"""Record validated four-channel packets from the receiver board.

The receiver firmware must forward each valid SPI AudioFrame_t packet to a
PC-facing serial port for this script to read it. This script does not access
SPI directly through the STM32 board's GPIO pins.

CombinedAudioFrame_t wire format, little-endian:
    uint8_t  magic0       = 0xCC
    uint8_t  magic1       = 0x33
    uint16_t frame_id
    uint16_t payload_size = 2048
    int16_t  samples[1024] (256 four-channel frames, interleaved)
    uint16_t checksum      (sum of the first 2054 bytes)
"""

import argparse
import struct
import sys
import time
import wave

import numpy as np
import serial
import serial.tools.list_ports
from serial.serialutil import SerialException


MAGIC = b"\xCC\x33"
AUDIO_FRAME_SAMPLES = 256
AUDIO_CHANNELS = 4
AUDIO_BYTES_PER_SAMPLE = 2
PAYLOAD_SIZE = AUDIO_FRAME_SAMPLES * AUDIO_CHANNELS * AUDIO_BYTES_PER_SAMPLE
HEADER_SIZE = 6
CHECKSUM_SIZE = 2
FRAME_SIZE = HEADER_SIZE + PAYLOAD_SIZE + CHECKSUM_SIZE
SAMPLE_RATE = 16_000


def find_stm32_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        description = (port.description or "").lower()
        manufacturer = (port.manufacturer or "").lower()
        if "stm32" in description or "stm32" in manufacturer:
            return port.device
        if "usb serial" in description or "virtual com" in description:
            return port.device

    if len(ports) == 1:
        return ports[0].device

    return None


def checksum_u8(data):
    return sum(data) & 0xFFFF


def extract_frame(buffer):
    """Return (frame_id, samples) when a valid frame is available."""
    while True:
        magic_index = buffer.find(MAGIC)
        if magic_index < 0:
            # Keep a possible first magic byte for the next serial read.
            if buffer[-1:] == MAGIC[:1]:
                del buffer[:-1]
            else:
                buffer.clear()
            return None

        if magic_index:
            del buffer[:magic_index]

        if len(buffer) < FRAME_SIZE:
            return None

        candidate = bytes(buffer[:FRAME_SIZE])
        frame_id, payload_size = struct.unpack_from("<HH", candidate, 2)
        received_checksum = struct.unpack_from("<H", candidate, FRAME_SIZE - 2)[0]

        if payload_size != PAYLOAD_SIZE:
            del buffer[:2]
            continue

        if checksum_u8(candidate[:-CHECKSUM_SIZE]) != received_checksum:
            # Discard one byte and search again so a corrupted frame can be
            # recovered without restarting the script.
            del buffer[:1]
            continue

        samples = np.frombuffer(
            candidate[HEADER_SIZE:HEADER_SIZE + PAYLOAD_SIZE],
            dtype="<i2",
        ).copy()
        del buffer[:FRAME_SIZE]
        return frame_id, samples


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record validated STM32 receiver audio frames to WAV."
    )
    parser.add_argument("--port", help="Receiver PC serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=2000000)
    parser.add_argument("--output", default="received_audio.wav")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="Recording duration; 0 means until Ctrl+C")
    return parser.parse_args()


def main():
    args = parse_args()
    port = args.port or find_stm32_port()

    if not port:
        print("No serial port found. Use --port COMx.", file=sys.stderr)
        return 1

    print(f"Opening {port} at {args.baud} baud")
    ser = serial.Serial(port, args.baud, timeout=0.1)
    buffer = bytearray()
    recorded_samples = []
    valid_frames = 0
    bad_or_skipped_frames = 0
    previous_frame_id = None
    start_time = time.monotonic()

    try:
        print("Recording validated audio. Press Ctrl+C to stop.")
        while True:
            # A timed read avoids ser.in_waiting, which can fail when the
            # ST-LINK Virtual COM Port briefly re-enumerates after reset.
            chunk = ser.read(4096)
            if chunk:
                buffer.extend(chunk)

            while True:
                parsed = extract_frame(buffer)
                if parsed is None:
                    break

                frame_id, samples = parsed
                if previous_frame_id is not None:
                    expected = (previous_frame_id + 1) & 0xFFFF
                    if frame_id != expected:
                        bad_or_skipped_frames += (frame_id - expected) & 0xFFFF
                previous_frame_id = frame_id
                recorded_samples.append(samples)
                valid_frames += 1

                if valid_frames % 16 == 0:
                    print(f"Valid frames: {valid_frames}, last ID: {frame_id}")

            if args.seconds > 0 and time.monotonic() - start_time >= args.seconds:
                break

    except KeyboardInterrupt:
        print("\nStopping reception.")
    except SerialException as exc:
        print(f"\nSerial connection lost on {port}: {exc}", file=sys.stderr)
        print("Reconnect the receiver, confirm its current COM port, and retry.",
              file=sys.stderr)
        return 2
    finally:
        ser.close()

    if not recorded_samples:
        print("No valid audio frames received.", file=sys.stderr)
        return 1

    all_samples = np.concatenate(recorded_samples).astype("<i2", copy=False)
    with wave.open(args.output, "wb") as wav_file:
        wav_file.setnchannels(AUDIO_CHANNELS)
        wav_file.setsampwidth(AUDIO_BYTES_PER_SAMPLE)
        wav_file.setframerate(SAMPLE_RATE)
        wav_file.writeframes(all_samples.tobytes())

    stereo_frames = len(all_samples) // AUDIO_CHANNELS
    duration = stereo_frames / SAMPLE_RATE
    print(f"Saved {args.output}")
    print(f"Valid frames: {valid_frames}")
    print(f"Skipped frames: {bad_or_skipped_frames}")
    print(f"Audio duration: {duration:.2f} seconds")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
