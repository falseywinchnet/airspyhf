#!/usr/bin/env python3
"""Compare two 16-bit stereo IQ WAV files and report codec distortion."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import struct
import wave


def read(path: Path) -> list[int]:
    with wave.open(str(path), "rb") as wav:
        if wav.getnchannels() != 2 or wav.getsampwidth() != 2:
            raise ValueError(f"{path}: expected stereo signed 16-bit PCM")
        raw = wav.readframes(wav.getnframes())
    return list(struct.unpack(f"<{len(raw) // 2}h", raw))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("reference", type=Path)
    parser.add_argument("decoded", type=Path)
    args = parser.parse_args()
    reference = read(args.reference)
    decoded = read(args.decoded)
    if len(reference) != len(decoded):
        raise ValueError("sample counts differ")
    errors = [b - a for a, b in zip(reference, decoded)]
    signal_energy = sum(value * value for value in reference)
    error_energy = sum(value * value for value in errors)
    snr = math.inf if error_energy == 0 else 10 * math.log10(signal_energy / error_energy)
    mean_abs = sum(abs(value) for value in errors) / len(errors)
    print(
        f"{args.decoded}: scalar_samples={len(errors)} max_abs_error={max(map(abs, errors))} "
        f"mean_abs_error={mean_abs:.4f} snr_db={snr:.3f} exact={error_energy == 0}"
    )


if __name__ == "__main__":
    main()

