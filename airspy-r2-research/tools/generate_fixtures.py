#!/usr/bin/env python3
"""Generate deterministic stereo PCM WAV fixtures for codec experiments."""

from __future__ import annotations

import math
from pathlib import Path
import random
import struct
import wave


SAMPLE_RATE = 10_000_000
SAMPLE_COUNT = 4096


def clamp16(value: int) -> int:
    return max(-32768, min(32767, value))


def write_iq(path: Path, samples: list[tuple[int, int]]) -> None:
    with wave.open(str(path), "wb") as out:
        out.setnchannels(2)
        out.setsampwidth(2)
        out.setframerate(SAMPLE_RATE)
        out.writeframes(b"".join(struct.pack("<hh", clamp16(i), clamp16(q)) for i, q in samples))


def main() -> None:
    output = Path(__file__).resolve().parents[1] / "analysis" / "generated" / "fixtures"
    output.mkdir(parents=True, exist_ok=True)
    rng = random.Random(0xA15F1)

    fixtures: dict[str, list[tuple[int, int]]] = {
        "zero": [(0, 0)] * SAMPLE_COUNT,
        "constant": [(1000, -1000)] * SAMPLE_COUNT,
        "ramp": [((n % 256) - 128, 127 - (n % 256)) for n in range(SAMPLE_COUNT)],
        "impulse": [(30000 if n == 1024 else 0, -20000 if n == 2048 else 0) for n in range(SAMPLE_COUNT)],
        "tone": [
            (round(12000 * math.cos(2 * math.pi * 37 * n / 1024)),
             round(12000 * math.sin(2 * math.pi * 37 * n / 1024)))
            for n in range(SAMPLE_COUNT)
        ],
        "noise12": [(rng.randrange(-2048, 2048) << 4, rng.randrange(-2048, 2048) << 4) for _ in range(SAMPLE_COUNT)],
    }
    for name, samples in fixtures.items():
        write_iq(output / f"{name}.wav", samples)


if __name__ == "__main__":
    main()

