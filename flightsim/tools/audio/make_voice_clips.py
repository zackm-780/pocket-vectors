#!/usr/bin/env python3
"""Generate the GPWS voice clips (S2) with CMU Flite.

    python3 tools/audio/make_voice_clips.py [--voice rms] [--out assets/audio/voice]

Needs the `flite` program (Debian / Ubuntu: apt install flite). Each phrase is spoken by Flite,
trimmed, band-limited a little (300 Hz - 3.4 kHz, like a cockpit speaker), normalised to -1 dBFS,
resampled to 22,050 Hz and written as 16-bit mono WAV, the format src/audio/VoiceBank.cpp loads.

Flite and its voices are Copyright Carnegie Mellon University and the University of Edinburgh,
under a BSD-style licence (see port/licenses/flite.txt); the clips are produced with them,
unmodified apart from the processing above (PLAN D22).
"""
from __future__ import annotations

import argparse
import array
import math
import shutil
import subprocess
import sys
import tempfile
import wave
from pathlib import Path

RATE_HZ = 22050

# file stem -> phrase. Stems are what VoiceBank expects (src/audio/VoiceBank.cpp voiceClipName).
PHRASES = {
    "minimums": "minimums",
    "2500": "twenty five hundred",
    "1000": "one thousand",
    "500": "five hundred",
    "100": "one hundred",
    "50": "fifty",
    "40": "forty",
    "30": "thirty",
    "20": "twenty",
    "10": "ten",
    "sink_rate": "sink rate",
    "pull_up": "pull up",
    "terrain": "terrain",
    "too_low_gear": "too low, gear",
    "too_low_flaps": "too low, flaps",
}


def read_wav(path: Path) -> tuple[list[float], int]:
    with wave.open(str(path), "rb") as w:
        if w.getsampwidth() != 2:
            raise SystemExit(f"{path}: expected 16-bit samples")
        rate, ch = w.getframerate(), w.getnchannels()
        data = array.array("h", w.readframes(w.getnframes()))
    if sys.byteorder == "big":
        data.byteswap()
    mono = [data[i] / 32768.0 for i in range(0, len(data), ch)]
    return mono, rate


def trim(x: list[float], threshold: float = 0.01, padS: float = 0.02, rate: int = RATE_HZ) -> list[float]:
    idx = [i for i, v in enumerate(x) if abs(v) > threshold]
    if not idx:
        return x
    pad = int(padS * rate)
    return x[max(0, idx[0] - pad): min(len(x), idx[-1] + pad)]


def one_pole_highpass(x: list[float], cutHz: float, rate: int) -> list[float]:
    a = math.exp(-2 * math.pi * cutHz / rate)
    out, prevX, prevY = [], 0.0, 0.0
    for v in x:
        y = a * (prevY + v - prevX)
        out.append(y)
        prevX, prevY = v, y
    return out


def one_pole_lowpass(x: list[float], cutHz: float, rate: int) -> list[float]:
    a = 1 - math.exp(-2 * math.pi * cutHz / rate)
    out, y = [], 0.0
    for v in x:
        y += a * (v - y)
        out.append(y)
    return out


def resample(x: list[float], src: int, dst: int) -> list[float]:
    if src == dst:
        return x
    n = int(len(x) * dst / src)
    out = []
    for i in range(n):
        t = i * src / dst
        j = int(t)
        f = t - j
        a = x[j] if j < len(x) else 0.0
        b = x[j + 1] if j + 1 < len(x) else a
        out.append(a + (b - a) * f)
    return out


def write_wav(path: Path, x: list[float]) -> None:
    peak = max((abs(v) for v in x), default=0.0) or 1.0
    gain = 10 ** (-1 / 20) / peak  # -1 dBFS
    fade = int(0.005 * RATE_HZ)
    data = array.array("h")
    for i, v in enumerate(x):
        env = min(1.0, i / fade, (len(x) - 1 - i) / fade) if fade else 1.0
        data.append(int(round(max(-1.0, min(1.0, v * gain * env)) * 32767)))
    if sys.byteorder == "big":
        data.byteswap()
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE_HZ)
        w.writeframes(data.tobytes())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--voice", default="rms", help="Flite voice (rms, slt, awb, kal)")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parents[2] / "assets/audio/voice"))
    args = ap.parse_args()
    flite = shutil.which("flite")
    if not flite:
        print("make_voice_clips: flite not found (apt install flite)", file=sys.stderr)
        return 2
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    total = 0
    with tempfile.TemporaryDirectory() as td:
        for stem, phrase in PHRASES.items():
            raw = Path(td) / f"{stem}.wav"
            subprocess.run([flite, "-voice", args.voice, "-t", phrase, "-o", str(raw)], check=True)
            x, rate = read_wav(raw)
            x = trim(x, rate=rate)
            x = one_pole_highpass(x, 300.0, rate)
            x = one_pole_lowpass(x, 3400.0, rate)
            x = resample(x, rate, RATE_HZ)
            dst = out / f"{stem}.wav"
            write_wav(dst, x)
            size = dst.stat().st_size
            total += size
            print(f"{dst.name:18s} {len(x) / RATE_HZ:5.2f} s  {size / 1024:6.1f} KiB  \"{phrase}\"")
    print(f"total {total / 1024:.0f} KiB in {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
