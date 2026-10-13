#!/usr/bin/env python3
"""Synthetic C1 mode A/B frames as unsigned 8-bit I/Q, and a sensitivity sweep.

Frames follow the decoder's framing (t1_c1_packet_decoder.h): 0101... preamble,
0x543D sync, 0x54CD (frame A) or 0x543D (frame B) mode word, L-field and
CRC-16/EN-13757 blocks. NRZ 2-FSK, bit 1 = +deviation. Noise is complex white
Gaussian noise specified as Eb/N0, so streams at different sample rates carry
the same noise density.
"""

import argparse
import json
import math
import os
from pathlib import Path
import random
import statistics
import subprocess
import sys
import tempfile

CHIP_RATE = 100e3
DEVIATION = 45e3
PREAMBLE_BITS = 32
SYNC = 0x543D
MODE_WORD = {"A": 0x54CD, "B": 0x543D}

# T1/C1 mode A telegram length including CRC bytes, indexed by L-field.
def full_length_a(l_field):
    if l_field < 10:
        return l_field + 3
    return 12 + (l_field - 9) + 2 * ((l_field - 9 + 15) // 16)


def crc_wmbus(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x3D65) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc ^ 0xFFFF


def frame_bytes(mode, l_field, serial, rng):
    """Return the over-the-air bytes after the mode word and the decoder's
    cooked payload (CRCs stripped, mode B L-field reduced by 2 per block)."""
    header = [l_field, 0x44, 0x2D, 0x2C] + list(serial.to_bytes(4, "little")) + [0x1B, 0x16]
    if mode == "A":
        total = full_length_a(l_field)
        data = header + [rng.randrange(256) for _ in range(l_field + 1 - 10)]
        air, cooked, at = [], [], 0
        block = 10
        while at < len(data):
            chunk = data[at:at + block]
            crc = crc_wmbus(chunk)
            air += chunk + [crc >> 8, crc & 255]
            cooked += chunk
            at += block
            block = 16
        assert len(air) == total
        return air, cooked
    total = 1 + l_field
    data = header + [rng.randrange(256) for _ in range(total - 10 - 2 * (1 + (total > 128)))]
    air, cooked = [], []
    first = data[:126]
    crc = crc_wmbus(first)
    air += first + [crc >> 8, crc & 255]
    cooked += first
    blocks = 1
    if len(data) > 126:
        rest = data[126:]
        crc = crc_wmbus(rest)
        air += rest + [crc >> 8, crc & 255]
        cooked += rest
        blocks += 1
    assert len(air) == total
    cooked[0] -= 2 * blocks
    return air, cooked


def frame_bits(mode, air):
    bits = [i & 1 for i in range(PREAMBLE_BITS)]
    for word in (SYNC, MODE_WORD[mode]):
        bits += [(word >> (15 - i)) & 1 for i in range(16)]
    for byte in air:
        bits += [(byte >> (7 - i)) & 1 for i in range(8)]
    return bits + [i & 1 for i in range(8)]


def make_frames(count, seed):
    """Mode A/B frames with short, medium and maximum L-fields."""
    rng = random.Random(seed)
    lengths = {"A": [9, 20, 50, 120, 255], "B": [11, 40, 127, 140, 255]}
    frames = []
    for n in range(count):
        mode = "AB"[n % 2]
        l_field = lengths[mode][(n // 2) % 5]
        serial = 0x27000000 + seed % 1000 * 1000 + n
        air, cooked = frame_bytes(mode, l_field, serial, rng)
        frames.append({"mode": mode, "l_field": l_field, "bits": frame_bits(mode, air),
                       "key": ["C1", "%08X" % serial, "0x" + bytes(cooked).hex()]})
    return frames


def modulate(frames, sample_rate, offset_hz=0.0, chip_error=0.0, ebn0_db=None,
             amplitude=24.0, gap_bits=200, lead_bits=2000, seed=1, noise_density=None):
    """FSK-modulate frames separated by noise-only gaps into u8 I/Q bytes.

    Noise is given either relative to the signal (ebn0_db) or as an absolute
    density: noise_density is the per-component standard deviation in LSB at
    1.6 MS/s, scaled with the sample rate to keep the density constant."""
    rng = random.Random(seed)
    chip_rate = CHIP_RATE * (1 + chip_error)
    sigma = 0.0
    if ebn0_db is not None:
        sigma = amplitude * math.sqrt(sample_rate / (2 * CHIP_RATE * 10 ** (ebn0_db / 10)))
    elif noise_density is not None:
        sigma = noise_density * math.sqrt(sample_rate / 1.6e6)
    gauss, cos, sin = rng.gauss, math.cos, math.sin
    out = bytearray()
    phase = rng.uniform(0, 2 * math.pi)
    step = 2 * math.pi / sample_rate
    time = 0.0
    lead = lead_bits / chip_rate

    def emit(duration, bits):
        nonlocal phase, time
        start = time
        n0 = int(math.ceil(start * sample_rate - 1e-9))
        n1 = int(math.ceil((start + duration) * sample_rate - 1e-9))
        for n in range(n0, n1):
            if bits:
                index = int((n / sample_rate - start) * chip_rate)
                frequency = offset_hz + (DEVIATION if bits[min(index, len(bits) - 1)] else -DEVIATION)
                i = amplitude * cos(phase)
                q = amplitude * sin(phase)
                phase += step * frequency
            else:
                i = q = 0.0
            if sigma:
                i += gauss(0.0, sigma)
                q += gauss(0.0, sigma)
            out.append(min(255, max(0, int(math.floor(127.5 + i + 0.5)))))
            out.append(min(255, max(0, int(math.floor(127.5 + q + 0.5)))))
        time = start + duration

    emit(lead, None)
    for frame in frames:
        # Random fractional bit alignment per frame.
        emit(rng.uniform(0, 1) / chip_rate + gap_bits / chip_rate, None)
        emit(len(frame["bits"]) / chip_rate, frame["bits"])
    emit(gap_bits / chip_rate, None)
    out += bytes(4096 - len(out) % 4096) if len(out) % 4096 else b""
    return bytes(out)


def decode(binary, options, data):
    result = subprocess.run([str(binary)] + options + ["-v"], input=data, capture_output=True,
                            env=dict(os.environ, LC_ALL="C", TZ="UTC"), timeout=600)
    if result.returncode or result.stderr:
        raise RuntimeError("decoder failed: %r %r" % (result.returncode, result.stderr))
    rows = []
    for line in result.stdout.decode().splitlines():
        fields = line.split(";")
        rows.append({"algorithm": fields[0], "mode": fields[1], "crc_ok": fields[2] == "1",
                     "rssi": int(fields[5]), "key": [fields[1], fields[7], fields[8]]})
    return rows


def score(frames, rows):
    sent = {tuple(frame["key"]) for frame in frames}
    ok = [row for row in rows if row["crc_ok"]]
    decoded = {tuple(row["key"]) for row in ok}
    by_algorithm = {}
    for name in ("rla", "t2a"):
        found = {tuple(row["key"]) for row in ok if row["algorithm"] == name}
        by_algorithm[name] = len(sent - found)
    return {"missed": len(sent - decoded), "extra": len(decoded - sent),
            "crc_failed": sum(not row["crc_ok"] for row in rows),
            "missed_by_algorithm": by_algorithm,
            "rssi": statistics.median(row["rssi"] for row in ok) if ok else None,
            "decoded": sorted("%s;%s" % (k[1], k[2][:12]) for k in decoded)}


PATHS = {
    "default": (1600000, ["-p", "S"]),
    "fast4": (1600000, ["-c", "4"]),
    "fast3": (1200000, ["-c", "3"]),
}


def run_point(point):
    binary, condition = point["binary"], point["condition"]
    frames = make_frames(condition["frames"], condition["frame_seed"])
    streams = {}
    result = dict(condition)
    for name in condition["paths"]:
        rate, options = PATHS[name]
        if rate not in streams:
            streams[rate] = modulate(frames, rate, condition["offset_hz"], condition["chip_error"],
                                     condition["ebn0_db"], condition["amplitude"],
                                     seed=condition["noise_seed"] + rate,
                                     noise_density=condition.get("noise_density"))
        result[name] = score(frames, decode(binary, condition["options"] + options, streams[rate]))
    return result


def sweep(args):
    import multiprocessing
    conditions = []
    base = {"frames": args.frames, "amplitude": args.amplitude, "chip_error": 0.0,
            "options": args.options, "paths": args.paths}
    ebn0 = [args.ebn0_min + 0.5 * k for k in range(int((args.ebn0_max - args.ebn0_min) / 0.5) + 1)]
    for offset in args.offsets:
        for snr in ebn0:
            conditions.append(dict(base, group="offset", offset_hz=offset * 1e3, ebn0_db=snr))
    for chip in args.chip_errors:
        for snr in ebn0:
            conditions.append(dict(base, group="chip", offset_hz=0.0, chip_error=chip / 100, ebn0_db=snr))
    # Weak signals near the RSSI capture threshold over a fixed noise floor.
    for density in args.noise_densities:
        for amplitude in args.amplitudes:
            conditions.append(dict(base, group="amplitude", offset_hz=0.0, ebn0_db=None,
                                   amplitude=amplitude, noise_density=density))
    for k, condition in enumerate(conditions):
        condition["frame_seed"] = args.seed + k
        condition["noise_seed"] = 50 * args.seed + 7 * k
    points = [{"binary": str(Path(args.binary).resolve()), "condition": c} for c in conditions]
    with multiprocessing.Pool(args.jobs) as pool:
        results = []
        for n, result in enumerate(pool.imap(run_point, points)):
            results.append(result)
            print("%d/%d %s offset=%g chip=%g ebn0=%s amp=%g %s" % (
                n + 1, len(points), result["group"], result["offset_hz"], result["chip_error"],
                result["ebn0_db"], result["amplitude"],
                " ".join("%s:%d/%d" % (p, result[p]["missed"], result[p]["crc_failed"]) for p in args.paths)),
                flush=True)
    Path(args.output).write_text(json.dumps(results, indent=1) + "\n")


def first_drop(points, path, key):
    """Highest SNR (or amplitude) at which a frame is missed or CRC-failed,
    and the lowest point of the swept range above which nothing is lost."""
    lost = [p[key] for p in points if p[path]["missed"] or p[path]["extra"]]
    return max(lost) if lost else None


def report(args):
    results = json.loads(Path(args.input).read_text())
    paths = [p for p in ("default", "fast4", "fast3") if p in results[0]]
    print("First-drop SNR (Eb/N0 dB): highest point with a missed or extra frame; '-' = none lost")
    rows = []
    for group, label, key in (("offset", "offset_hz", "ebn0_db"), ("chip", "chip_error", "ebn0_db"),
                              ("amplitude", "noise_density", "amplitude")):
        conditions = sorted({p[label] for p in results if p["group"] == group})
        for value in conditions:
            points = [p for p in results if p["group"] == group and p[label] == value]
            drops = {path: first_drop(points, path, key) for path in paths}
            line = "%-9s %-13s " % (group, "%s=%g" % (label, value / 1e3 if label == "offset_hz" else
                                                    value * 100 if label == "chip_error" else value))
            line += " ".join("%s %5s" % (path, "-" if drops[path] is None else "%g" % drops[path]) for path in paths)
            if "default" in drops and drops["default"] is not None:
                for path in paths[1:]:
                    if drops[path] is not None:
                        line += "  %s-default %+g" % (path, drops[path] - drops["default"])
            totals = {path: sum(p[path]["missed"] for p in points) for path in paths}
            failed = {path: sum(p[path]["crc_failed"] for p in points) for path in paths}
            line += "  | missed " + "/".join(str(totals[path]) for path in paths)
            line += " crc-failed rows " + "/".join(str(failed[path]) for path in paths)
            print(line)
            rows.append(dict(group=group, value=value, first_drop=drops, missed=totals, crc_failed=failed))
    if args.output:
        Path(args.output).write_text(json.dumps(rows, indent=1) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    gen = sub.add_parser("generate", help="Write one stream and its expected CRC-OK rows")
    gen.add_argument("--rate", type=int, default=1600000)
    gen.add_argument("--frames", type=int, default=10)
    gen.add_argument("--seed", type=int, default=1)
    gen.add_argument("--offset-khz", type=float, default=0.0)
    gen.add_argument("--chip-error", type=float, default=0.0, help="percent")
    gen.add_argument("--ebn0", type=float, help="dB; omit for no noise")
    gen.add_argument("--amplitude", type=float, default=24.0)
    gen.add_argument("output", type=Path)
    sw = sub.add_parser("sweep", help="Compare paths over offset, chip-rate, SNR and amplitude")
    sw.add_argument("--binary", required=True)
    sw.add_argument("--output", required=True)
    sw.add_argument("--paths", nargs="+", default=["default", "fast4", "fast3"])
    sw.add_argument("--options", nargs="*", default=["-o"])
    sw.add_argument("--frames", type=int, default=20)
    sw.add_argument("--amplitude", type=float, default=24.0)
    sw.add_argument("--offsets", type=float, nargs="*", default=[0, 25, -25, 50, -50, 75, -75])
    sw.add_argument("--chip-errors", type=float, nargs="*", default=[2, -2, 10, -10])
    sw.add_argument("--amplitudes", type=float, nargs="*", default=[6, 7, 8, 9, 10, 11, 12, 14, 16])
    sw.add_argument("--noise-densities", type=float, nargs="*", default=[2.0, 4.0],
                    help="per-component noise LSB at 1.6 MS/s for the amplitude group")
    sw.add_argument("--seed", type=int, default=1000)
    sw.add_argument("--ebn0-min", type=float, default=4.0)
    sw.add_argument("--ebn0-max", type=float, default=16.0)
    sw.add_argument("--jobs", type=int, default=os.cpu_count())
    rp = sub.add_parser("report", help="Summarize a sweep: first-dropped-frame SNR per condition")
    rp.add_argument("input")
    rp.add_argument("--output")
    args = parser.parse_args()
    if args.command == "sweep":
        return sweep(args)
    if args.command == "report":
        return report(args)
    frames = make_frames(args.frames, args.seed)
    data = modulate(frames, args.rate, args.offset_khz * 1e3, args.chip_error / 100, args.ebn0,
                    args.amplitude, seed=args.seed)
    args.output.write_bytes(data)
    json.dump([frame["key"] for frame in frames], sys.stdout, indent=1)
    print()


if __name__ == "__main__":
    raise SystemExit(main())
