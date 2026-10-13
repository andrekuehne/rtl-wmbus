#!/usr/bin/env python3
"""Replay checked-in fixtures against reviewed output; never refresh goldens."""

import argparse
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


def normalize(stdout):
    rows = []
    for row in stdout.splitlines(keepends=True):
        fields = row.split(b";")
        if len(fields) != 8 or fields[0] not in (b"T1", b"C1", b"S1"):
            raise ValueError("Unexpected packet row: " + repr(row))
        if not re.fullmatch(rb"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{6}", fields[3]):
            raise ValueError("Unexpected timestamp: " + repr(fields[3]))
        fields[3] = b"<timestamp>"
        rows.append(b";".join(fields))
    return b"".join(rows)


def compare(stdout, stderr, returncode, expected):
    if returncode != 0:
        raise AssertionError("Decoder exit status: " + str(returncode))
    if stderr:
        raise AssertionError("Unexpected stderr: " + repr(stderr))
    actual = normalize(stdout)
    if actual != expected:
        diff = difflib.unified_diff(
            expected.decode().splitlines(True), actual.decode().splitlines(True),
            fromfile="baseline", tofile="candidate",
        )
        raise AssertionError("".join(diff))
    return actual


def checked_bytes(path, expected_hash, compressed=False):
    data = gzip.decompress(path.read_bytes()) if compressed else path.read_bytes()
    if hashlib.sha256(data).hexdigest() != expected_hash:
        raise ValueError("Checksum mismatch: " + str(path))
    return data


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=root / "build/rtl_wmbus")
    parser.add_argument("--case", action="append", help="Select a case; repeatable")
    parser.add_argument("--output", type=Path, default=root / "build/test-results")
    args = parser.parse_args()
    binary = args.binary.resolve()
    output = args.output.resolve()
    if output.is_relative_to(root / "tests"):
        parser.error("Results must be outside the reviewed tests directory")
    manifest = json.loads((root / "tests/fixtures/manifest.json").read_text())
    cases = manifest["cases"]
    if args.case:
        if set(args.case) - {case["name"] for case in cases}:
            parser.error("Unknown capture case")
        cases = [case for case in cases if case["name"] in args.case]
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for case in cases:
        name = case["name"]
        try:
            data = checked_bytes(root / case["input"], case["sha256"], case.get("gzip", False))
            expected = checked_bytes(root / case["expected"], case["expected_sha256"])
            result = subprocess.run(
                [str(binary)] + case["options"], input=data, capture_output=True,
                cwd=output, env=dict(os.environ, LC_ALL="C", TZ="UTC"), timeout=60,
            )
            (output / (name + ".stdout")).write_bytes(result.stdout)
            (output / (name + ".stderr")).write_bytes(result.stderr)
            actual = compare(result.stdout, result.stderr, result.returncode, expected)
            (output / (name + ".normalized")).write_bytes(actual)
            print("PASS " + name, flush=True)
            results.append({"name": name, "passed": True})
        except (AssertionError, ValueError, OSError, subprocess.TimeoutExpired) as error:
            print("FAIL " + name + ": " + str(error), flush=True)
            results.append({"name": name, "passed": False, "error": str(error)})
    (output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    return int(any(not result["passed"] for result in results))


if __name__ == "__main__":
    raise SystemExit(main())
