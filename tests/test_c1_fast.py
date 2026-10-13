"""Opt-in C1 fast path (-c R): frame sets, streaming and option checks."""

import json
import os
from pathlib import Path
import subprocess
import time
import unittest

import c1_synth
from replay_captures import checked_bytes, normalize


ROOT = Path(__file__).resolve().parents[1]
BINARY = Path(os.environ.get("RTL_WMBUS_TEST_BINARY", ROOT / "build/rtl_wmbus")).resolve()
CASES = {case["name"]: case for case in json.loads((ROOT / "tests/fixtures/manifest.json").read_text())["cases"]}
ENV = dict(os.environ, LC_ALL="C", TZ="UTC")


def expected(case):
    return checked_bytes(ROOT / case["expected"], case["expected_sha256"])


def crc_ok_rows(output):
    """(mode, serial, payload) of CRC-valid rows, ignoring RSSI and duplicates."""
    rows = set()
    for row in output.splitlines():
        fields = row.split(b";")
        if fields[-7] == b"1":
            rows.add((fields[-8], fields[-2], fields[-1]))
    return rows


def run(options, data, timeout=60):
    return subprocess.run([str(BINARY)] + options, input=data, capture_output=True, env=ENV, timeout=timeout)


class FastPathChecks(unittest.TestCase):
    def test_fast_path_keeps_every_default_crc_ok_row(self):
        fast_cases = [case for case in CASES.values() if "fast_path_of" in case]
        self.assertEqual(len(fast_cases), 4)
        for case in fast_cases:
            with self.subTest(case=case["name"]):
                default = crc_ok_rows(expected(CASES[case["fast_path_of"]]))
                fast = crc_ok_rows(expected(case))
                self.assertTrue(default)
                self.assertLessEqual(default, fast)

    def test_synthetic_frames_at_both_rates(self):
        frames = c1_synth.make_frames(10, 27)
        sent = {tuple(frame["key"]) for frame in frames}
        # Each fast-path algorithm alone decodes every frame without CRC-failed
        # rows; the default path needs both (its run-length path misses some
        # synthetic frames and reports CRC errors).
        for rate, options, algorithms in ((1600000, ["-p", "S"], [("rla", "t2a")]),
                                          (1600000, ["-c", "4"], [("rla",), ("t2a",)]),
                                          (1200000, ["-c", "3"], [("rla",), ("t2a",)])):
            data = c1_synth.modulate(frames, rate, offset_hz=10e3, ebn0_db=22, seed=rate)
            rows = c1_synth.decode(BINARY, ["-o"] + options, data)
            for algorithm in algorithms:
                with self.subTest(rate=rate, options=options, algorithm=algorithm):
                    decoded = {tuple(row["key"]) for row in rows if row["crc_ok"] and row["algorithm"] in algorithm}
                    self.assertEqual(decoded, sent)
                    if len(algorithm) == 1:
                        self.assertFalse([row for row in rows if not row["crc_ok"] and row["algorithm"] in algorithm])

    def test_fragmented_pipe_partial_block_and_phase_across_blocks(self):
        # 2048-sample blocks are not a multiple of R = 3; the partial final
        # block is ignored as in the default path.
        frames = c1_synth.make_frames(6, 3)
        for rate, rate_option in ((1200000, "3"), (1600000, "4")):
            data = c1_synth.modulate(frames, rate, ebn0_db=22, seed=5)
            self.assertEqual(len(data) % 4096, 0)
            whole = run(["-o", "-c", rate_option], data)
            self.assertEqual(whole.returncode, 0)
            self.assertEqual(len(crc_ok_rows(normalize(whole.stdout))), len(frames))
            with self.subTest(rate=rate):
                with subprocess.Popen([str(BINARY), "-o", "-c", rate_option], stdin=subprocess.PIPE,
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=ENV) as process:
                    stream = data + bytes([0, 255]) * 2047 + b"\xff"
                    for offset in range(0, len(stream), 997):
                        process.stdin.write(stream[offset:offset + 997])
                    process.stdin.close()
                    process.stdin = None
                    stdout, stderr = process.communicate(timeout=60)
                self.assertEqual(process.returncode, 0)
                self.assertEqual(stderr, b"")
                self.assertEqual(normalize(stdout), normalize(whole.stdout))

    @unittest.skipUnless(os.name == "posix", "POSIX flow monitoring")
    def test_flow_monitor(self):
        case = CASES["fast-tracked-issue49"]
        data = checked_bytes(ROOT / case["input"], case["sha256"], case.get("gzip", False))
        result = run(case["options"] + ["-f"], data)
        self.assertEqual(result.returncode, 0)
        self.assertEqual(result.stderr, b"rtl_wmbus: monitoring flow\n")
        self.assertEqual(normalize(result.stdout), expected(case))
        # A stalled input ends the program after five seconds.
        with subprocess.Popen([str(BINARY), "-c", "4", "-f"], stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=ENV) as process:
            process.stdin.write(data[:8192])
            process.stdin.flush()
            start = time.monotonic()
            returncode = process.wait(timeout=30)
            elapsed = time.monotonic() - start
            stderr = process.stderr.read()
        self.assertGreaterEqual(elapsed, 4)
        self.assertEqual(returncode, 1)
        self.assertEqual(stderr, b"rtl_wmbus: monitoring flow\n"
                                 b"rtl_wmbus: exiting since incoming data stopped flowing!\n")

    def test_invalid_combinations_are_usage_errors(self):
        for options in (["-c", "4", "-d", "2"], ["-d", "2", "-c", "4"], ["-c", "4", "-p", "T"],
                        ["-p", "T", "-c", "3"], ["-c", "2"], ["-c", "9"], ["-c", "4x"], ["-c", ""]):
            with self.subTest(options=options):
                result = run(options, b"")
                self.assertEqual(result.returncode, 1)
                self.assertTrue(result.stderr.startswith(b"rtl_wmbus: -c "))
                self.assertIn(b"Usage", result.stdout)
        result = run(["-c", "4", "-p", "S"], bytes(8192))
        self.assertEqual((result.returncode, result.stdout, result.stderr), (0, b"", b""))


if __name__ == "__main__":
    unittest.main()
