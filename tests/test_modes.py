"""Keep enabled-mode output and input block semantics when -p skips work."""

import json
import os
from pathlib import Path
import subprocess
import unittest

from replay_captures import checked_bytes, normalize


ROOT = Path(__file__).resolve().parents[1]
BINARY = Path(os.environ.get("RTL_WMBUS_TEST_BINARY", ROOT / "build/rtl_wmbus")).resolve()
CASES = json.loads((ROOT / "tests/fixtures/manifest.json").read_text())["cases"]
ENV = dict(os.environ, LC_ALL="C", TZ="UTC")


def fixture(case):
    data = checked_bytes(ROOT / case["input"], case["sha256"], case.get("gzip", False))
    expected = checked_bytes(ROOT / case["expected"], case["expected_sha256"])
    return data, expected


def enabled_rows(expected, disabled):
    modes = {b"S1"} if disabled == "S" else {b"T1", b"C1"}
    return b"".join(row for row in expected.splitlines(keepends=True)
                    if row.split(b";", 1)[0] not in modes)


class ModeChecks(unittest.TestCase):
    def check_result(self, result, expected, stderr=b""):
        self.assertEqual(result.returncode, 0)
        self.assertEqual(result.stderr, stderr)
        self.assertEqual(normalize(result.stdout), expected)

    def test_disabled_modes_keep_complete_enabled_output(self):
        for case in CASES:
            data, expected = fixture(case)
            for disabled in ("S", "T"):
                with self.subTest(case=case["name"], disabled=disabled):
                    result = subprocess.run(
                        [str(BINARY)] + case["options"] + ["-p", disabled],
                        input=data, capture_output=True, env=ENV, timeout=60)
                    self.check_result(result, enabled_rows(expected, disabled))
            for flags in (["-p", "S", "-p", "T"], ["-p", "t", "-p", "s"]):
                with self.subTest(case=case["name"], flags=flags):
                    result = subprocess.run([str(BINARY)] + case["options"] + flags,
                                            input=data, capture_output=True, env=ENV, timeout=60)
                    self.check_result(result, b"")

    def test_fragmented_pipe_and_partial_final_block(self):
        # d=3 crosses each 2048-complex-sample block at a different phase.
        case = next(case for case in CASES if case["name"] == "tracked-issue48")
        data, expected = fixture(case)
        self.assertEqual(len(data) % 4096, 0)
        for flags, wanted in (([], expected), (["-p", "S"], enabled_rows(expected, "S")),
                              (["-p", "T"], enabled_rows(expected, "T")),
                              (["-p", "S", "-p", "T"], b"")):
            with self.subTest(flags=flags):
                with subprocess.Popen([str(BINARY)] + case["options"] + flags,
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE, env=ENV) as process:
                    stream = data + bytes([0, 255]) * 2047 + b"\xff"
                    for offset in range(0, len(stream), 997):
                        process.stdin.write(stream[offset:offset + 997])
                    process.stdin.close()
                    process.stdin = None
                    stdout, stderr = process.communicate(timeout=60)
                    self.check_result(subprocess.CompletedProcess(process.args, process.returncode,
                                                                  stdout, stderr), wanted)

    def test_dispatch_options_preserve_fragmented_stream(self):
        # Exercise both discriminator choices and each decoder algorithm across
        # block boundaries without restarting their persistent histories.
        for case in CASES:
            data, _ = fixture(case)
            for discriminator in ([], ["-a"]):
                for algorithm in ([], ["-r", "0"], ["-t", "0"]):
                    options = case["options"] + discriminator + algorithm
                    with self.subTest(case=case["name"], options=options):
                        reference = subprocess.run([str(BINARY)] + options, input=data,
                                                   capture_output=True, env=ENV, timeout=60)
                        self.assertEqual(reference.returncode, 0)
                        self.assertEqual(reference.stderr, b"")
                        with subprocess.Popen([str(BINARY)] + options,
                                              stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                              stderr=subprocess.PIPE, env=ENV) as process:
                            # fread must keep waiting for a complete block;
                            # the odd partial final block remains unprocessed.
                            stream = data + bytes([0, 255]) * 2047 + b"\xff"
                            for offset in range(0, len(stream), 997):
                                process.stdin.write(stream[offset:offset + 997])
                            process.stdin.close()
                            process.stdin = None
                            stdout, stderr = process.communicate(timeout=60)
                            self.check_result(subprocess.CompletedProcess(
                                process.args, process.returncode, stdout, stderr),
                                normalize(reference.stdout))

    @unittest.skipUnless(os.name == "posix", "POSIX flow monitoring")
    def test_flow_monitor_still_reads_disabled_mode_stream(self):
        case = next(case for case in CASES if case["name"] == "tracked-issue48")
        data, expected = fixture(case)
        for flags, wanted in ((["-p", "S"], enabled_rows(expected, "S")),
                              (["-p", "T"], enabled_rows(expected, "T")),
                              (["-p", "S", "-p", "T"], b"")):
            with self.subTest(flags=flags):
                result = subprocess.run([str(BINARY)] + case["options"] + flags + ["-f"],
                                        input=data, capture_output=True, env=ENV, timeout=60)
                self.check_result(result, wanted, b"rtl_wmbus: monitoring flow\n")


if __name__ == "__main__":
    unittest.main()
