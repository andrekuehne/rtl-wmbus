import gzip
import hashlib
from pathlib import Path
import tempfile
import unittest

from replay_captures import checked_bytes, compare, normalize


class ReplayChecks(unittest.TestCase):
    packet = b"C1;1;1;2026-10-13 08:00:00.123456;98;95;92966281;0x3144\n"

    def test_only_timestamp_is_ignored(self):
        changed = self.packet.replace(b"08:00:00.123456", b"09:01:02.654321")
        self.assertEqual(normalize(changed), normalize(self.packet))

    def test_packet_fields_are_significant(self):
        fields = self.packet.split(b";")
        for index in (0, 1, 2, 4, 5, 6, 7):
            with self.subTest(field=index):
                changed = list(fields)
                changed[index] = b"T1" if index == 0 else b"9" + changed[index]
                with self.assertRaises(AssertionError):
                    compare(b";".join(changed), b"", 0, normalize(self.packet))

    def test_packet_order_and_duplicates_are_significant(self):
        other = self.packet.replace(b";98;95;", b";99;94;")
        expected = normalize(self.packet + other)
        for changed in (other + self.packet, self.packet, self.packet + other + other):
            with self.subTest(output=changed):
                with self.assertRaises(AssertionError):
                    compare(changed, b"", 0, expected)

    def test_exit_stderr_and_line_endings_are_significant(self):
        expected = normalize(self.packet)
        for stdout, stderr, code in (
            (self.packet, b"error", 0), (self.packet, b"", 1),
            (self.packet.rstrip(b"\n"), b"", 0),
            (self.packet.replace(b"\n", b"\r\n"), b"", 0),
        ):
            with self.subTest(stdout=stdout, stderr=stderr, code=code):
                with self.assertRaises(AssertionError):
                    compare(stdout, stderr, code, expected)

    def test_malformed_output_is_rejected(self):
        for stdout in (b"unexpected\n", self.packet.replace(b".123456", b".123")):
            with self.assertRaises(ValueError):
                normalize(stdout)

    def test_empty_output_cannot_replace_packets(self):
        with self.assertRaises(AssertionError):
            compare(b"", b"", 0, normalize(self.packet))

    def test_fixture_checksum_and_compression(self):
        data = b"recorded I/Q bytes"
        digest = hashlib.sha256(data).hexdigest()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fixture.gz"
            path.write_bytes(gzip.compress(data, mtime=0))
            self.assertEqual(checked_bytes(path, digest, compressed=True), data)
            with self.assertRaises(ValueError):
                checked_bytes(path, "0" * 64, compressed=True)


if __name__ == "__main__":
    unittest.main()
