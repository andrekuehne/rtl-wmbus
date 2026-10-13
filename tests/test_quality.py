import unittest

from check_quality import compiler_diagnostics, static_diagnostics, unexpected


class DiagnosticChecks(unittest.TestCase):
    def test_known_warning_does_not_allow_new_or_additional_warning(self):
        known = {"file": "rtl_wmbus.c", "source": "old code", "id": "old", "message": "old warning"}
        new = dict(known, message="new warning")
        self.assertEqual(unexpected([known], [known]), [])
        self.assertEqual(unexpected([known, new], [known]), [new])
        self.assertEqual(unexpected([known, known], [known]), [known])

    def test_unrecognized_compiler_warning_fails_closed(self):
        with self.assertRaises(ValueError):
            compiler_diagnostics("clang: warning: unknown command-line warning")

    def test_invalid_analyzer_output_fails_closed(self):
        with self.assertRaises(ValueError):
            static_diagnostics(b"<not-results />")


if __name__ == "__main__":
    unittest.main()
