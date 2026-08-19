import csv
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

from app.telemetry_recorder import TelemetryRecorder


class TelemetryRecorderTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp_dir.name)

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_daily_files_have_header_and_append_rows(self):
        fields = ("timestamp", "pitch_deg", "roll_deg")
        first = datetime(2026, 8, 18, 12, 0, 0)
        recorder = TelemetryRecorder("attitude", fields)
        recorder.start(self.directory, first)
        recorder.write({"pitch_deg": "1.25", "roll_deg": "-2.50"}, first)
        recorder.stop()

        again = TelemetryRecorder("attitude", fields)
        again.start(self.directory, datetime(2026, 8, 18, 13, 0, 0))
        again.write({"pitch_deg": "3.00", "roll_deg": "4.00"},
                    datetime(2026, 8, 18, 13, 0, 1))
        again.stop()

        path = self.directory / "attitude_20260818.csv"
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["pitch_deg"], "1.25")
        self.assertEqual(rows[1]["roll_deg"], "4.00")
        self.assertEqual(path.read_bytes().count(b"timestamp"), 1)

    def test_date_change_switches_file(self):
        fields = ("timestamp", "acc_total_mg")
        recorder = TelemetryRecorder("six_axis", fields)
        recorder.start(self.directory, datetime(2026, 8, 18, 23, 59, 59))
        first_path = recorder.write({"acc_total_mg": "1000.0"},
                                    datetime(2026, 8, 18, 23, 59, 59))
        second_path = recorder.write({"acc_total_mg": "1001.0"},
                                     datetime(2026, 8, 19, 0, 0, 0))
        recorder.stop()

        self.assertEqual(first_path.name, "six_axis_20260818.csv")
        self.assertEqual(second_path.name, "six_axis_20260819.csv")
        self.assertIn("1000.0", first_path.read_text(encoding="utf-8-sig"))
        self.assertIn("1001.0", second_path.read_text(encoding="utf-8-sig"))

    def test_start_rejects_missing_directory(self):
        recorder = TelemetryRecorder("attitude", ("timestamp", "pitch_deg"))
        with self.assertRaises(ValueError):
            recorder.start(self.directory / "missing")


if __name__ == "__main__":
    unittest.main()
