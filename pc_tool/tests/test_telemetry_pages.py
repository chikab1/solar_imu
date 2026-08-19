import os
import struct
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QApplication

from app.protocol.commands import Command, Status
from app.ui.pages import LiveAttitudePage, SixAxisPage


class FakeDeviceClient(QObject):
    response = Signal(int, int, object)
    disconnected = Signal()

    def request(self, _command):
        return True


class TelemetryPageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.client = FakeDeviceClient()
        self.live_page = LiveAttitudePage(self.client)
        self.six_axis_page = SixAxisPage(self.client)

    def tearDown(self):
        self.live_page.deleteLater()
        self.six_axis_page.deleteLater()
        self.app.processEvents()

    def test_live_response_updates_attitude(self):
        payload = struct.pack("<8h", 1, 2, 3, 4, 5, 6, 123, -456)
        self.client.response.emit(Command.GET_IMU_LIVE, Status.OK, payload)
        self.assertEqual(self.live_page.pitch_card.value_label.text(), "1.23")
        self.assertEqual(self.live_page.roll_card.value_label.text(), "-4.56")

    def test_six_axis_response_updates_total_and_history(self):
        payload = struct.pack("<8h", 3, 4, 12, 1, -2, 3, 123, -456)
        self.client.response.emit(Command.GET_IMU_LIVE, Status.OK, payload)

        self.assertEqual(self.six_axis_page.acc_values["X"].text(), "3 mg")
        self.assertEqual(self.six_axis_page.total_acc_value.text(), "13.0 mg")
        self.assertEqual(self.six_axis_page.gyro_values[1].text(), "-2 °/s")
        self.assertEqual(self.six_axis_page.angle_values[0].text(), "1.23 °")
        self.assertEqual(len(self.six_axis_page.acc_chart.series["X"][0]), 1)
        self.assertEqual(
            self.six_axis_page.acc_chart.series["总加速度"][0][-1], 13.0)
        self.assertEqual(len(self.six_axis_page.gyro_chart.series["Z"][0]), 1)
        self.assertEqual(len(self.six_axis_page.angle_chart.series["Roll"][0]), 1)

    def test_six_axis_response_records_complete_sample(self):
        payload = struct.pack("<8h", 3, 4, 12, 1, -2, 3, 123, -456)
        with tempfile.TemporaryDirectory() as directory:
            path = self.six_axis_page.recording.recorder.start(directory)
            self.client.response.emit(Command.GET_IMU_LIVE, Status.OK, payload)
            self.six_axis_page.stop_recording()
            text = Path(path).read_text(encoding="utf-8-sig")
        self.assertIn("acc_total_mg", text)
        self.assertIn(",3,4,12,13.0,1,-2,3,1.23,-4.56", text)

    def test_clear_history_clears_all_charts(self):
        payload = struct.pack("<8h", 3, 4, 12, 1, -2, 3, 123, -456)
        self.client.response.emit(Command.GET_IMU_LIVE, Status.OK, payload)
        self.six_axis_page.clear_history()
        self.assertEqual(len(self.six_axis_page.acc_chart.series["X"][0]), 0)
        self.assertEqual(len(self.six_axis_page.gyro_chart.series["X"][0]), 0)
        self.assertEqual(len(self.six_axis_page.angle_chart.series["Pitch"][0]), 0)


if __name__ == "__main__":
    unittest.main()
