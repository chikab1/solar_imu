import json
import os
import tempfile
import unittest
from datetime import datetime
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QObject, QSettings, Signal
from PySide6.QtWidgets import QApplication, QPlainTextEdit

from app.mqtt_client import MqttMessage
from app.ui.mqtt_page import MqttPage


class FakeMqttClient(QObject):
    connected = Signal()
    disconnected = Signal()
    error = Signal(str)
    published = Signal(str)
    subscribed = Signal(str)
    unsubscribed = Signal(str)
    message = Signal(object)

    def __init__(self):
        super().__init__()
        self.published_messages = []

    def connect_broker(self, _settings):
        pass

    def disconnect_broker(self):
        pass

    def publish(self, *_args):
        self.published_messages.append(_args)

    def subscribe(self, *_args):
        pass

    def unsubscribe(self, *_args):
        pass


class MqttPageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.settings_file = tempfile.NamedTemporaryFile(suffix=".ini", delete=False)
        self.settings_file.close()
        self.settings = QSettings(self.settings_file.name, QSettings.IniFormat)
        self.client = FakeMqttClient()
        self.page = MqttPage(self.client, self.settings)

    def tearDown(self):
        self.page.deleteLater()
        self.app.processEvents()
        os.unlink(self.settings_file.name)

    def test_sections_and_history_panels(self):
        self.assertFalse(self.page.settings_toggle.isChecked())
        self.assertTrue(self.page.publish_box.findChild(QObject, "sectionToggle").isChecked())
        self.assertTrue(self.page.subscribe_box.findChild(QObject, "sectionToggle").isChecked())
        self.assertEqual(
            [row["topic"].text() for row in self.page.subscribe_rows],
            ["device/+/data", "device/+/ack", ""],
        )
        self.assertEqual(len(self.page.subscribe_rows), 3)
        self.assertTrue(all(row["history"].isReadOnly() for row in self.page.subscribe_rows))
        self.assertTrue(all(isinstance(row["history"], QPlainTextEdit)
                            for row in self.page.subscribe_rows))

    def test_messages_route_to_matching_history_panels(self):
        self.page.add_message(MqttMessage(
            datetime(2026, 8, 13, 12, 0), "device/111/data", b"data", 1, False))
        self.page.add_message(MqttMessage(
            datetime(2026, 8, 13, 12, 1), "device/111/ack", b"ack", 1, True))
        self.assertIn("device/111/data", self.page.subscribe_rows[0]["history"].toPlainText())
        self.assertIn("device/111/ack", self.page.subscribe_rows[1]["history"].toPlainText())
        self.assertEqual(self.page.subscribe_rows[2]["history"].toPlainText(), "")

        self.page.subscribe_rows[2]["topic"].setText("device/+/data")
        self.page.add_message(MqttMessage(
            datetime(2026, 8, 13, 12, 2), "device/222/data", b"multi", 0, False))
        self.assertIn("device/222/data", self.page.subscribe_rows[0]["history"].toPlainText())
        self.assertIn("device/222/data", self.page.subscribe_rows[2]["history"].toPlainText())

    def test_configuration_is_restored_without_using_real_user_settings(self):
        self.page.host.setText("persisted.example")
        self.page.port.setValue(2883)
        self.page.publish_retain.setChecked(True)
        self.page.subscribe_rows[2]["topic"].setText("/device/+/data")
        self.page.save_settings()

        self.page.deleteLater()
        self.app.processEvents()
        restored = MqttPage(FakeMqttClient(), self.settings)
        try:
            self.assertEqual(restored.host.text(), "persisted.example")
            self.assertEqual(restored.port.value(), 2883)
            self.assertTrue(restored.publish_retain.isChecked())
            self.assertEqual(restored.subscribe_rows[2]["topic"].text(), "/device/+/data")
        finally:
            restored.deleteLater()
            self.app.processEvents()

    def test_manual_mode_keeps_existing_payload_when_switching(self):
        self.assertEqual(self.page.publish_mode.currentData(), "manual")
        self.assertIn('"cmd_id":', self.page.publish_payload.toPlainText())
        manual_payload = '{\n  "cmd_id": 456,\n  "ver": 1,\n  "wu": 800\n}'
        self.page.publish_payload.setPlainText(manual_payload)
        self.page.publish_mode.setCurrentIndex(1)
        self.page.publish_mode.setCurrentIndex(0)
        self.assertEqual(self.page.publish_payload.toPlainText(), manual_payload)

    def test_auto_mode_publishes_generated_json(self):
        self.page.publish_mode.setCurrentIndex(1)
        self.page.publish_topic.setText("device/867926053214567/settings")
        self.page.auto_wu.setValue(900)
        self.page.auto_tilt.setValue(35)
        self.page.auto_sleep.setValue(2400)
        self.page.publish_qos.setCurrentIndex(1)
        self.page.publish_retain.setChecked(True)
        with patch("app.ui.mqtt_page.MqttPage._utc_timestamp", return_value=1700000000):
            self.page.publish()

        self.assertEqual(len(self.client.published_messages), 1)
        topic, payload, qos, retain = self.client.published_messages[0]
        document = json.loads(payload.decode("utf-8"))
        self.assertEqual(topic, "device/867926053214567/settings")
        self.assertEqual(qos, 1)
        self.assertTrue(retain)
        self.assertEqual(document["cmd_id"], 1700000000)
        self.assertEqual(document["ver"], 1)
        self.assertEqual(document["wu"], 900)
        self.assertEqual(document["tilt"], 35)
        self.assertEqual(document["sleep"], 2400)
        self.assertEqual(json.loads(self.page.auto_preview.toPlainText()), document)

    def test_auto_timestamp_uses_utc_clock(self):
        with patch("app.ui.mqtt_page.datetime") as mocked_datetime:
            mocked_datetime.now.return_value.timestamp.return_value = 1786593600
            self.assertEqual(self.page._utc_timestamp(), 1786593600)
            mocked_datetime.now.assert_called_once_with(__import__("datetime").timezone.utc)

    def test_refresh_timestamp_updates_auto_preview(self):
        self.page.publish_mode.setCurrentIndex(1)
        with patch("app.ui.mqtt_page.MqttPage._utc_timestamp", return_value=1700000100):
            self.page.refresh_timestamp()
        payload = json.loads(self.page.auto_preview.toPlainText())
        self.assertEqual(payload["cmd_id"], 1700000100)
        self.assertIn("UTC时间戳", self.page.page_status.text())

        self.page.publish_mode.setCurrentIndex(1)
        with patch("app.ui.mqtt_page.MqttPage._utc_timestamp", return_value=1700000000):
            self.page.publish()
            self.page.publish()
        self.assertEqual(len(self.client.published_messages), 1)
        self.assertIn("UTC Unix时间", self.page.page_status.text())

    def test_auto_cmd_id_restores_and_accepts_next_second(self):
        self.page.publish_mode.setCurrentIndex(1)
        with patch("app.ui.mqtt_page.MqttPage._utc_timestamp", return_value=1700000000):
            self.page.publish()
        first = json.loads(self.client.published_messages[-1][1].decode("utf-8"))["cmd_id"]

        self.page.deleteLater()
        self.app.processEvents()
        restored = MqttPage(FakeMqttClient(), self.settings)
        try:
            restored.publish_mode.setCurrentIndex(1)
            with patch("app.ui.mqtt_page.MqttPage._utc_timestamp", return_value=1700000001):
                restored.publish()
            second = json.loads(
                restored.mqtt_client.published_messages[-1][1].decode("utf-8"))["cmd_id"]
            self.assertEqual(first, 1700000000)
            self.assertEqual(second, 1700000001)
        finally:
            restored.deleteLater()
            self.app.processEvents()

    def test_publish_configuration_restores_both_modes(self):
        manual_payload = '{\n  "cmd_id": 789,\n  "ver": 1\n}'
        self.page.publish_payload.setPlainText(manual_payload)
        self.page.publish_mode.setCurrentIndex(1)
        self.page.auto_wu.setValue(1000)
        self.page.auto_tilt.setValue(40)
        self.page.auto_sleep.setValue(3600)
        self.page.save_settings()
        self.page.deleteLater()
        self.app.processEvents()

        restored = MqttPage(FakeMqttClient(), self.settings)
        try:
            self.assertEqual(restored.publish_mode.currentData(), "auto")
            self.assertEqual(restored.publish_payload.toPlainText(), manual_payload)
            self.assertEqual(restored.auto_wu.value(), 1000)
            self.assertEqual(restored.auto_tilt.value(), 40)
            self.assertEqual(restored.auto_sleep.value(), 3600)
        finally:
            restored.deleteLater()
            self.app.processEvents()

    def test_history_is_bounded(self):
        for index in range(MqttPage.MAX_HISTORY + 25):
            self.page.add_message(MqttMessage(
                datetime(2026, 8, 13, 12, 0),
                "device/111/data",
                str(index).encode("ascii"),
                0,
                False,
            ))
        history = self.page.subscribe_rows[0]["messages"]
        self.assertEqual(len(history), MqttPage.MAX_HISTORY)
        self.assertNotIn("\n0", self.page.subscribe_rows[0]["history"].toPlainText())
        self.assertIn("324", self.page.subscribe_rows[0]["history"].toPlainText())


if __name__ == "__main__":
    unittest.main()
