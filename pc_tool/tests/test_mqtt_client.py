import unittest
from datetime import datetime

from app.mqtt_client import MqttMessage, MqttSettings


class MqttModelTests(unittest.TestCase):
    def test_settings_keep_broker_values(self):
        settings = MqttSettings("broker.local", 1883, "user", "secret", "client-1")
        self.assertEqual(settings.host, "broker.local")
        self.assertEqual(settings.port, 1883)
        self.assertEqual(settings.username, "user")
        self.assertEqual(settings.password, "secret")
        self.assertEqual(settings.client_id, "client-1")

    def test_message_preserves_wildcard_source_topic_and_payload(self):
        message = MqttMessage(datetime(2026, 8, 13, 12, 0), "/device/+/data", b"\xff", 1, True)
        self.assertEqual(message.topic, "/device/+/data")
        self.assertEqual(message.payload, b"\xff")
        self.assertEqual(message.qos, 1)
        self.assertTrue(message.retain)


if __name__ == "__main__":
    unittest.main()
