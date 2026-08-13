from dataclasses import dataclass
from datetime import datetime
from queue import Empty, Queue
import threading
import time

from PySide6.QtCore import QObject, QThread, QTimer, Qt, Signal, Slot


@dataclass(frozen=True)
class MqttMessage:
    received_at: datetime
    topic: str
    payload: bytes
    qos: int
    retain: bool


@dataclass(frozen=True)
class MqttSettings:
    host: str
    port: int
    username: str = ""
    password: str = ""
    client_id: str = ""
    keepalive: int = 60


class MqttWorker(QThread):
    connected = Signal()
    disconnected = Signal()
    error = Signal(str)
    published = Signal(str)
    subscribed = Signal(str)
    unsubscribed = Signal(str)
    message = Signal(object)

    def __init__(self):
        super().__init__()
        self.commands = Queue()
        self.stop_event = threading.Event()
        self.client = None
        self.settings = None
        self.user_disconnect = False

    def connect_broker(self, settings: MqttSettings):
        self.commands.put(("connect", settings))

    def disconnect_broker(self):
        self.commands.put(("disconnect", None))

    def publish_message(self, topic: str, payload: bytes, qos: int, retain: bool):
        self.commands.put(("publish", (topic, payload, qos, retain)))

    def subscribe_topic(self, topic: str, qos: int):
        self.commands.put(("subscribe", (topic, qos)))

    def unsubscribe_topic(self, topic: str):
        self.commands.put(("unsubscribe", topic))

    def shutdown(self):
        self.stop_event.set()
        self.commands.put(("stop", None))

    def run(self):
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            self.error.emit("缺少 paho-mqtt，请在 solarimu 环境安装 requirements.txt")
            self.disconnected.emit()
            return

        try:
            while not self.stop_event.is_set():
                self._process_commands(mqtt)
                if self.client is not None:
                    result = self.client.loop(timeout=0.1)
                    if result != mqtt.MQTT_ERR_SUCCESS and not self.user_disconnect:
                        self.error.emit(f"MQTT网络循环失败：{mqtt.error_string(result)}")
                        self._close_client()
                        self.disconnected.emit()
                else:
                    try:
                        command, value = self.commands.get(timeout=0.1)
                    except Empty:
                        continue
                    self._execute(command, value, mqtt)
        except Exception as exc:
            self.error.emit(self._safe_error(exc))
        finally:
            self._close_client()
            self.disconnected.emit()

    def _process_commands(self, mqtt):
        while True:
            try:
                command, value = self.commands.get_nowait()
            except Empty:
                return
            self._execute(command, value, mqtt)
            if self.stop_event.is_set():
                return

    def _execute(self, command, value, mqtt):
        if command == "stop":
            self._close_client()
            return
        if command == "connect":
            self._connect(value, mqtt)
        elif command == "disconnect":
            self.user_disconnect = True
            self._close_client()
            self.disconnected.emit()
        elif self.client is None:
            self.error.emit("MQTT尚未连接")
        elif command == "publish":
            topic, payload, qos, retain = value
            info = self.client.publish(topic, payload, qos=qos, retain=retain)
            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                self.error.emit(f"MQTT发布失败：{mqtt.error_string(info.rc)}")
            else:
                self.published.emit(topic)
        elif command == "subscribe":
            topic, qos = value
            result, _ = self.client.subscribe(topic, qos=qos)
            if result != mqtt.MQTT_ERR_SUCCESS:
                self.error.emit(f"MQTT订阅失败：{mqtt.error_string(result)}")
            else:
                self.subscribed.emit(topic)
        elif command == "unsubscribe":
            result, _ = self.client.unsubscribe(value)
            if result != mqtt.MQTT_ERR_SUCCESS:
                self.error.emit(f"MQTT退订失败：{mqtt.error_string(result)}")
            else:
                self.unsubscribed.emit(value)

    def _connect(self, settings: MqttSettings, mqtt):
        self._close_client()
        if not settings.host:
            self.error.emit("Broker地址不能为空")
            self.disconnected.emit()
            return
        self.user_disconnect = False
        client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=settings.client_id,
            protocol=mqtt.MQTTv311,
        )
        if settings.username:
            client.username_pw_set(settings.username, settings.password)
        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message
        try:
            result = client.connect(settings.host, settings.port, settings.keepalive)
        except Exception as exc:
            self.error.emit(f"MQTT连接失败：{self._safe_error(exc)}")
            return
        if result != mqtt.MQTT_ERR_SUCCESS:
            self.error.emit(f"MQTT连接失败：{mqtt.error_string(result)}")
            return
        self.client = client
        self.settings = settings

    def _close_client(self):
        client = self.client
        self.client = None
        if client is not None:
            try:
                client.disconnect()
            except Exception:
                pass

    def _on_connect(self, _client, _userdata, _flags, reason_code, _properties):
        if self._reason_failed(reason_code):
            self.error.emit(f"MQTT认证/连接被拒绝：{reason_code}")
        else:
            self.connected.emit()

    def _on_disconnect(self, _client, _userdata, _disconnect_flags, reason_code, _properties):
        if not self.user_disconnect and self._reason_failed(reason_code):
            self.error.emit(f"MQTT连接断开：{reason_code}")

    @staticmethod
    def _reason_failed(reason_code):
        failure = getattr(reason_code, "is_failure", None)
        if failure is not None:
            return bool(failure)
        try:
            return int(reason_code) != 0
        except (TypeError, ValueError):
            return str(reason_code).lower() not in {
                "success", "normal disconnection", "normal disconnection."
            }

    def _on_message(self, _client, _userdata, msg):
        self.message.emit(MqttMessage(
            received_at=datetime.now(),
            topic=str(msg.topic),
            payload=bytes(msg.payload),
            qos=int(msg.qos),
            retain=bool(msg.retain),
        ))

    @staticmethod
    def _safe_error(exc):
        return str(exc).replace("password", "******")


class MqttClient(QObject):
    connected = Signal()
    disconnected = Signal()
    error = Signal(str)
    published = Signal(str)
    subscribed = Signal(str)
    unsubscribed = Signal(str)
    message = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = MqttWorker()
        self.worker.connected.connect(self._on_connected, Qt.DirectConnection)
        self.worker.disconnected.connect(self._on_disconnected, Qt.DirectConnection)
        self.worker.error.connect(self._on_error, Qt.DirectConnection)
        self.worker.published.connect(self._on_published, Qt.DirectConnection)
        self.worker.subscribed.connect(self._on_subscribed, Qt.DirectConnection)
        self.worker.unsubscribed.connect(self._on_unsubscribed, Qt.DirectConnection)
        self.worker.message.connect(self._on_message, Qt.DirectConnection)
        self.worker.start()

    @property
    def is_connected(self):
        return self.worker.client is not None

    def connect_broker(self, settings: MqttSettings):
        self.worker.connect_broker(settings)

    def disconnect_broker(self):
        self.worker.disconnect_broker()

    def publish(self, topic: str, payload: bytes, qos: int, retain: bool):
        self.worker.publish_message(topic, payload, qos, retain)

    def subscribe(self, topic: str, qos: int):
        self.worker.subscribe_topic(topic, qos)

    def unsubscribe(self, topic: str):
        self.worker.unsubscribe_topic(topic)

    def close(self):
        if self.worker.isRunning():
            self.worker.shutdown()
            self.worker.wait(2500)

    def _relay(self, signal, *args):
        QTimer.singleShot(0, self, lambda: signal.emit(*args))

    @Slot()
    def _on_connected(self):
        self._relay(self.connected)

    @Slot()
    def _on_disconnected(self):
        self._relay(self.disconnected)

    @Slot(str)
    def _on_error(self, message):
        self._relay(self.error, message)

    @Slot(str)
    def _on_published(self, topic):
        self._relay(self.published, topic)

    @Slot(str)
    def _on_subscribed(self, topic):
        self._relay(self.subscribed, topic)

    @Slot(str)
    def _on_unsubscribed(self, topic):
        self._relay(self.unsubscribed, topic)

    @Slot(object)
    def _on_message(self, value):
        self._relay(self.message, value)
