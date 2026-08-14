from collections import deque
from datetime import datetime, timezone
import json

from PySide6.QtCore import QSettings, Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel,
    QLineEdit, QMessageBox, QPlainTextEdit, QPushButton, QScrollArea,
    QSpinBox, QVBoxLayout, QWidget,
)

from app.mqtt_client import MqttMessage, MqttSettings


class MqttPage(QWidget):
    MAX_HISTORY = 300
    DEFAULT_TOPICS = ("device/+/data", "device/+/ack", "")

    def __init__(self, mqtt_client, settings=None):
        super().__init__()
        self.mqtt_client = mqtt_client
        self.settings = settings or QSettings("SolarIMU", "PC Tool")
        self._restoring = True
        self.subscribe_rows = []
        self._build_ui()
        self._restore_settings()
        self._connect_signals()
        self._restoring = False

    def _build_ui(self):
        outer = QVBoxLayout(self)
        title_row = QHBoxLayout()
        title = QLabel("MQTT远程消息")
        title.setObjectName("pageTitle")
        title_row.addWidget(title)
        title_row.addStretch()
        self.page_status = QLabel("配置会自动保存")
        self.page_status.setObjectName("hintLabel")
        title_row.addWidget(self.page_status)
        outer.addLayout(title_row)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(2, 2, 8, 2)
        self._build_broker_section(layout)
        self._build_publish_section(layout)
        self._build_subscribe_section(layout)
        layout.addStretch()
        scroll.setWidget(content)
        outer.addWidget(scroll, 1)

    def _build_broker_section(self, parent_layout):
        self.broker_box, broker_body = self._collapsible("Broker设置", False)
        parent_layout.addWidget(self.broker_box)
        header = self.broker_box.layout().itemAt(0).widget()
        self.settings_toggle = header.findChild(QPushButton, "sectionToggle")
        self.connect_button = QPushButton("连接Broker")
        self.connect_button.setObjectName("primaryButton")
        self.disconnect_button = QPushButton("断开")
        self.disconnect_button.setEnabled(False)
        self.status = QLabel("● 未连接")
        self.status.setObjectName("connectionOff")
        header.layout().addWidget(self.connect_button)
        header.layout().addWidget(self.disconnect_button)
        header.layout().addWidget(self.status)
        header.layout().addStretch()

        form = QFormLayout(broker_body)
        self.host = QLineEdit()
        self.port = QSpinBox()
        self.port.setRange(1, 65535)
        self.username = QLineEdit()
        self.password = QLineEdit()
        self.password.setEchoMode(QLineEdit.Password)
        self.client_id = QLineEdit()
        self.client_id.setPlaceholderText("留空由Broker分配")
        form.addRow("Broker地址", self.host)
        form.addRow("端口", self.port)
        form.addRow("用户名", self.username)
        form.addRow("密码", self.password)
        form.addRow("Client ID", self.client_id)

    def _build_publish_section(self, parent_layout):
        self.publish_box, publish_body = self._collapsible("发布消息", True)
        parent_layout.addWidget(self.publish_box)
        form = QFormLayout(publish_body)
        self.publish_mode = QComboBox()
        self.publish_mode.addItem("手动文本", "manual")
        self.publish_mode.addItem("自动填充", "auto")
        self.publish_topic = QLineEdit()
        self.publish_payload = QPlainTextEdit()
        self.publish_payload.setFont(QFont("Consolas", 9))
        self.publish_payload.setMinimumHeight(110)
        self.auto_config = QWidget()
        auto_form = QFormLayout(self.auto_config)
        self.auto_wu = self._threshold_spin(250, 2000, 750, " mg")
        self.auto_tilt = self._threshold_spin(10, 90, 30, " °")
        self.auto_sleep = self._threshold_spin(600, 65535, 1800, " 秒")
        self.auto_preview = QPlainTextEdit()
        self.auto_preview.setReadOnly(True)
        self.auto_preview.setFont(QFont("Consolas", 9))
        self.auto_preview.setMinimumHeight(110)
        preview_label = QWidget()
        preview_label_layout = QHBoxLayout(preview_label)
        preview_label_layout.setContentsMargins(0, 0, 0, 0)
        preview_label_layout.addWidget(QLabel("自动生成预览"))
        self.refresh_timestamp_button = QPushButton("刷新时间戳")
        preview_label_layout.addWidget(self.refresh_timestamp_button)
        preview_label_layout.addStretch()
        auto_form.addRow("加速度阈值", self.auto_wu)
        auto_form.addRow("倾角阈值", self.auto_tilt)
        auto_form.addRow("唤醒/休眠时间", self.auto_sleep)
        auto_form.addRow(preview_label, self.auto_preview)
        self.manual_payload_label = QLabel("手动Payload")
        self.auto_payload_label = QLabel("自动配置")
        self.publish_qos = self._qos_combo()
        self.publish_retain = QCheckBox("Retain")
        self.publish_button = QPushButton("发布")
        self.publish_button.setObjectName("primaryButton")
        options = QHBoxLayout()
        options.addWidget(QLabel("QoS"))
        options.addWidget(self.publish_qos)
        options.addWidget(self.publish_retain)
        options.addWidget(self.publish_button)
        options.addStretch()
        form.addRow("模式", self.publish_mode)
        form.addRow("Topic", self.publish_topic)
        form.addRow(self.manual_payload_label, self.publish_payload)
        form.addRow(self.auto_payload_label, self.auto_config)
        form.addRow("选项", options)
        self._update_publish_mode()

    @staticmethod
    def _threshold_spin(minimum, maximum, value, suffix):
        spin = QSpinBox()
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setSuffix(suffix)
        return spin

    def _build_subscribe_section(self, parent_layout):
        self.subscribe_box, subscribe_body = self._collapsible("订阅消息", True)
        parent_layout.addWidget(self.subscribe_box)
        body_layout = QVBoxLayout(subscribe_body)
        hint = QLabel("每个窗口独立接收并保留历史消息；第三个 Topic 可自定义，支持 + 和 # 通配符。")
        hint.setObjectName("hintLabel")
        body_layout.addWidget(hint)
        for index, default_topic in enumerate(self.DEFAULT_TOPICS):
            row = self._build_subscribe_panel(index, default_topic)
            self.subscribe_rows.append(row)
            body_layout.addWidget(row["box"])

    def _build_subscribe_panel(self, index, default_topic):
        box = QGroupBox(f"订阅窗口 {index + 1}")
        layout = QVBoxLayout(box)
        controls = QHBoxLayout()
        topic = QLineEdit()
        topic.setPlaceholderText("留空表示未配置")
        qos = self._qos_combo()
        subscribe_button = QPushButton("订阅")
        unsubscribe_button = QPushButton("退订")
        clear_button = QPushButton("清空历史")
        controls.addWidget(QLabel("Topic"))
        controls.addWidget(topic, 1)
        controls.addWidget(QLabel("QoS"))
        controls.addWidget(qos)
        controls.addWidget(subscribe_button)
        controls.addWidget(unsubscribe_button)
        controls.addWidget(clear_button)
        layout.addLayout(controls)
        history = QPlainTextEdit()
        history.setReadOnly(True)
        history.setFont(QFont("Consolas", 9))
        history.setPlaceholderText("等待消息…")
        history.setMinimumHeight(120)
        layout.addWidget(history)
        return {
            "box": box,
            "topic": topic,
            "qos": qos,
            "subscribe": subscribe_button,
            "unsubscribe": unsubscribe_button,
            "clear": clear_button,
            "history": history,
            "messages": deque(maxlen=self.MAX_HISTORY),
        }

    @staticmethod
    def _collapsible(title, expanded):
        box = QGroupBox()
        box_layout = QVBoxLayout(box)
        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(0, 0, 0, 0)
        toggle = QPushButton()
        toggle.setObjectName("sectionToggle")
        toggle.setCheckable(True)
        toggle.setChecked(expanded)
        toggle.setText(f"▼ {title}" if expanded else f"▶ {title}")
        header_layout.addWidget(toggle)
        box_layout.addWidget(header)
        body = QWidget()
        body.setVisible(expanded)
        box_layout.addWidget(body)
        toggle.toggled.connect(body.setVisible)
        toggle.toggled.connect(
            lambda checked, button=toggle, name=title: button.setText(
                f"▼ {name}" if checked else f"▶ {name}"))
        return box, body

    @staticmethod
    def _qos_combo():
        combo = QComboBox()
        for qos in (0, 1, 2):
            combo.addItem(str(qos), qos)
        combo.setCurrentIndex(1)
        return combo

    def _connect_signals(self):
        self.connect_button.clicked.connect(self.connect_broker)
        self.disconnect_button.clicked.connect(self.mqtt_client.disconnect_broker)
        self.publish_button.clicked.connect(self.publish)
        self.publish_mode.currentIndexChanged.connect(self._publish_mode_changed)
        self.refresh_timestamp_button.clicked.connect(self.refresh_timestamp)
        self.auto_wu.valueChanged.connect(self._auto_values_changed)
        self.auto_tilt.valueChanged.connect(self._auto_values_changed)
        self.auto_sleep.valueChanged.connect(self._auto_values_changed)
        self.mqtt_client.connected.connect(self.on_connected)
        self.mqtt_client.disconnected.connect(self.on_disconnected)
        self.mqtt_client.error.connect(self.on_error)
        self.mqtt_client.published.connect(lambda topic: self._status(f"已发布：{topic}"))
        self.mqtt_client.subscribed.connect(lambda topic: self._status(f"已订阅：{topic}"))
        self.mqtt_client.unsubscribed.connect(lambda topic: self._status(f"已退订：{topic}"))
        self.mqtt_client.message.connect(self.add_message)

        fields = [self.host, self.username, self.password, self.client_id,
                  self.publish_topic, self.publish_payload]
        for field in fields:
            signal = field.textChanged if isinstance(field, QLineEdit) else field.textChanged
            signal.connect(self.save_settings)
        self.port.valueChanged.connect(self.save_settings)
        self.publish_qos.currentIndexChanged.connect(self.save_settings)
        self.publish_retain.toggled.connect(self.save_settings)
        self.auto_wu.valueChanged.connect(self.save_settings)
        self.auto_tilt.valueChanged.connect(self.save_settings)
        self.auto_sleep.valueChanged.connect(self.save_settings)
        for row in self.subscribe_rows:
            row["subscribe"].clicked.connect(
                lambda _checked=False, item=row: self.subscribe_topic(item))
            row["unsubscribe"].clicked.connect(
                lambda _checked=False, item=row: self.unsubscribe_topic(item))
            row["clear"].clicked.connect(
                lambda _checked=False, item=row: self.clear_history(item))
            row["topic"].textChanged.connect(self.save_settings)
            row["qos"].currentIndexChanged.connect(self.save_settings)

    @staticmethod
    def _utc_timestamp():
        return int(datetime.now(timezone.utc).timestamp())

    @staticmethod
    def _default_manual_payload():
        return json.dumps({
            "cmd_id": MqttPage._utc_timestamp(),
            "ver": 1,
            "wu": 750,
            "tilt": 30,
            "sleep": 3600,
        }, ensure_ascii=False, indent=2)

    def _restore_settings(self):
        self.host.setText(self.settings.value("mqtt/host", "101.34.217.153"))
        self.port.setValue(int(self.settings.value("mqtt/port", 1883)))
        self.username.setText(self.settings.value("mqtt/username", "solar_imu"))
        self.password.setText(self.settings.value("mqtt/password", "solar_imu"))
        self.client_id.setText(self.settings.value("mqtt/client_id", ""))
        self.publish_topic.setText(
            self.settings.value("mqtt/publish/topic", "device/867926053214567/settings"))
        self.publish_payload.setPlainText(self.settings.value(
            "mqtt/publish/payload", self._default_manual_payload()))
        self.publish_qos.setCurrentIndex(int(self.settings.value("mqtt/publish/qos", 1)))
        self.publish_retain.setChecked(self._setting_bool("mqtt/publish/retain", False))
        self.publish_mode.setCurrentIndex(
            max(0, self.publish_mode.findData(
                self.settings.value("mqtt/publish/mode", "manual"))))
        self.auto_wu.setValue(int(self.settings.value("mqtt/publish/auto/wu", 750)))
        self.auto_tilt.setValue(int(self.settings.value("mqtt/publish/auto/tilt", 30)))
        self.auto_sleep.setValue(int(self.settings.value("mqtt/publish/auto/sleep", 1800)))
        self._update_publish_mode()
        for index, row in enumerate(self.subscribe_rows):
            prefix = f"mqtt/subscribe/{index}"
            row["topic"].setText(self.settings.value(f"{prefix}/topic", self.DEFAULT_TOPICS[index]))
            row["qos"].setCurrentIndex(int(self.settings.value(f"{prefix}/qos", 1)))

    def save_settings(self):
        if self._restoring:
            return
        self.settings.setValue("mqtt/host", self.host.text())
        self.settings.setValue("mqtt/port", self.port.value())
        self.settings.setValue("mqtt/username", self.username.text())
        self.settings.setValue("mqtt/password", self.password.text())
        self.settings.setValue("mqtt/client_id", self.client_id.text())
        self.settings.setValue("mqtt/publish/topic", self.publish_topic.text())
        self.settings.setValue("mqtt/publish/payload", self.publish_payload.toPlainText())
        self.settings.setValue("mqtt/publish/mode", self.publish_mode.currentData())
        self.settings.setValue("mqtt/publish/auto/wu", self.auto_wu.value())
        self.settings.setValue("mqtt/publish/auto/tilt", self.auto_tilt.value())
        self.settings.setValue("mqtt/publish/auto/sleep", self.auto_sleep.value())
        self.settings.setValue("mqtt/publish/qos", self.publish_qos.currentData())
        self.settings.setValue("mqtt/publish/retain", self.publish_retain.isChecked())
        for index, row in enumerate(self.subscribe_rows):
            prefix = f"mqtt/subscribe/{index}"
            self.settings.setValue(f"{prefix}/topic", row["topic"].text())
            self.settings.setValue(f"{prefix}/qos", row["qos"].currentData())
        self.settings.sync()
        self.page_status.setText("配置已保存")

    def connect_broker(self):
        host = self.host.text().strip()
        if not host:
            QMessageBox.warning(self, "连接失败", "Broker地址不能为空。")
            return
        self.save_settings()
        self.connect_button.setEnabled(False)
        self.status.setText("● 正在连接…")
        self.mqtt_client.connect_broker(MqttSettings(
            host=host,
            port=self.port.value(),
            username=self.username.text(),
            password=self.password.text(),
            client_id=self.client_id.text().strip(),
        ))

    def _publish_mode_changed(self):
        self._update_publish_mode()
        self.save_settings()

    def _auto_values_changed(self):
        self._update_auto_preview()

    def refresh_timestamp(self):
        self._update_auto_preview()
        self._status("已刷新UTC时间戳")

    def _update_publish_mode(self):
        automatic = self.publish_mode.currentData() == "auto"
        self.manual_payload_label.setVisible(not automatic)
        self.publish_payload.setVisible(not automatic)
        self.auto_payload_label.setVisible(automatic)
        self.auto_config.setVisible(automatic)
        if automatic:
            self._update_auto_preview()

    def _next_auto_cmd_id(self):
        current = self._utc_timestamp()
        previous = int(self.settings.value("mqtt/publish/last_auto_cmd_id", 0))
        if current <= previous:
            self._status("自动发布已暂停：cmd_id必须使用新的UTC Unix时间秒，请稍后再试或检查系统时钟")
            return None
        self.settings.setValue("mqtt/publish/last_auto_cmd_id", current)
        self.settings.sync()
        return current

    def _candidate_auto_cmd_id(self):
        return self._utc_timestamp()

    def _automatic_payload(self, cmd_id):
        return json.dumps({
            "cmd_id": cmd_id,
            "ver": 1,
            "wu": self.auto_wu.value(),
            "tilt": self.auto_tilt.value(),
            "sleep": self.auto_sleep.value(),
        }, ensure_ascii=False, indent=2)

    def _update_auto_preview(self):
        if hasattr(self, "auto_preview"):
            self.auto_preview.setPlainText(
                self._automatic_payload(self._candidate_auto_cmd_id()))

    def _payload_for_publish(self):
        if self.publish_mode.currentData() == "auto":
            cmd_id = self._next_auto_cmd_id()
            if cmd_id is None:
                return None
            return self._automatic_payload(cmd_id)
        return self.publish_payload.toPlainText()

    def publish(self):
        topic = self.publish_topic.text()
        if not topic:
            QMessageBox.warning(self, "发布失败", "Topic不能为空。")
            return
        payload = self._payload_for_publish()
        if payload is None:
            return
        if self.publish_mode.currentData() == "auto":
            self.auto_preview.setPlainText(payload)
        self.mqtt_client.publish(
            topic,
            payload.encode("utf-8"),
            self.publish_qos.currentData(),
            self.publish_retain.isChecked(),
        )

    def subscribe_topic(self, row):
        topic = row["topic"].text()
        if not topic:
            QMessageBox.warning(self, "订阅失败", "Topic不能为空。")
            return
        self.mqtt_client.subscribe(topic, row["qos"].currentData())

    def unsubscribe_topic(self, row):
        topic = row["topic"].text()
        if topic:
            self.mqtt_client.unsubscribe(topic)

    def on_connected(self):
        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)
        self.status.setText("● MQTT已连接")
        self.status.setObjectName("connectionOn")
        self._polish(self.status)

    def on_disconnected(self):
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)
        self.status.setText("● MQTT未连接")
        self.status.setObjectName("connectionOff")
        self._polish(self.status)

    def on_error(self, message):
        self._status(message)
        self.status.setText(f"● {message}")

    def add_message(self, message: MqttMessage):
        try:
            from paho.mqtt.client import topic_matches_sub
        except ImportError:
            return
        for row in self.subscribe_rows:
            subscription = row["topic"].text()
            if not subscription:
                continue
            try:
                matches = topic_matches_sub(subscription, message.topic)
            except (TypeError, ValueError):
                matches = False
            if matches:
                row["messages"].append(self._format_message(message))
                row["history"].setPlainText("\n\n".join(row["messages"]))
                row["history"].verticalScrollBar().setValue(
                    row["history"].verticalScrollBar().maximum())

    def clear_history(self, row):
        row["messages"].clear()
        row["history"].clear()

    @staticmethod
    def _format_message(message):
        try:
            payload = message.payload.decode("utf-8")
        except UnicodeDecodeError:
            payload = "0x" + message.payload.hex(" ")
        retain = "Retain" if message.retain else "非Retain"
        return (f"[{message.received_at.strftime('%Y-%m-%d %H:%M:%S')}] "
                f"QoS={message.qos} {retain}\n"
                f"Topic: {message.topic}\n"
                f"{payload}")

    def _setting_bool(self, key, default):
        value = self.settings.value(key, default)
        return str(value).lower() in {"1", "true", "yes", "on"}

    def _status(self, message):
        self.page_status.setText(message)
        window = self.window()
        if hasattr(window, "statusBar"):
            window.statusBar().showMessage(message, 5000)

    @staticmethod
    def _polish(widget):
        widget.style().unpolish(widget)
        widget.style().polish(widget)
