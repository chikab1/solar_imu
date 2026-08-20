from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QPixmap
from PySide6.QtSerialPort import QSerialPortInfo
from PySide6.QtWidgets import (QComboBox, QFrame, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QMainWindow, QMessageBox, QPushButton,
    QStackedWidget, QVBoxLayout, QWidget)

from app.device_client import DeviceClient
from app.mqtt_client import MqttClient
from app.protocol.commands import Command, STATUS_TEXT, Status
from .mqtt_page import MqttPage
from .pages import (AboutPage, ConfigPage, LiveAttitudePage, ModemPage,
                    OverviewPage, SerialPage, SixAxisPage)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Solar IMU 路牌监测工具 V1.2.0")
        self.resize(1240, 800)
        self.client = DeviceClient(self)
        self.mqtt_client = MqttClient(self)
        self._sleep_disconnect_pending = False
        self._build_ui()
        self.client.connected.connect(self._connected)
        self.client.disconnected.connect(self._disconnected)
        self.client.error.connect(self._error)
        self.client.response.connect(self._response_status)
        self.client.request_queued.connect(self._request_queued)
        self.refresh_ports()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QHBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(245)
        side = QVBoxLayout(sidebar)
        brand_header = QHBoxLayout()
        logo = QLabel()
        logo.setObjectName("universityLogo")
        logo_path = Path(__file__).resolve().parent / "assets" / "hdu_logo.png"
        pixmap = QPixmap(str(logo_path))
        if not pixmap.isNull():
            logo.setPixmap(pixmap.scaled(64, 64, Qt.KeepAspectRatio,
                                        Qt.SmoothTransformation))
        logo.setFixedSize(64, 64)
        logo.setAlignment(Qt.AlignCenter)
        brand_text = QVBoxLayout()
        brand = QLabel("SOLAR IMU")
        brand.setObjectName("brandTitle")
        subtitle = QLabel("路牌状态监测")
        subtitle.setObjectName("brandSubtitle")
        brand_text.addWidget(brand)
        brand_text.addWidget(subtitle)
        brand_header.addWidget(logo)
        brand_header.addLayout(brand_text, 1)
        side.addLayout(brand_header)
        self.nav = QListWidget()
        self.nav.setObjectName("navigation")
        side.addWidget(self.nav, 1)
        self.connection_dot = QLabel("● 未连接")
        self.connection_dot.setObjectName("connectionOff")
        side.addWidget(self.connection_dot)
        outer.addWidget(sidebar)
        content = QVBoxLayout()
        connection = QHBoxLayout()
        self.port = QComboBox()
        self.refresh_button = QPushButton("刷新串口")
        self.connect_button = QPushButton("连接设备")
        self.connect_button.setObjectName("primaryButton")
        connection.addWidget(QLabel("设备端口"))
        connection.addWidget(self.port, 1)
        connection.addWidget(self.refresh_button)
        connection.addWidget(self.connect_button)
        content.addLayout(connection)
        self.operation_feedback = QLabel("● 准备就绪：连接设备后可执行操作")
        self.operation_feedback.setObjectName("operationFeedbackInfo")
        content.addWidget(self.operation_feedback)
        self.stack = QStackedWidget()
        content.addWidget(self.stack, 1)
        outer.addLayout(content, 1)
        self.live_attitude_page = LiveAttitudePage(self.client)
        self.six_axis_page = SixAxisPage(self.client)
        pages = [
            ("实时姿态", self.live_attitude_page),
            ("设备总览", OverviewPage(self.client)),
            ("参数设置", ConfigPage(self.client)),
            ("4G与上报", ModemPage(self.client)),
            ("六轴数据", self.six_axis_page),
            ("通信记录", SerialPage(self.client)),
            ("MQTT远程消息", MqttPage(self.mqtt_client)),
            ("关于", AboutPage()),
        ]
        for title, page in pages:
            self.nav.addItem(QListWidgetItem(title))
            self.stack.addWidget(page)
        self.nav.currentRowChanged.connect(self.stack.setCurrentIndex)
        self.nav.setCurrentRow(0)
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)

    def refresh_ports(self):
        selected = self.port.currentData()
        self.port.clear()
        ports = sorted(QSerialPortInfo.availablePorts(), key=lambda p: p.portName())
        for info in ports:
            description = info.description() or "串口设备"
            self.port.addItem(f"{info.portName()} — {description}", info.portName())
        if selected:
            index = self.port.findData(selected)
            if index >= 0:
                self.port.setCurrentIndex(index)
        if not ports:
            self.port.addItem("未发现串口", "")

    def toggle_connection(self):
        if self.client.is_connected:
            self.client.disconnect_device()
            return
        if not self.port.currentData():
            QMessageBox.warning(self, "无法连接", "未发现可用串口，请连接USB转串口设备后刷新。")
        else:
            self.connect_button.setEnabled(False)
            self.connect_button.setText("正在唤醒设备…")
            self.client.connect_device(self.port.currentData())

    def _connected(self):
        self.connect_button.setEnabled(True)
        self.connect_button.setText("断开连接")
        self.connection_dot.setText("● 设备已连接")
        self.connection_dot.setObjectName("connectionOn")
        self.connection_dot.style().unpolish(self.connection_dot)
        self.connection_dot.style().polish(self.connection_dot)
        self.statusBar().showMessage("连接成功", 3000)
        self._show_operation_feedback("● 设备已连接，可以开始操作", "success")

    def _disconnected(self):
        self.connect_button.setEnabled(True)
        self.connect_button.setText("连接设备")
        self.connection_dot.setText("● 未连接")
        self.connection_dot.setObjectName("connectionOff")
        self.connection_dot.style().unpolish(self.connection_dot)
        self.connection_dot.style().polish(self.connection_dot)
        if self._sleep_disconnect_pending:
            self._sleep_disconnect_pending = False
            self._show_operation_feedback(
                "● 设备已进入低功耗待机，串口连接已自动断开", "success"
            )
        else:
            self._show_operation_feedback("● 设备未连接", "info")

    def _disconnect_after_sleep(self):
        self.client.disconnect_device()

    def _error(self, message):
        self.statusBar().showMessage(message, 8000)
        self._show_operation_feedback(f"● 操作未执行：{message}", "error")
        QMessageBox.warning(self, "设备通信", message)

    def _show_operation_feedback(self, text, level):
        names = {
            "info": "operationFeedbackInfo",
            "success": "operationFeedbackSuccess",
            "error": "operationFeedbackError",
        }
        self.operation_feedback.setText(text)
        self.operation_feedback.setObjectName(names[level])
        self.operation_feedback.style().unpolish(self.operation_feedback)
        self.operation_feedback.style().polish(self.operation_feedback)

    def _request_queued(self, command):
        pending = {
            Command.MODEM_ON: "正在开启 4G 模块…",
            Command.MODEM_OFF: "正在关闭 4G 模块…",
            Command.RUN_REPORT: "上报任务已发送，正在等待设备确认…",
            Command.SLEEP: "正在请求设备进入低功耗待机…",
            Command.SET_CONFIG: "正在写入工作参数…",
            Command.CLEAR_QUEUE: "正在清除未发送事件…",
        }
        try:
            text = pending.get(Command(command))
        except ValueError:
            text = None
        if text:
            self._show_operation_feedback(f"● {text}", "info")

    def _response_status(self, command, status, _data):
        if status:
            try:
                text = STATUS_TEXT[Status(status)]
            except (ValueError, KeyError):
                text = f"未知错误 {status}"
            self.statusBar().showMessage(f"命令 0x{command:02X}：{text}", 6000)
            self._show_operation_feedback(f"● 操作失败：{text}", "error")
            return

        completed = {
            Command.MODEM_ON: "4G 模块已开启",
            Command.MODEM_OFF: "4G 模块已关闭",
            Command.RUN_REPORT: "上报任务已开始，可在设备总览查看最终结果",
            Command.SLEEP: "低功耗待机指令已确认；下次串口通信会重新唤醒设备",
            Command.SET_CONFIG: "参数已写入，正在回读验证",
            Command.CLEAR_QUEUE: "未发送事件已清除",
        }
        try:
            text = completed.get(Command(command))
        except ValueError:
            text = None
        if text:
            self.statusBar().showMessage(text, 6000)
            self._show_operation_feedback(f"● {text}", "success")
            if command == Command.SLEEP:
                self._sleep_disconnect_pending = True
                QTimer.singleShot(100, self._disconnect_after_sleep)

    def closeEvent(self, event):
        self.live_attitude_page.stop_recording()
        self.six_axis_page.stop_recording()
        self.mqtt_client.close()
        self.client.disconnect_device()
        super().closeEvent(event)
