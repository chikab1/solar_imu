from datetime import datetime
import math
from pathlib import Path
import struct

from PySide6.QtCore import QSettings, Qt, QTimer
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QFormLayout,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMessageBox, QPushButton,
    QScrollArea, QTableWidget, QTableWidgetItem, QTextEdit, QVBoxLayout, QWidget)

from app.protocol.commands import Command, STATUS_TEXT, Status, report_payload
from app.protocol.models import (DeviceIdentity, DeviceStatus, EventRecord,
                                  ImuDiagnostic, ImuLive)
from app.telemetry_recorder import TelemetryRecorder
from .widgets import AttitudePreview3D, MultiLineChart, StatusCard


def page_title(text: str):
    label = QLabel(text)
    label.setObjectName("pageTitle")
    return label


def set_combo_value(combo: QComboBox, value):
    index = combo.findData(value)
    if index >= 0:
        combo.setCurrentIndex(index)


class RecordingControls(QWidget):
    SETTINGS_KEY = "telemetry/record_directory"

    def __init__(self, parent, category, fieldnames):
        super().__init__(parent)
        self.settings = QSettings("SolarIMU", "PC Tool")
        self.recorder = TelemetryRecorder(category, fieldnames)
        self.directory = self.settings.value(self.SETTINGS_KEY, "")
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.choose_button = QPushButton("选择保存文件夹")
        self.record_button = QPushButton("开始记录")
        self.status_label = QLabel()
        self.status_label.setObjectName("hintLabel")
        layout.addWidget(self.choose_button)
        layout.addWidget(self.record_button)
        layout.addWidget(self.status_label)
        self.choose_button.clicked.connect(self.choose_directory)
        self.record_button.clicked.connect(self.toggle_recording)
        self._update_status()

    @property
    def is_recording(self):
        return self.recorder.is_recording

    def choose_directory(self):
        selected = QFileDialog.getExistingDirectory(
            self, "选择遥测数据保存文件夹", self.directory or "")
        if selected:
            self.directory = selected
            self.settings.setValue(self.SETTINGS_KEY, selected)
            self.settings.sync()
            self._update_status()

    def toggle_recording(self):
        if self.is_recording:
            self.stop()
            return
        if not self.directory or not Path(self.directory).is_dir():
            QMessageBox.information(self, "未选择保存文件夹", "请先选择一个有效的保存文件夹。")
            return
        try:
            path = self.recorder.start(self.directory)
        except (OSError, ValueError) as exc:
            self.status_label.setText(f"记录失败：{exc}")
            return
        self.record_button.setText("停止记录")
        self.status_label.setText(f"记录中：{path.name}")

    def write(self, values):
        if not self.is_recording:
            return
        try:
            path = self.recorder.write(values)
        except OSError as exc:
            self.status_label.setText(f"记录失败：{exc}")
            self.record_button.setText("开始记录")
            return
        self.status_label.setText(f"记录中：{path.name}")

    def stop(self):
        self.recorder.stop()
        self.record_button.setText("开始记录")
        self._update_status()

    def _update_status(self):
        if self.is_recording:
            return
        if self.directory and Path(self.directory).is_dir():
            self.status_label.setText(f"目录：{Path(self.directory).name}")
        else:
            self.status_label.setText("未选择目录")


class LiveAttitudePage(QWidget):
    def __init__(self, client):
        super().__init__()
        self.client = client
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._poll)
        self.recording = RecordingControls(
            self, "attitude", ("timestamp", "pitch_deg", "roll_deg"))
        layout = QVBoxLayout(self)
        layout.addWidget(page_title("实时姿态"))
        controls = QHBoxLayout()
        self.interval = QComboBox()
        for text, ms in (("0.1 秒", 100), ("0.2 秒", 200), ("0.5 秒", 500),
                         ("1 秒", 1000), ("2 秒", 2000)):
            self.interval.addItem(text, ms)
        self.interval.setCurrentIndex(2)
        self.start_button = QPushButton("开始读取")
        self.start_button.setObjectName("primaryButton")
        controls.addWidget(QLabel("刷新频率"))
        controls.addWidget(self.interval)
        controls.addStretch()
        controls.addWidget(self.start_button)
        controls.addWidget(self.recording)
        layout.addLayout(controls)
        cards = QHBoxLayout()
        self.pitch_card = StatusCard("Pitch", "--", "°")
        self.roll_card = StatusCard("Roll", "--", "°")
        cards.addWidget(self.pitch_card)
        cards.addWidget(self.roll_card)
        layout.addLayout(cards)
        self.preview = AttitudePreview3D()
        layout.addWidget(self.preview, 1)
        hint = QLabel("预览根据实时 Pitch 和 Roll 绘制；Yaw 未由传感器提供，因此不会显示或推算。")
        hint.setObjectName("hintLabel")
        layout.addWidget(hint)
        self.start_button.clicked.connect(self.toggle)
        self.interval.currentIndexChanged.connect(self._interval_changed)
        client.response.connect(self.on_response)
        client.disconnected.connect(self.stop_reading)
        client.disconnected.connect(self.stop_recording)

    def toggle(self):
        if self.timer.isActive():
            self.stop_reading()
        else:
            if not self.client.request(Command.GET_IMU_LIVE):
                return
            self.timer.start(self.interval.currentData())
            self.start_button.setText("停止读取")

    def _poll(self):
        if not self.client.request(Command.GET_IMU_LIVE):
            self.stop_reading()

    def stop_reading(self):
        self.timer.stop()
        self.start_button.setText("开始读取")

    def stop_recording(self):
        self.recording.stop()

    def _interval_changed(self):
        if self.timer.isActive():
            self.timer.start(self.interval.currentData())

    def on_response(self, command, status, data):
        if command != Command.GET_IMU_LIVE or status != Status.OK:
            return
        live = ImuLive.parse(data)
        pitch, roll = live.pitch_cdeg / 100, live.roll_cdeg / 100
        self.pitch_card.set_value(f"{pitch:.2f}")
        self.roll_card.set_value(f"{roll:.2f}")
        self.preview.set_attitude(pitch, roll)
        self.recording.write({"pitch_deg": f"{pitch:.2f}",
                              "roll_deg": f"{roll:.2f}"})


class OverviewPage(QWidget):
    def __init__(self, client):
        super().__init__()
        self.client = client
        self._read_all_pending = False
        layout = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(page_title("设备总览"))
        top.addStretch()
        read = QPushButton("读取全部状态")
        read.setObjectName("primaryButton")
        read.clicked.connect(self.read_all)
        top.addWidget(read)
        layout.addLayout(top)
        cards = QGridLayout()
        self.cards = {
            "voltage": StatusCard("设备电压", "--", "V"),
            "network": StatusCard("4G模块", "--", ""),
            "imu": StatusCard("姿态传感器", "--", ""),
            "queue": StatusCard("待发送事件", "--", "条"),
        }
        for i, card in enumerate(self.cards.values()):
            cards.addWidget(card, 0, i)
        layout.addLayout(cards)
        identity = QGroupBox("设备身份")
        form = QFormLayout(identity)
        self.imei = QLabel("--")
        self.uid = QLabel("--")
        self.topic = QLabel("--")
        for value in (self.imei, self.uid, self.topic):
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value.setToolTip("可拖选文本后按 Ctrl+C 复制")
        form.addRow("IMEI（4G模块编号）", self.imei)
        form.addRow("设备唯一编号", self.uid)
        form.addRow("数据上报地址", self.topic)
        layout.addWidget(identity)
        details = QGroupBox("运行状态")
        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["项目", "当前状态"])
        self.table.horizontalHeader().setStretchLastSection(True)
        QVBoxLayout(details).addWidget(self.table)
        layout.addWidget(details, 1)
        queue_box = QGroupBox("未发送事件")
        queue_layout = QVBoxLayout(queue_box)
        self.queue_text = QLabel("暂无未发送事件")
        self.queue_text.setWordWrap(True)
        clear = QPushButton("清除未发送事件")
        clear.setObjectName("dangerButton")
        clear.clicked.connect(lambda: client.request(Command.CLEAR_QUEUE))
        queue_layout.addWidget(self.queue_text)
        queue_layout.addWidget(clear, 0, Qt.AlignRight)
        layout.addWidget(queue_box)
        client.response.connect(self.on_response)

    def read_all(self):
        if not self.client.ensure_connected():
            return
        # 队列索引必须小于队列数量。先读取状态，确认有待发送事件后才读取第0条，
        # 避免空队列返回BAD_VALUE被误显示为设备参数错误。
        self._read_all_pending = True
        for cmd in (Command.GET_STATUS, Command.GET_IMU_DIAG,
                    Command.GET_DEVICE_ID):
            self.client.request(cmd)

    def _rows(self, rows):
        self.table.setRowCount(len(rows))
        for row, (name, value) in enumerate(rows):
            self.table.setItem(row, 0, QTableWidgetItem(name))
            self.table.setItem(row, 1, QTableWidgetItem(str(value)))

    def on_response(self, command, status, data):
        if status != Status.OK:
            if command == Command.READ_QUEUE and status == Status.BAD_VALUE:
                self.queue_text.setText("暂无未发送事件")
            return
        if command == Command.GET_STATUS:
            value = DeviceStatus.parse(data)
            self.cards["voltage"].set_value(f"{value.voltage_mv / 1000:.2f}")
            self.cards["network"].set_value("已开启" if value.modem_on else "已关闭")
            self.cards["imu"].set_value("正常" if value.imu_ok else "需检查")
            self.cards["queue"].set_value(value.queue_count)
            self._rows([
                ("4G模块状态", "已开启" if value.modem_on else "已关闭"),
                ("信号强度", f"{value.csq}（0~31，越大越好）" if value.csq != 99 else "暂无信号"),
                ("定时唤醒周期", f"{value.sleep_sec / 60:g} 分钟"),
                ("最近上报", "成功" if value.last_report_ok else "未成功或暂无记录"),
                ("最近上报耗时", f"{value.report_duration_ms / 1000:.1f} 秒"),
                ("RTC唤醒次数", value.rtc_hw_wake_count),
                ("IMU唤醒次数", value.imu_exti_wake_count),
                ("串口异常次数", value.uart_error_count),
            ])
            if self._read_all_pending:
                self._read_all_pending = False
                if value.queue_count:
                    self.client.request(Command.READ_QUEUE)
                else:
                    self.queue_text.setText("暂无未发送事件")
        elif command == Command.GET_DEVICE_ID:
            value = DeviceIdentity.parse(data)
            self.imei.setText(value.imei if value.imei_valid else "尚未读取（完成一次4G上报后可用）")
            self.uid.setText(value.mcu_uid)
            self.topic.setText(value.mqtt_topic or "尚未生成")
        elif command == Command.READ_QUEUE:
            count, index = data[:2]
            event = EventRecord.parse(data[2:])
            when = datetime.fromtimestamp(event.timestamp).strftime("%Y-%m-%d %H:%M:%S") if event.timestamp else "时间未知"
            self.queue_text.setText(
                f"事件 #{event.event_id}　{when}　电压 {event.voltage_mv / 1000:.2f} V　"
                f"峰值加速度 {event.acc_norm_peak_mg} mg　唤醒类型 {event.wake_reason}"
            )
        elif command == Command.CLEAR_QUEUE:
            self.queue_text.setText("暂无未发送事件")
            self.cards["queue"].set_value(0)


class ConfigPage(QWidget):
    def __init__(self, client):
        super().__init__()
        self.client = client
        self.pending = None
        self.verify_status = None
        self.verify_diag = None
        layout = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(page_title("工作参数"))
        top.addStretch()
        read = QPushButton("读取当前参数")
        read.clicked.connect(self.read_config)
        top.addWidget(read)
        layout.addLayout(top)
        box = QGroupBox("监测与唤醒设置")
        form = QFormLayout(box)
        self.wu = QComboBox()
        for value in (400, 500, 600, 750, 800, 1000, 1250, 1500, 2000):
            self.wu.addItem(f"{value} mg" + ("（推荐）" if value == 500 else ""), value)
        self.wu.setCurrentIndex(self.wu.findData(500))
        self.tilt = QComboBox()
        for value in (10, 20, 30, 40):
            self.tilt.addItem(f"{value}°" + ("（推荐）" if value == 30 else ""), value)
        self.voltage = QComboBox()
        for mv in range(3500, 4001, 50):
            self.voltage.addItem(f"{mv / 1000:.2f} V", mv)
        self.rtc = QComboBox()
        for minutes in (10, 20, 30, 60, 120, 360, 720, 1092):
            self.rtc.addItem(f"{minutes} 分钟" + ("（默认）" if minutes == 60 else ""), minutes * 60)
        form.addRow("震动唤醒阈值", self.wu)
        form.addRow("倾角唤醒阈值", self.tilt)
        form.addRow("低电压保护", self.voltage)
        form.addRow("定时上报周期", self.rtc)
        layout.addWidget(box)
        note = QLabel("推荐用于路牌监测：500 mg、30°。倾角芯片实际支持10°/20°/30°/40°四档；RTC最低10分钟。")
        note.setWordWrap(True)
        note.setObjectName("hintLabel")
        layout.addWidget(note)
        self.result = QLabel("请先读取当前参数")
        layout.addWidget(self.result)
        write = QPushButton("写入工作参数")
        write.setObjectName("primaryButton")
        write.clicked.connect(self.write_config)
        layout.addWidget(write, 0, Qt.AlignRight)
        layout.addStretch()
        client.response.connect(self.on_response)

    def read_config(self):
        if not self.client.ensure_connected():
            return
        self.client.request(Command.GET_STATUS)
        self.client.request(Command.GET_IMU_DIAG)

    def write_config(self):
        self.pending = (self.wu.currentData(), self.tilt.currentData(),
                        self.rtc.currentData(), self.voltage.currentData())
        self.verify_status = self.verify_diag = None
        try:
            if not self.client.set_config(*self.pending):
                self.pending = None
                return
            self.result.setText("正在写入并回读确认…")
        except ValueError as exc:
            self.result.setText(str(exc))

    def on_response(self, command, status, data):
        if command not in (Command.SET_CONFIG, Command.GET_STATUS, Command.GET_IMU_DIAG):
            return
        if status != Status.OK:
            self.result.setText(STATUS_TEXT.get(Status(status), f"设备错误 {status}"))
            return
        if command == Command.SET_CONFIG:
            if self.client.ensure_connected():
                self.client.request(Command.GET_STATUS)
                self.client.request(Command.GET_IMU_DIAG)
            return
        if command == Command.GET_STATUS:
            self.verify_status = DeviceStatus.parse(data)
            set_combo_value(self.rtc, self.verify_status.sleep_sec)
            set_combo_value(self.voltage, self.verify_status.vlow_mv)
        else:
            self.verify_diag = ImuDiagnostic.parse(data)
            set_combo_value(self.wu, self.verify_diag.wu_mg)
            set_combo_value(self.tilt, self.verify_diag.tilt_deg)
        if self.pending and self.verify_status and self.verify_diag:
            actual = (self.verify_diag.wu_mg, self.verify_diag.tilt_deg,
                      self.verify_status.sleep_sec, self.verify_status.vlow_mv)
            if actual == self.pending:
                self.result.setText("✓ 参数已写入设备，回读校验一致")
                self.pending = None
            else:
                self.result.setText(f"写入后回读不一致：期望{self.pending}，设备{actual}")
        elif not self.pending and self.verify_status and self.verify_diag:
            self.result.setText("已读取设备当前参数")


class ModemPage(QWidget):
    def __init__(self, client):
        super().__init__()
        self.client = client
        layout = QVBoxLayout(self)
        layout.addWidget(page_title("4G与数据上报"))
        intro = QLabel("这里控制设备的4G模块，并可人工触发一次完整的数据采集、定位和上报。")
        intro.setWordWrap(True)
        layout.addWidget(intro)
        box = QGroupBox("4G模块")
        row = QHBoxLayout(box)
        on = QPushButton("开启4G模块")
        off = QPushButton("关闭4G模块")
        on.clicked.connect(lambda: client.request(Command.MODEM_ON))
        off.clicked.connect(lambda: client.request(Command.MODEM_OFF))
        row.addWidget(on)
        row.addWidget(off)
        layout.addWidget(box)
        report_row = QHBoxLayout()
        self.include_gps = QCheckBox("携带 GPS 位置信息")
        self.include_gps.setChecked(True)
        self.report = QPushButton("立即采集并上报")
        self.report.setObjectName("primaryButton")
        self.report.clicked.connect(self.run_report)
        report_row.addWidget(self.include_gps)
        report_row.addStretch()
        report_row.addWidget(self.report)
        layout.addLayout(report_row)
        self.gps_hint = QLabel()
        self.gps_hint.setObjectName("hintLabel")
        self.gps_hint.setWordWrap(True)
        layout.addWidget(self.gps_hint)
        hint = QLabel("上报任务已提交后会在后台执行；可回到“设备总览”查看最近结果。")
        hint.setObjectName("hintLabel")
        layout.addWidget(hint)
        sleep_box = QGroupBox("设备低功耗控制")
        sleep_layout = QHBoxLayout(sleep_box)
        sleep_hint = QLabel("待机后设备会关闭当前活动并停止维护通信；需要再次操作时，"
                            "重新连接设备即可唤醒。")
        sleep_hint.setWordWrap(True)
        self.sleep_button = QPushButton("进入低功耗待机")
        self.sleep_button.setObjectName("dangerButton")
        self.sleep_button.clicked.connect(self.enter_sleep)
        sleep_layout.addWidget(sleep_hint, 1)
        sleep_layout.addWidget(self.sleep_button)
        layout.addWidget(sleep_box)
        layout.addStretch()
        self.include_gps.toggled.connect(self._update_gps_hint)
        self._update_gps_hint(self.include_gps.isChecked())

    def _update_gps_hint(self, include_gps):
        if include_gps:
            self.gps_hint.setText(
                "已选择 GPS：本次会等待定位结果；在室内或遮挡环境无定位时，"
                "最多等待约 60 秒后仍会完成上报，并标记定位失败。"
            )
        else:
            self.gps_hint.setText(
                "未选择 GPS：本次不启动搜星，立即采集、联网并上报；数据中不包含位置字段。"
            )

    def run_report(self):
        self.client.request(Command.RUN_REPORT,
                            report_payload(self.include_gps.isChecked()))

    def enter_sleep(self):
        if not self.client.ensure_connected():
            return
        answer = QMessageBox.question(
            self, "确认进入低功耗待机",
            "设备将停止当前活动并进入低功耗待机。\n"
            "之后需要重新连接设备才能继续维护操作。是否继续？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.client.request(Command.SLEEP)


class SixAxisPage(QWidget):
    def __init__(self, client):
        super().__init__()
        self.client = client
        self.timer = QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self._poll)
        self.recording = RecordingControls(
            self,
            "six_axis",
            ("timestamp", "acc_x_mg", "acc_y_mg", "acc_z_mg", "acc_total_mg",
             "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "pitch_deg", "roll_deg"),
        )
        root = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.addWidget(page_title("六轴数据"))

        controls = QHBoxLayout()
        self.acc_enable = QCheckBox("加速度")
        self.gyro_enable = QCheckBox("角速度")
        self.angle_enable = QCheckBox("姿态角")
        for item in (self.acc_enable, self.gyro_enable, self.angle_enable):
            item.setChecked(True)
            controls.addWidget(item)
        controls.addStretch()
        self.clear_button = QPushButton("清空曲线")
        self.start = QPushButton("开始读取")
        self.start.setObjectName("primaryButton")
        self.start.clicked.connect(self.toggle)
        self.clear_button.clicked.connect(self.clear_history)
        controls.addWidget(self.clear_button)
        controls.addWidget(self.start)
        controls.addWidget(self.recording)
        layout.addLayout(controls)

        self.groups = {}
        grid = QGridLayout()
        acc_box = QGroupBox("加速度")
        acc_layout = QGridLayout(acc_box)
        self.acc_values = {}
        for row, axis in enumerate(("X", "Y", "Z")):
            acc_layout.addWidget(QLabel(axis), row, 0)
            value = QLabel("-- mg")
            acc_layout.addWidget(value, row, 1)
            self.acc_values[axis] = value
        acc_layout.addWidget(QLabel("总加速度"), 0, 2)
        self.total_acc_value = QLabel("-- mg")
        acc_layout.addWidget(self.total_acc_value, 0, 3)
        self.groups["acc"] = acc_box
        grid.addWidget(acc_box, 0, 0)

        for col, (key, title, axes, unit) in enumerate((
            ("gyro", "角速度", ("X", "Y", "Z"), "°/s"),
            ("angle", "姿态角", ("Pitch", "Roll"), "°"),
        ), start=1):
            box = QGroupBox(title)
            form = QFormLayout(box)
            values = []
            for axis in axes:
                value = QLabel(f"-- {unit}")
                form.addRow(axis, value)
                values.append(value)
            self.groups[key] = box
            setattr(self, f"{key}_values", values)
            grid.addWidget(box, 0, col)
        layout.addLayout(grid)

        self.acc_chart = MultiLineChart(
            "加速度历史", "mg",
            (("X", "#df4a4a"), ("Y", "#36a96d"),
             ("Z", "#2d77d1"), ("总加速度", "#9a5ec5")),
        )
        self.gyro_chart = MultiLineChart(
            "角速度历史", "°/s",
            (("X", "#df4a4a"), ("Y", "#36a96d"), ("Z", "#2d77d1")),
        )
        self.angle_chart = MultiLineChart(
            "姿态角历史", "°",
            (("Pitch", "#1688c8"), ("Roll", "#f29d38")),
        )
        self.chart_boxes = {
            "acc": self._chart_box("加速度历史曲线", self.acc_chart),
            "gyro": self._chart_box("角速度历史曲线", self.gyro_chart),
            "angle": self._chart_box("姿态角历史曲线", self.angle_chart),
        }
        for box in self.chart_boxes.values():
            layout.addWidget(box)
        layout.addStretch()
        scroll.setWidget(content)
        root.addWidget(scroll)

        self.acc_enable.toggled.connect(
            lambda visible: self._set_section_visible("acc", visible))
        self.gyro_enable.toggled.connect(
            lambda visible: self._set_section_visible("gyro", visible))
        self.angle_enable.toggled.connect(
            lambda visible: self._set_section_visible("angle", visible))
        client.response.connect(self.on_response)
        client.disconnected.connect(self.stop_reading)
        client.disconnected.connect(self.stop_recording)

    @staticmethod
    def _chart_box(title, chart):
        box = QGroupBox(title)
        QVBoxLayout(box).addWidget(chart)
        return box

    def _set_section_visible(self, key, visible):
        self.groups[key].setVisible(visible)
        self.chart_boxes[key].setVisible(visible)

    def toggle(self):
        if self.timer.isActive():
            self.stop_reading()
        else:
            if not self.client.request(Command.GET_IMU_LIVE):
                return
            self.timer.start()
            self.start.setText("停止读取")

    def _poll(self):
        if not self.client.request(Command.GET_IMU_LIVE):
            self.stop_reading()

    def stop_reading(self):
        self.timer.stop()
        self.start.setText("开始读取")

    def stop_recording(self):
        self.recording.stop()

    def clear_history(self):
        self.acc_chart.clear()
        self.gyro_chart.clear()
        self.angle_chart.clear()

    def on_response(self, command, status, data):
        if command != Command.GET_IMU_LIVE or status != Status.OK:
            return
        value = ImuLive.parse(data)
        total_acc = math.sqrt(
            value.acc_x ** 2 + value.acc_y ** 2 + value.acc_z ** 2)
        for axis, number in zip(("X", "Y", "Z"),
                                (value.acc_x, value.acc_y, value.acc_z)):
            self.acc_values[axis].setText(f"{number} mg")
        self.total_acc_value.setText(f"{total_acc:.1f} mg")
        for label, number in zip(self.gyro_values,
                                 (value.gyro_x, value.gyro_y, value.gyro_z)):
            label.setText(f"{number} °/s")
        pitch = value.pitch_cdeg / 100
        roll = value.roll_cdeg / 100
        for label, number in zip(self.angle_values, (pitch, roll)):
            label.setText(f"{number:.2f} °")

        self.acc_chart.append({"X": value.acc_x, "Y": value.acc_y,
                               "Z": value.acc_z, "总加速度": total_acc})
        self.gyro_chart.append({"X": value.gyro_x, "Y": value.gyro_y,
                                "Z": value.gyro_z})
        self.angle_chart.append({"Pitch": pitch, "Roll": roll})
        self.recording.write({
            "acc_x_mg": value.acc_x,
            "acc_y_mg": value.acc_y,
            "acc_z_mg": value.acc_z,
            "acc_total_mg": f"{total_acc:.1f}",
            "gyro_x_dps": value.gyro_x,
            "gyro_y_dps": value.gyro_y,
            "gyro_z_dps": value.gyro_z,
            "pitch_deg": f"{pitch:.2f}",
            "roll_deg": f"{roll:.2f}",
        })


class SerialPage(QWidget):
    def __init__(self, client):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addWidget(page_title("通信记录"))
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)
        clear = QPushButton("清空记录")
        clear.clicked.connect(self.log.clear)
        layout.addWidget(clear, 0, Qt.AlignRight)
        client.traffic.connect(self.on_traffic)

    def on_traffic(self, direction, data):
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.append(f"[{stamp}] {direction}  {data.hex(' ').upper()}")


class AboutPage(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.addStretch()
        title = QLabel("Solar IMU Tool")
        title.setObjectName("aboutTitle")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        version = QLabel("路牌监测设备维护工具  V1.2.0")
        version.setAlignment(Qt.AlignCenter)
        layout.addWidget(version)
        text = QLabel("面向安装、运维和售后人员：查看姿态与六轴数据、设置唤醒参数、检查4G上报及未发送事件。")
        text.setWordWrap(True)
        text.setAlignment(Qt.AlignCenter)
        layout.addWidget(text)
        layout.addStretch()
