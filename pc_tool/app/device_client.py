from PySide6.QtCore import QObject, Qt, QTimer, Signal, Slot

from app.protocol.commands import Command, config_payload
from app.transport.serial_worker import SerialWorker


class DeviceClient(QObject):
    connected = Signal()
    disconnected = Signal()
    response = Signal(int, int, object)
    error = Signal(str)
    traffic = Signal(str, object)
    request_queued = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.worker = None

    @property
    def is_connected(self):
        return bool(self.worker and self.worker.isRunning())

    def connect_device(self, port: str):
        self.disconnect_device()
        self.worker = SerialWorker(port)
        # QThread子类对象本身属于主线程；明确使用DirectConnection，确保
        # run()内发出的Python对象信号先进入中继，再由Qt排队给界面接收者。
        self.worker.connected.connect(self._on_connected, Qt.DirectConnection)
        self.worker.disconnected.connect(self._on_disconnected, Qt.DirectConnection)
        self.worker.response.connect(self._on_response, Qt.DirectConnection)
        self.worker.error.connect(self._on_error, Qt.DirectConnection)
        self.worker.traffic.connect(self._on_traffic, Qt.DirectConnection)
        self.worker.start()

    @Slot()
    def _on_connected(self):
        QTimer.singleShot(0, self, self.connected.emit)

    @Slot()
    def _on_disconnected(self):
        QTimer.singleShot(0, self, self.disconnected.emit)

    @Slot(int, int, object)
    def _on_response(self, command, status, data):
        QTimer.singleShot(0, self,
                          lambda c=command, s=status, d=bytes(data):
                          self.response.emit(c, s, d))

    @Slot(str)
    def _on_error(self, message):
        QTimer.singleShot(0, self, lambda m=message: self.error.emit(m))

    @Slot(str, object)
    def _on_traffic(self, direction, data):
        QTimer.singleShot(0, self,
                          lambda direction=direction, data=bytes(data):
                          self.traffic.emit(direction, data))

    def disconnect_device(self):
        if self.worker:
            self.worker.shutdown()
            self.worker.wait(2500)
            self.worker = None

    def ensure_connected(self):
        """Return whether the serial device is usable, showing one common hint.

        Pages that issue multiple commands for one click must call this once
        before starting their batch, so a disconnected device never produces a
        dialog for every individual command.
        """
        if self.is_connected:
            return True
        self.error.emit("请先连接设备")
        return False

    def request(self, command: int, payload: bytes = b""):
        if not self.ensure_connected():
            return False
        self.worker.request(command, payload)
        self.request_queued.emit(int(command))
        return True

    def set_config(self, wu_mg: int, tilt_deg: int, sleep_sec: int, vlow_mv: int):
        return self.request(Command.SET_CONFIG,
                            config_payload(wu_mg, tilt_deg, sleep_sec, vlow_mv))
