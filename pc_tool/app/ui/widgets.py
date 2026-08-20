from collections import deque
from math import cos, radians, sin

from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QFrame, QLabel, QVBoxLayout, QWidget


class StatusCard(QFrame):
    def __init__(self, title: str, value: str = "--", unit: str = "", parent=None):
        super().__init__(parent)
        self.setObjectName("statusCard")
        layout = QVBoxLayout(self)
        title_label = QLabel(title)
        title_label.setObjectName("cardTitle")
        self.value_label = QLabel(value)
        self.value_label.setObjectName("cardValue")
        self.unit_label = QLabel(unit)
        self.unit_label.setObjectName("cardUnit")
        layout.addWidget(title_label)
        layout.addWidget(self.value_label)
        layout.addWidget(self.unit_label)

    def set_value(self, value, unit=None):
        self.value_label.setText(str(value))
        if unit is not None:
            self.unit_label.setText(unit)


class LineChart(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(280)
        self.series = {
            "Pitch": (deque(maxlen=200), QColor("#1688c8")),
            "Roll": (deque(maxlen=200), QColor("#f29d38")),
        }
        self.visible = {name: True for name in self.series}

    def append(self, pitch: float, roll: float):
        self.series["Pitch"][0].append(pitch)
        self.series["Roll"][0].append(roll)
        self.update()

    def clear(self):
        for values, _ in self.series.values():
            values.clear()
        self.update()

    def set_series_visible(self, name: str, visible: bool):
        self.visible[name] = visible
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.rect().adjusted(42, 15, -15, -28)
        painter.fillRect(self.rect(), QColor("#ffffff"))
        painter.setPen(QPen(QColor("#dce6f0"), 1))
        for i in range(5):
            y = rect.top() + i * rect.height() / 4
            painter.drawLine(rect.left(), int(y), rect.right(), int(y))
        all_values = [v for name, (values, _) in self.series.items() if self.visible[name] for v in values]
        limit = max(10.0, max((abs(v) for v in all_values), default=10.0) * 1.15)
        painter.setPen(QColor("#71849a"))
        painter.drawText(5, rect.top() + 5, f"{limit:.0f}°")
        painter.drawText(9, rect.bottom(), f"-{limit:.0f}°")
        for name, (values, color) in self.series.items():
            if not self.visible[name] or len(values) < 2:
                continue
            points = []
            for i, value in enumerate(values):
                x = rect.left() + i * rect.width() / (values.maxlen - 1)
                y = rect.center().y() - value / limit * rect.height() / 2
                points.append(QPointF(x, y))
            painter.setPen(QPen(color, 2))
            painter.drawPolyline(points)


class MultiLineChart(QWidget):
    """Bounded, dependency-free history plot for related telemetry series."""

    def __init__(self, title, unit, series, parent=None, max_samples=300):
        super().__init__(parent)
        self.setMinimumHeight(190)
        self.title = title
        self.unit = unit
        self.series = {
            name: (deque(maxlen=max_samples), QColor(color))
            for name, color in series
        }
        self.visible = {name: True for name in self.series}

    def append(self, values):
        for name, (samples, _color) in self.series.items():
            samples.append(float(values[name]))
        self.update()

    def clear(self):
        for samples, _color in self.series.values():
            samples.clear()
        self.update()

    def set_series_visible(self, name, visible):
        if name in self.visible:
            self.visible[name] = visible
            self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#ffffff"))
        rect = self.rect().adjusted(48, 32, -16, -28)
        if rect.width() <= 0 or rect.height() <= 0:
            return

        painter.setPen(QColor("#31516c"))
        painter.drawText(12, 21, self.title)
        legend_items = [
            (name, color)
            for name, (_samples, color) in self.series.items()
            if self.visible[name]
        ]
        legend_width = sum(
            18 + painter.fontMetrics().horizontalAdvance(name) + 14
            for name, _color in legend_items
        )
        legend_x = max(rect.left(), rect.right() - legend_width)
        for name, color in legend_items:
            painter.setPen(QPen(color, 2))
            painter.drawLine(legend_x, 19, legend_x + 14, 19)
            painter.setPen(QColor("#61758a"))
            painter.drawText(legend_x + 18, 23, name)
            legend_x += 18 + painter.fontMetrics().horizontalAdvance(name) + 14

        painter.setPen(QPen(QColor("#dce6f0"), 1))
        for index in range(5):
            y = rect.top() + index * rect.height() / 4
            painter.drawLine(rect.left(), int(y), rect.right(), int(y))
        painter.drawLine(rect.center().x(), rect.top(), rect.center().x(), rect.bottom())

        values = [
            value
            for name, (samples, _color) in self.series.items()
            if self.visible[name]
            for value in samples
        ]
        limit = max(1.0, max((abs(value) for value in values), default=1.0) * 1.15)
        painter.setPen(QColor("#71849a"))
        painter.drawText(4, rect.top() + 5, f"+{limit:.0f} {self.unit}")
        painter.drawText(12, rect.center().y() + 5, f"0 {self.unit}")
        painter.drawText(4, rect.bottom(), f"-{limit:.0f} {self.unit}")

        for name, (samples, color) in self.series.items():
            if not self.visible[name] or len(samples) < 2:
                continue
            points = []
            for index, value in enumerate(samples):
                x = rect.left() + index * rect.width() / (samples.maxlen - 1)
                y = rect.center().y() - value / limit * rect.height() / 2
                points.append(QPointF(x, y))
            painter.setPen(QPen(color, 2))
            painter.drawPolyline(QPolygonF(points))


class AttitudePreview3D(QWidget):
    """A perspective preview driven only by the available Pitch and Roll."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(360)
        self.pitch = 0.0
        self.roll = 0.0

    def set_attitude(self, pitch: float, roll: float):
        self.pitch = pitch
        self.roll = roll
        self.update()

    def _transform(self, x, y, z):
        # The sensor frame is rotated 90 degrees from the board artwork:
        # visual_pitch = device_roll, visual_roll = -device_pitch.
        pitch = radians(self.roll)
        roll = radians(-self.pitch)
        y_pitch = y * cos(pitch) - z * sin(pitch)
        z_pitch = y * sin(pitch) + z * cos(pitch)
        x_roll = x * cos(roll) + z_pitch * sin(roll)
        z_roll = -x * sin(roll) + z_pitch * cos(roll)
        return x_roll, y_pitch, z_roll

    def _project(self, x, y, z, rect):
        x_roll, y_pitch, z_roll = self._transform(x, y, z)
        perspective = 1.0 / max(0.55, 1.0 - z_roll * 0.18)
        scale = min(rect.width() / 4.4, rect.height() / 3.0)
        return QPointF(rect.center().x() + x_roll * scale * perspective,
                       rect.center().y() - y_pitch * scale * perspective)

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#f8fbff"))
        rect = self.rect().adjusted(18, 18, -18, -18)
        painter.setPen(QPen(QColor("#cbdbea"), 1))
        painter.setBrush(QColor("#ffffff"))
        painter.drawRoundedRect(rect, 10, 10)

        horizon_y = rect.center().y()
        painter.setPen(QPen(QColor("#d8e6f3"), 1, Qt.DashLine))
        painter.drawLine(rect.left() + 18, horizon_y, rect.right() - 18, horizon_y)
        painter.setPen(QColor("#6f8398"))
        painter.drawText(rect.left() + 16, rect.top() + 27, "3D 姿态预览")
        painter.setPen(QColor("#8a9caf"))
        painter.drawText(rect.right() - 100, rect.top() + 27, "Pitch / Roll")

        half_x, half_y, half_z = 1.18, 0.76, 0.38
        def face(points):
            polygon = QPolygonF([self._project(x, y, z, rect) for x, y, z in points])
            depth = sum(self._transform(x, y, z)[2] for x, y, z in points) / len(points)
            return depth, polygon

        # Distinct enclosure faces and depth ordering make the direction readable.
        faces = (
            (QColor("#174f78"), ((-half_x, -half_y, -half_z), (half_x, -half_y, -half_z),
                                   (half_x, half_y, -half_z), (-half_x, half_y, -half_z))),
            (QColor("#1d638f"), ((-half_x, -half_y, -half_z), (-half_x, -half_y, half_z),
                                   (-half_x, half_y, half_z), (-half_x, half_y, -half_z))),
            (QColor("#247eaf"), ((half_x, -half_y, -half_z), (half_x, -half_y, half_z),
                                  (half_x, half_y, half_z), (half_x, half_y, -half_z))),
            (QColor("#206891"), ((-half_x, -half_y, -half_z), (half_x, -half_y, -half_z),
                                   (half_x, -half_y, half_z), (-half_x, -half_y, half_z))),
            (QColor("#4d9ed0"), ((-half_x, half_y, -half_z), (half_x, half_y, -half_z),
                                  (half_x, half_y, half_z), (-half_x, half_y, half_z))),
            (QColor("#d7eaf5"), ((-half_x, -half_y, half_z), (half_x, -half_y, half_z),
                                  (half_x, half_y, half_z), (-half_x, half_y, half_z))),
        )
        painter.setPen(QPen(QColor("#0b3f62"), 2))
        for color, points in sorted(faces, key=lambda item: face(item[1])[0]):
            painter.setBrush(color)
            painter.drawPolygon(face(points)[1])

        def draw_axis(name, vector, color):
            origin = self._project(0.0, 0.0, half_z + 0.03, rect)
            end = self._project(vector[0], vector[1], half_z + 0.03 + vector[2], rect)
            painter.setPen(QPen(color, 3))
            painter.drawLine(origin, end)
            dx, dy = end.x() - origin.x(), end.y() - origin.y()
            length = max(1.0, (dx * dx + dy * dy) ** 0.5)
            nx, ny = -dy / length, dx / length
            back_x, back_y = end.x() - dx / length * 10, end.y() - dy / length * 10
            painter.setBrush(color)
            painter.drawPolygon(QPolygonF([
                end, QPointF(back_x + nx * 4, back_y + ny * 4),
                QPointF(back_x - nx * 4, back_y - ny * 4),
            ]))
            painter.setPen(color)
            painter.drawText(end.x() + 5, end.y() - 4, name)

        # Sensor-frame axes: X red, Y green, Z blue.
        draw_axis("X", (0.82, 0.0, 0.0), QColor("#df4a4a"))
        draw_axis("Y", (0.0, 0.72, 0.0), QColor("#36a96d"))
        draw_axis("Z", (0.0, 0.0, 0.72), QColor("#2d77d1"))

        painter.setPen(QColor("#31516c"))
        painter.drawText(rect.left() + 16, rect.bottom() - 18,
                         f"Pitch  {self.pitch:+.2f}°")
        painter.drawText(rect.center().x() - 45, rect.bottom() - 18,
                         f"Roll  {self.roll:+.2f}°")
        painter.setPen(QColor("#7d91a5"))
        painter.drawText(rect.right() - 158, rect.bottom() - 18,
                         "Yaw unavailable")
