import sys
from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from app.ui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Solar IMU Tool")
    theme = Path(__file__).resolve().parent / "ui" / "theme.qss"
    if theme.exists():
        app.setStyleSheet(theme.read_text(encoding="utf-8"))
    window = MainWindow()
    window.show()
    # 发布包自动验收入口：只有Qt、主题、全部页面和串口模块均成功加载后，
    # 才会进入事件循环并在短延时后以0退出。普通用户启动不受影响。
    if "--smoke-test" in sys.argv:
        QTimer.singleShot(750, app.quit)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
