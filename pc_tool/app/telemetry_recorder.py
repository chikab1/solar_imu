import csv
from datetime import datetime
from pathlib import Path


class TelemetryRecorder:
    """Append timestamped telemetry rows to a per-day CSV file."""

    def __init__(self, category, fieldnames):
        self.category = category
        self.fieldnames = tuple(fieldnames)
        if not self.fieldnames or self.fieldnames[0] != "timestamp":
            raise ValueError("CSV首列必须是timestamp")
        self.directory = None
        self._date_key = None
        self._file = None
        self._writer = None
        self.path = None
        self.last_error = ""

    @property
    def is_recording(self):
        return self._file is not None

    def start(self, directory, now=None):
        target = Path(directory).expanduser()
        if not target.is_dir():
            raise ValueError("保存目录不存在或不可用")
        self.stop()
        self.directory = target
        self._open_for(now or datetime.now().astimezone())
        return self.path

    def write(self, values, now=None):
        if not self.is_recording:
            return None
        moment = now or datetime.now().astimezone()
        date_key = moment.strftime("%Y%m%d")
        if date_key != self._date_key:
            self._open_for(moment)
        row = {name: values.get(name, "") for name in self.fieldnames}
        row["timestamp"] = moment.isoformat(timespec="milliseconds")
        try:
            self._writer.writerow(row)
            self._file.flush()
        except OSError as exc:
            self.last_error = str(exc)
            self.stop()
            raise
        return self.path

    def stop(self):
        if self._file is not None:
            self._file.flush()
            self._file.close()
        self._file = None
        self._writer = None
        self._date_key = None

    def _open_for(self, moment):
        self.stop()
        self._date_key = moment.strftime("%Y%m%d")
        self.path = self.directory / f"{self.category}_{self._date_key}.csv"
        write_header = not self.path.exists() or self.path.stat().st_size == 0
        self._file = self.path.open("a", encoding="utf-8", newline="")
        if write_header:
            self._file.write(chr(0xFEFF))
        self._writer = csv.DictWriter(self._file, fieldnames=self.fieldnames)
        if write_header:
            self._writer.writeheader()
            self._file.flush()
