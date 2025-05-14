import sys
import sqlite3
import asyncio
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QHBoxLayout, QVBoxLayout, QLabel, QProgressBar,
    QLineEdit, QPushButton, QListWidget, QMessageBox,
    QDialog, QSlider
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from bleak import BleakScanner, BleakClient

# BLE / Database configuration
TARGET_NAME       = "beatTooth"
SONG_NAME_UUID    = "12345678-1234-1234-1234-1234567890ac"
SCORE_CHAR_UUID   = "12345678-1234-1234-1234-1234567890ad"
TIME_CHAR_UUID    = "12345678-1234-1234-1234-1234567890ae"
TOTAL_TIME_UUID   = "12345678-1234-1234-1234-1234567890af"
FORCE_CHAR_UUID   = "12345678-1234-1234-1234-1234567890b0"
SPEED_CHAR_UUID   = "12345678-1234-1234-1234-1234567890b1"
DB_PATH           = "leaderboard.db"

# Initialize SQLite DB & table
conn = sqlite3.connect(DB_PATH)
c = conn.cursor()
c.execute("""
CREATE TABLE IF NOT EXISTS scores (
    id    INTEGER PRIMARY KEY AUTOINCREMENT,
    name  TEXT    NOT NULL,
    score INTEGER NOT NULL,
    ts    DATETIME DEFAULT CURRENT_TIMESTAMP
)
""")
conn.commit()

class BLEWorker(QThread):
    connection_changed   = pyqtSignal(bool)
    song_name_updated    = pyqtSignal(str)
    score_updated        = pyqtSignal(int)
    time_updated         = pyqtSignal(int)
    total_time_updated   = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = True
        self.client = None

    def run(self):
        # create a fresh loop for this thread
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        # now run your main loop on it
        self.loop.run_until_complete(self._ble_loop())

    async def _ble_loop(self):
        prev_state = False
        while self._running:
            devices = await BleakScanner.discover(timeout=5.0, return_adv=True)
            target = None
            for _, (dev, adv) in devices.items():
                name = adv.local_name or dev.name
                if name == TARGET_NAME:
                    target = dev
                    break
            if not target:
                if prev_state:
                    self.connection_changed.emit(False)
                    prev_state = False
                await asyncio.sleep(1)
                continue
            try:
                client = BleakClient(target.address)
                await client.connect()
                self.client = client
                if client.is_connected:
                    if not prev_state:
                        self.connection_changed.emit(True)
                        prev_state = True
                await client.start_notify(SONG_NAME_UUID, self._on_song_name)
                await client.start_notify(SCORE_CHAR_UUID, self._on_score)
                await client.start_notify(TIME_CHAR_UUID, self._on_time)
                await client.start_notify(TOTAL_TIME_UUID, self._on_total_time)
                while client.is_connected and self._running:
                    await asyncio.sleep(1)
                await client.disconnect()
            except Exception as e:
                print(f"BLE error: {e}")
                if prev_state:
                    self.connection_changed.emit(False)
                    prev_state = False
            await asyncio.sleep(1)

    def _on_song_name(self, sender, data):
        text = bytes(data).decode('utf-8').rstrip('\x00')
        self.song_name_updated.emit(text)

    def _on_score(self, sender, data):
        score = int.from_bytes(data, 'little')
        self.score_updated.emit(score)

    def _on_time(self, sender, data):
        t = int.from_bytes(data, 'little')
        self.time_updated.emit(t)

    def _on_total_time(self, sender, data):
        tot = int.from_bytes(data, 'little')
        self.total_time_updated.emit(tot)

    def stop(self):
        self._running = False
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BeatTooth")
        self.resize(800, 400)

        central = QWidget()
        self.setCentralWidget(central)
        h_layout = QHBoxLayout(central)

        # Left pane
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(10, 10, 10, 10)

        # Connection indicator (red/green)
        self.conn_indicator = QLabel()
        self.conn_indicator.setFixedSize(16, 16)
        self.conn_indicator.setStyleSheet(
            "background-color: red; border-radius: 8px;"
        )
        left_layout.addWidget(self.conn_indicator, alignment=Qt.AlignLeft)

        # Settings button
        self.settings_button = QPushButton("Settings")
        self.settings_button.setStyleSheet("font-size: 14pt;")
        self.settings_button.clicked.connect(self.open_settings)
        left_layout.addWidget(self.settings_button, alignment=Qt.AlignLeft)

        # Now Playing label
        self.title_song = QLabel("Now Playing:")
        self.title_song.setStyleSheet("font-size: 18pt; font-weight: bold;")
        left_layout.addWidget(self.title_song)

        # Song name
        self.song_label = QLabel("--")
        self.song_label.setStyleSheet("font-size: 24pt;")
        left_layout.addWidget(self.song_label)

        # Progress bar
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        left_layout.addWidget(self.progress)

        # Score label
        self.title_score = QLabel("Score:")
        self.title_score.setStyleSheet("font-size: 18pt; font-weight: bold;")
        left_layout.addWidget(self.title_score)

        self.score_label = QLabel("0")
        self.score_label.setStyleSheet("font-size: 48pt;")
        left_layout.addWidget(self.score_label)

        # Name entry + Done button
        self.name_entry = QLineEdit()
        self.name_entry.setPlaceholderText("Enter your name...")
        self.name_entry.setMaxLength(20)
        self.name_entry.setEnabled(False)
        left_layout.addWidget(self.name_entry)

        self.done_button = QPushButton("Done")
        self.done_button.setStyleSheet("font-size: 14pt;")
        self.done_button.clicked.connect(self.save_score)
        self.done_button.setEnabled(False)
        left_layout.addWidget(self.done_button)

        h_layout.addWidget(left_widget, stretch=1)

        # Right pane: Leaderboard
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(10, 10, 10, 10)

        self.title_lb = QLabel("Leaderboard")
        self.title_lb.setStyleSheet("font-size: 18pt; font-weight: bold;")
        right_layout.addWidget(self.title_lb)

        self.leader_list = QListWidget()
        self.leader_list.setStyleSheet("font-size: 14pt;")
        right_layout.addWidget(self.leader_list)

        h_layout.addWidget(right_widget, stretch=1)

        # BLE Worker
        self.worker = BLEWorker()
        self.worker.connection_changed.connect(self.on_connection_changed)
        self.worker.song_name_updated.connect(self.on_song_name)
        self.worker.score_updated.connect(self.on_score)
        self.worker.time_updated.connect(self.on_time)
        self.worker.total_time_updated.connect(self.on_total_time)
        self.worker.start()

        self.current_score = 0
        self.update_leaderboard()

    # — Slots for BLEWorker signals —

    def on_connection_changed(self, connected):
        color = "green" if connected else "red"
        self.conn_indicator.setStyleSheet(
            f"background-color: {color}; border-radius: 8px;"
        )

    def on_song_name(self, name):
        self.song_label.setText(name)
        self.score_label.setText("0")
        self.progress.setValue(0)
        self.title_song.setText("Now Playing:")
        self.name_entry.setEnabled(False)
        self.done_button.setEnabled(False)

    def on_score(self, score):
        self.current_score = score
        self.score_label.setText(str(score))
        self.title_song.setText("Enter your name:")
        self.name_entry.setEnabled(True)
        self.done_button.setEnabled(True)

    def on_time(self, t):
        self.progress.setValue(t)

    def on_total_time(self, tot):
        self.progress.setRange(0, tot)

    # — Settings dialog —

    def open_settings(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Settings")
        layout = QVBoxLayout(dialog)

        layout.addWidget(QLabel("Force"))
        self.force_slider = QSlider(Qt.Horizontal)
        self.force_slider.setRange(0, 2)
        self.force_slider.setValue(1)
        self.force_slider.setTickPosition(QSlider.TicksBelow)
        self.force_slider.setTickInterval(1)
        self.force_slider.valueChanged.connect(self.update_force_label)
        layout.addWidget(self.force_slider)
        self.force_value_label = QLabel("Normal")
        layout.addWidget(self.force_value_label)

        layout.addWidget(QLabel("Speed"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(0, 2)
        self.speed_slider.setValue(1)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.speed_slider.setTickInterval(1)
        self.speed_slider.valueChanged.connect(self.update_speed_label)
        layout.addWidget(self.speed_slider)
        self.speed_value_label = QLabel("Normal")
        layout.addWidget(self.speed_value_label)

        save_btn = QPushButton("Save")
        save_btn.clicked.connect(lambda: self.save_settings(dialog))
        layout.addWidget(save_btn)

        dialog.exec_()

    def update_force_label(self, v):
        self.force_value_label.setText({0: "Soft", 1: "Normal", 2: "Hard"}[v])

    def update_speed_label(self, v):
        self.speed_value_label.setText({0: "Slow", 1: "Normal", 2: "Fast"}[v])

    # — Write settings to the already-open BLE connection —

    def save_settings(self, dialog):
        force = self.force_slider.value()
        speed = self.speed_slider.value()
        dialog.accept()

        client = self.worker.client
        if not client:
            QMessageBox.warning(self, "Error", "Still scanning for device…")
            return

        # schedule the MainWindow's coroutine on the BLEWorker's loop
        future = asyncio.run_coroutine_threadsafe(
            self._write_settings(client, force, speed),
            self.worker.loop
        )

        try:
            # wait up to 5 seconds
            result = future.result(timeout=5)
            QMessageBox.information(self, "Settings", "Settings saved")
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to write: {e}")

    async def _write_settings(self, client: BleakClient, force: int, speed: int):
        await client.write_gatt_char(FORCE_CHAR_UUID, bytearray([force]))
        await client.write_gatt_char(SPEED_CHAR_UUID, bytearray([speed]))

    # — Score saving & leaderboard —

    def save_score(self):
        name = self.name_entry.text().strip()
        if not name:
            QMessageBox.warning(self, "Missing Name",
                                "Please enter your name before saving.")
            return
        c.execute(
            "INSERT INTO scores (name, score) VALUES (?, ?)",
            (name, self.current_score)
        )
        conn.commit()
        self.name_entry.clear()
        self.name_entry.setEnabled(False)
        self.done_button.setEnabled(False)
        self.update_leaderboard()

    def update_leaderboard(self):
        self.leader_list.clear()
        c.execute(
            "SELECT name, score FROM scores "
            "ORDER BY score DESC, ts ASC LIMIT 10"
        )
        for idx, (name, score) in enumerate(c.fetchall(), start=1):
            self.leader_list.addItem(f"{idx}. {name} — {score}")

    def closeEvent(self, event):
        self.worker.stop()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
