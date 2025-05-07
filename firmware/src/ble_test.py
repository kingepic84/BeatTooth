import sys
import time
import sqlite3
import asyncio
import threading
from bleak import BleakScanner, BleakClient
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QHBoxLayout, QVBoxLayout, QLabel,
    QLineEdit, QPushButton, QListWidget, QMessageBox
)
from PyQt5.QtCore import QThread, pyqtSignal

# ────────────────────────────────────────────────────────────────────────────────
# BLE / Database configuration
# ────────────────────────────────────────────────────────────────────────────────
TARGET_NAME     = "beatTooth"
SCORE_CHAR_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214"
DB_PATH         = "leaderboard.db"

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
"""
)
conn.commit()

async def read_dance_score():
    """Scan, connect to BLE peripheral, and return its current dance score."""
    devices = await BleakScanner.discover(timeout=5.0, return_adv=True)
    for _, (dev, adv) in devices.items():
        name = adv.local_name or dev.name
        if name == TARGET_NAME:
            async with BleakClient(dev.address) as client:
                if not client.is_connected:
                    raise RuntimeError("Failed to connect to device")
                data = await client.read_gatt_char(SCORE_CHAR_UUID)
                return int(data[0])
    raise RuntimeError(f"Device '{TARGET_NAME}' not found")

class BLEWorker(QThread):
    score_updated = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = True

    def run(self):
        while self._running:
            try:
                score = asyncio.run(read_dance_score())
            except Exception:
                score = None
            if score is not None:
                self.score_updated.emit(score)
            time.sleep(2)

    def stop(self):
        self._running = False
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BeatTooth")
        self.resize(800, 400)

        # Central widget
        central = QWidget()
        self.setCentralWidget(central)

        # Layouts
        h_layout = QHBoxLayout(central)

        # Left pane for current score
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(10, 10, 10, 10)

        self.title_score = QLabel("Current Score")
        self.title_score.setStyleSheet("font-size: 18pt; font-weight: bold;")
        left_layout.addWidget(self.title_score)

        self.score_label = QLabel("--")
        self.score_label.setStyleSheet("font-size: 48pt;")
        left_layout.addWidget(self.score_label, stretch=1)

        self.name_entry = QLineEdit()
        self.name_entry.setPlaceholderText("Enter your name...")
        left_layout.addWidget(self.name_entry)

        self.done_button = QPushButton("Done")
        self.done_button.setStyleSheet("font-size: 14pt;")
        self.done_button.clicked.connect(self.save_score)
        left_layout.addWidget(self.done_button)

        h_layout.addWidget(left_widget, stretch=1)

        # Right pane for leaderboard
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(10, 10, 10, 10)

        self.title_lb = QLabel("Leaderboard")
        self.title_lb.setStyleSheet("font-size: 18pt; font-weight: bold;")
        right_layout.addWidget(self.title_lb)

        self.leader_list = QListWidget()
        self.leader_list.setStyleSheet("font-size: 14pt;")
        right_layout.addWidget(self.leader_list, stretch=1)

        h_layout.addWidget(right_widget, stretch=1)

        # BLE Thread
        self.worker = BLEWorker()
        self.worker.score_updated.connect(self.update_score)
        self.worker.start()

        # State
        self.current_score = 0
        self.update_leaderboard()

    def update_score(self, score):
        self.current_score = score
        self.score_label.setText(str(score))

    def save_score(self):
        name = self.name_entry.text().strip()
        if not name:
            QMessageBox.warning(self, "Missing Name", "Please enter your name before saving.")
            return
        c.execute("INSERT INTO scores (name, score) VALUES (?, ?)", (name, self.current_score))
        conn.commit()
        self.name_entry.clear()
        self.update_leaderboard()

    def update_leaderboard(self):
        self.leader_list.clear()
        c.execute("SELECT name, score FROM scores ORDER BY score DESC, ts ASC LIMIT 10")
        rows = c.fetchall()
        for idx, (name, score) in enumerate(rows, 1):
            self.leader_list.addItem(f"{idx}. {name} — {score}")

    def closeEvent(self, event):
        self.worker.stop()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
