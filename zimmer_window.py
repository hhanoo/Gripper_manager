import json
import os
import signal
import sys
import threading

from PySide6.QtCore import QFile, Qt, QTimer
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import (QApplication, QCheckBox, QGroupBox, QLabel, QLineEdit, QPushButton, QSpinBox)

from console_colors import BLUE, GREEN, NC, RED, YELLOW
from zimmer import Zimmer


class ZimmerWindow:

    def __init__(self):
        # --- Load UI ---
        loader = QUiLoader()
        ui_file = QFile("zimmer_window.ui")
        ui_file.open(QFile.ReadOnly)
        self.window = loader.load(ui_file)
        ui_file.close()

        # --- Initialize UI ---
        self.gripper = Zimmer()

        # --- Bind widgets ---
        # Connection
        self.edit_ip: QLineEdit = self.window.findChild(QLineEdit, "edit_ip")
        self.edit_port: QLineEdit = self.window.findChild(QLineEdit, "edit_port")
        self.btn_connect: QPushButton = self.window.findChild(QPushButton, "btn_connect")
        self.btn_disconnect: QPushButton = self.window.findChild(QPushButton, "btn_disconnect")
        # Current
        self.label_cur_status: QLabel = self.window.findChild(QLabel, "label_cur_status")
        self.label_base_position: QLabel = self.window.findChild(QLabel, "label_base_position")
        self.label_shift_position: QLabel = self.window.findChild(QLabel, "label_shift_position")
        self.label_work_position: QLabel = self.window.findChild(QLabel, "label_work_position")
        self.label_cur_position: QLabel = self.window.findChild(QLabel, "label_cur_position")
        self.label_distance_range: QLabel = self.window.findChild(QLabel, "label_distance_range")
        # Simple Operation
        self.btn_open: QPushButton = self.window.findChild(QPushButton, "btn_open")
        self.btn_close: QPushButton = self.window.findChild(QPushButton, "btn_close")
        # Complex Operation
        self.spinBox_jaw_gap: QSpinBox = self.window.findChild(QSpinBox, "spinBox_jaw_gap")
        self.spinBox_force: QSpinBox = self.window.findChild(QSpinBox, "spinBox_force")
        self.spinBox_velocity: QSpinBox = self.window.findChild(QSpinBox, "spinBox_velocity")
        self.btn_complex_operation: QPushButton = self.window.findChild(QPushButton, "btn_complex_operation")
        # Homing Operation
        self.checkBox_homing: QCheckBox = self.window.findChild(QCheckBox, "checkBox_homing")
        self.btn_inside_homing: QPushButton = self.window.findChild(QPushButton, "btn_inside_homing")
        self.btn_outside_homing: QPushButton = self.window.findChild(QPushButton, "btn_outside_homing")
        # Status Word
        self.status_word_Box: QGroupBox = self.window.findChild(QGroupBox, "gridGroupBox_5")
        self.label_status_bit_list = []
        for i in range(16):
            self.label_status_bit_list.append(self.window.findChild(QLabel, f"label_status_bit_{i}"))
        # Error
        self.label_error_code: QLabel = self.window.findChild(QLabel, "label_error_code")
        self.label_error_msg: QLabel = self.window.findChild(QLabel, "label_error_msg")

        # --- Load JSON file ---
        if os.path.exists("zimmer_config.json"):
            with open("zimmer_config.json", "r") as f:
                self.config = json.load(f)
        else:
            self.config = {"ip": "", "port": "502", "jaw_gap": "100", "velocity": "50", "force": "50"}

        # --- Init UI defaults ---
        if self.edit_ip and not self.edit_ip.text().strip():
            self.edit_ip.setText(self.config["ip"])
        if self.edit_port and not self.edit_port.text().strip():
            self.edit_port.setText(self.config["port"])
        if self.label_distance_range:
            max_mm = (self.gripper.gripper_gap_maximum - 100) / 100.0  # counts(0.01mm) -> mm
            self.label_distance_range.setText(f"[ 0 ~ {max_mm:.2f} ] mm")
        if self.spinBox_jaw_gap:
            self.spinBox_jaw_gap.setValue(int(self.config["jaw_gap"]))
            self.spinBox_jaw_gap.setRange(100, (self.gripper.gripper_gap_maximum - 100) * 2)
        if self.spinBox_force:
            self.spinBox_force.setValue(int(self.config["force"]))
        if self.spinBox_velocity:
            self.spinBox_velocity.setValue(int(self.config["velocity"]))

        # --- Wire signals ---
        if self.btn_connect:
            self.btn_connect.clicked.connect(self.on_connect)
        if self.btn_disconnect:
            self.btn_disconnect.clicked.connect(self.on_connect)
        if self.btn_open:
            self.btn_open.clicked.connect(self.on_open)
        if self.btn_close:
            self.btn_close.clicked.connect(self.on_close)
        if self.btn_complex_operation:
            self.btn_complex_operation.clicked.connect(self.on_complex_operation)
        if self.checkBox_homing:
            self.checkBox_homing.stateChanged.connect(self.on_homing_state_changed)
        if self.btn_outside_homing:
            self.btn_outside_homing.clicked.connect(self.on_outside_homing)
        if self.btn_inside_homing:
            self.btn_inside_homing.clicked.connect(self.on_inside_homing)

        # --- Periodic UI refresh ---
        self.timer = QTimer(self.window)
        self.timer.setInterval(100)  # ms
        self.timer.timeout.connect(self.refresh_ui)
        self.timer.start()

        # --- Cleanup on app exit ---
        QApplication.instance().aboutToQuit.connect(self.gripper.disconnect)

        # Initial state text
        self.set_status("IDLE")

    # ------------------- Safe shutdown -------------------

    def shutdown(self):
        """Safe shutdown sequence"""
        if hasattr(self, "timer"):
            self.timer.stop()  # stop timer
        self.gripper.disconnect()  # disconnect hardware

    def closeEvent(self, event):
        self.shutdown()
        event.accept()

    # ------------------- Slots -------------------

    def on_connect(self):
        """Connect/Disconnect to device"""
        if self.gripper.connected:
            self.gripper.disconnect()
        else:
            ip = self.edit_ip.text().strip()
            port = int(self.edit_port.text())
            print(f"Connecting to {ip}:{port}")
            self.gripper.connect(ip=ip, port=port)

            if self.gripper.connected:
                self.config["ip"] = ip
                self.config["port"] = str(port)
                with open("zimmer_config.json", "w") as f:
                    json.dump(self.config, f)

                init_thread = threading.Thread(target=self.gripper.init, daemon=True)
                init_thread.start()

    def on_open(self):
        """Open gripper fully"""
        # release(-1) uses default fully-open profile in your code
        self.gripper.release(jaw_gap=-1, sync=False)

    def on_close(self):
        """Close gripper"""
        # grip(-1) uses default/internal max profile
        self.gripper.grip(jaw_gap=-1, sync=False)

    def on_complex_operation(self):
        """Complex operation"""
        jaw_gap = self.spinBox_jaw_gap.value()
        velocity = self.spinBox_velocity.value()
        force = self.spinBox_force.value()

        self.gripper.opt_velocity(velocity)
        self.gripper.opt_force(force)
        self.gripper.custom_position(jaw_gap, sync=False)

        self.config["jaw_gap"] = str(jaw_gap)
        self.config["velocity"] = str(velocity)
        self.config["force"] = str(force)
        with open("zimmer_config.json", "w") as f:
            json.dump(self.config, f)

    def on_homing_state_changed(self):
        """Homing state changed"""
        if self.checkBox_homing.checkState() == Qt.Checked:
            self.btn_outside_homing.setEnabled(True)
            self.btn_inside_homing.setEnabled(True)
        else:
            self.btn_outside_homing.setEnabled(False)
            self.btn_inside_homing.setEnabled(False)

    def on_outside_homing(self):
        """Outside homing"""
        self.gripper.outside_homing()

    def on_inside_homing(self):
        """Inside homing"""
        self.gripper.inside_homing()

    def refresh_ui(self):
        """Poll position/init and update labels"""

        # Connect status
        if self.gripper.connected:
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)

            status_word, status_word_bits = self.gripper.get_status_word()
            base_position = self.gripper.get_base_position()
            shift_position = self.gripper.get_shift_position()
            work_position = self.gripper.get_work_position()
            actual_position = self.gripper.get_actual_position()

            # Show Status
            inMotion = bool(status_word_bits[15 - 2])
            movementComplete = bool(status_word_bits[15 - 3])
            atBaseposition = bool(status_word_bits[15 - 8])
            atWorkposition = bool(status_word_bits[15 - 10])
            if inMotion and not movementComplete:
                self.set_status("MOVING")
            elif not inMotion and movementComplete:
                if atBaseposition:
                    self.set_status("OPEN")
                elif atWorkposition:
                    self.set_status("CLOSE")
            else:
                self.set_status("-")

            # Show Positions
            if isinstance(base_position, int) and self.label_base_position:
                self.label_base_position.setText(f"{base_position/100.0:.2f} mm")
            if isinstance(shift_position, int) and self.label_shift_position:
                self.label_shift_position.setText(f"{shift_position/100.0:.2f} mm")
            if isinstance(work_position, int) and self.label_work_position:
                self.label_work_position.setText(f"{work_position/100.0:.2f} mm")
            if isinstance(actual_position, int) and self.label_cur_position:
                self.label_cur_position.setText(f"{actual_position/100.0:.2f} mm")

            # show status word bits
            self.status_word_Box.setTitle(f"Status Word (0x{status_word:04X})")
            for i in range(16):
                self.label_status_bit_list[i].setText(f"{status_word_bits[15-i]}")

            # show error code
            error_code, error_msg = self.gripper.get_diagnosis()
            self.label_error_code.setText(f"0x{error_code:04X} ({error_code})")
            self.label_error_msg.setText(error_msg)
        else:
            self.set_status("DISCONNECTED")
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)

    def set_status(self, text: str):
        """Set status label"""
        if self.label_cur_status:
            self.label_cur_status.setText(text)


if __name__ == "__main__":
    # --- Create PySide6 application ---
    app = QApplication([])

    # --- Create GUI control window ---
    zimmer_window = ZimmerWindow()

    # --- Show UI ---
    zimmer_window.window.show()

    # --- Handle SIGINT ---
    def handle_sigint(sig, frame):
        zimmer_window.shutdown()
        QApplication.quit()

    signal.signal(signal.SIGINT, handle_sigint)

    # --- Start application ---
    zimmer_window.window.show()
    sys.exit(app.exec())
