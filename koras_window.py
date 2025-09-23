from PySide6.QtCore import QFile, QTimer
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import (QApplication, QLabel, QLineEdit, QPushButton, QRadioButton)

from console_colors import BLUE, GREEN, NC, RED, YELLOW
from koras import KORAS

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_TYPE = "gripper"
DEFAULT_VELOCITY = "50"
DEFAULT_FORCE = "50"


class KORASWindow:

    def __init__(self):
        # --- Load UI ---
        loader = QUiLoader()
        ui_file = QFile("koras_window.ui")
        ui_file.open(QFile.ReadOnly)
        self.window = loader.load(ui_file)
        ui_file.close()

        # --- Initialize UI ---
        self.gripper = KORAS()

        # --- Bind widgets ---
        self.edit_port: QLineEdit = self.window.findChild(QLineEdit, "edit_port")
        self.radio_gripper: QRadioButton = self.window.findChild(QRadioButton, "radio_gripper")
        self.radio_vacuum: QRadioButton = self.window.findChild(QRadioButton, "radio_vacuum")
        self.label_cur_status: QLabel = self.window.findChild(QLabel, "label_cur_status")
        self.label_cur_pose: QLabel = self.window.findChild(QLabel, "label_cur_pose")
        self.label_distance_range: QLabel = self.window.findChild(QLabel, "label_distance_range")
        self.edit_velocity: QLineEdit = self.window.findChild(QLineEdit, "edit_velocity")
        self.edit_force: QLineEdit = self.window.findChild(QLineEdit, "edit_force")

        self.btn_connect: QPushButton = self.window.findChild(QPushButton, "btn_connect")
        self.btn_option: QPushButton = self.window.findChild(QPushButton, "btn_option")
        self.btn_open: QPushButton = self.window.findChild(QPushButton, "btn_open")
        self.btn_close: QPushButton = self.window.findChild(QPushButton, "btn_close")

        # --- Init UI defaults ---
        if self.edit_port and not self.edit_port.text().strip():
            self.edit_port.setText(DEFAULT_PORT)
        if self.label_distance_range:
            self.label_distance_range.setText(f"[ 0 ~ 100 ] %")
        if self.edit_force and not self.edit_force.text().strip():
            self.edit_force.setText(DEFAULT_FORCE)
        if self.edit_velocity and not self.edit_velocity.text().strip():
            self.edit_velocity.setText(DEFAULT_VELOCITY)

        # --- Wire signals ---
        if self.btn_connect:
            self.btn_connect.clicked.connect(self.on_connect)
        if self.btn_option:
            self.btn_option.clicked.connect(self.on_apply_options)
        if self.btn_open:
            self.btn_open.clicked.connect(self.on_open)
        if self.btn_close:
            self.btn_close.clicked.connect(self.on_close)

        # --- Periodic UI refresh ---
        self.timer = QTimer(self.window)
        self.timer.setInterval(100)  # ms
        self.timer.timeout.connect(self.refresh_ui)
        self.timer.start()

        # --- Cleanup on app exit ---
        QApplication.instance().aboutToQuit.connect(self.gripper.disconnect)

        # Initial state text
        self.set_status("IDLE")

    # ------------------- Slots -------------------

    def on_connect(self):
        """Connect/Disconnect to device"""
        if self.gripper.connected:
            self.gripper.disconnect()
        else:
            port = self.edit_port.text().strip() if self.edit_port else DEFAULT_PORT
            port = int(self.edit_port.text()) if self.edit_port and self.edit_port.text() else DEFAULT_PORT

            self.gripper.connect(port_name=port, slave_id=1)

            if self.gripper.connected:
                self.gripper.init()

    def on_apply_options(self):
        """Apply velocity/force"""
        vel = int(self.edit_velocity.text()) if self.edit_velocity and self.edit_velocity.text() else 50
        frc = int(self.edit_force.text()) if self.edit_force and self.edit_force.text() else 50

        # clamp 0~100
        vel = max(0, min(100, vel))
        frc = max(0, min(100, frc))

        self.gripper.opt_velocity(vel)
        self.gripper.opt_force(frc)

    def on_open(self):
        """Open gripper fully"""
        # release(-1) uses default fully-open profile in your code
        if self.radio_gripper.isChecked():
            self.gripper.release(release_distance=-1)  # -1 means fully open

        elif self.radio_vacuum.isChecked():
            self.gripper.vacuum(vacuum=False)

    def on_close(self):
        """Close gripper"""
        # grip(-1) uses default/internal max profile
        if self.radio_gripper.isChecked():
            self.gripper.grip(grip_distance=-1)  # -1 means fully close

        elif self.radio_vacuum.isChecked():
            self.gripper.vacuum(vacuum=True)

    def refresh_ui(self):
        """Poll position/init and update labels"""

        # Connect status
        if self.gripper.connected:
            self.btn_connect.setText("DISCONNECT")
            self.btn_connect.setStyleSheet("font-size: 8px; font-weight: bold;")
        else:
            self.btn_connect.setText("CONNECT")
            self.btn_connect.setStyleSheet("font-size: 12px; font-weight: bold;")

        # position from counts to % (only for gripper)
        pos_counts = self.gripper.get_position()
        if isinstance(pos_counts, int) and self.label_cur_pose:
            self.label_cur_pose.setText(f"{pos_counts/10.0:.1f} %")

        # show INIT state
        if self.gripper.get_status() and self.label_cur_status:
            if self.gripper.get_status():
                self.set_status("CLOSE")
            else:
                self.set_status("OPEN")

    def set_status(self, text: str):
        """Set status label"""
        if self.label_cur_status:
            self.label_cur_status.setText(text)


if __name__ == "__main__":
    # --- Create PySide6 application ---
    app = QApplication([])

    # --- Create GUI control window ---
    koras_window = KORASWindow()

    # --- Show UI ---
    koras_window.window.show()

    # --- Start application ---
    app.exec()
