from PySide6.QtCore import QFile, QTimer
from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication, QLabel, QLineEdit, QPushButton

from console_colors import BLUE, GREEN, NC, RED, YELLOW
from keti_zimmer import KetiZimmer

DEFAULT_IP = '192.168.3.112'
DEFAULT_PORT = "502"
DEFAULT_VELOCITY = "50"
DEFAULT_FORCE = "50"


class MainWindow:

    def __init__(self):
        # --- Load UI ---
        loader = QUiLoader()
        ui_file = QFile("main_window.ui")
        ui_file.open(QFile.ReadOnly)
        self.window = loader.load(ui_file)
        ui_file.close()

        # --- Initialize UI ---
        self.gripper = KetiZimmer()

        # --- Bind widgets ---
        self.edit_ip: QLineEdit = self.window.findChild(QLineEdit, "edit_ip")
        self.edit_port: QLineEdit = self.window.findChild(QLineEdit, "edit_port")
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
        if self.edit_ip and not self.edit_ip.text().strip():
            self.edit_ip.setText(DEFAULT_IP)
        if self.edit_port and not self.edit_port.text().strip():
            self.edit_port.setText(DEFAULT_PORT)
        if self.label_distance_range:
            max_mm = self.gripper.gripper_max_distance / 100.0  # counts(0.01mm) -> mm
            self.label_distance_range.setText(f"[ 0 ~ {max_mm:.2f} ] mm")
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
            ip = self.edit_ip.text().strip() if self.edit_ip else DEFAULT_IP
            port = int(self.edit_port.text()) if self.edit_port and self.edit_port.text() else DEFAULT_PORT

            self.gripper.connect(ip=ip, port=port)

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
        self.gripper.release(release_distance=-1, sync=False)

    def on_close(self):
        """Close gripper"""
        # grip(-1) uses default/internal max profile
        self.gripper.grip(grip_distance=-1, sync=False)

    def refresh_ui(self):
        """Poll position/init and update labels"""

        # Connect status
        if self.gripper.connected:
            self.btn_connect.setText("DISCONNECT")
            self.btn_connect.setStyleSheet("font-size: 8px; font-weight: bold;")
        else:
            self.btn_connect.setText("CONNECT")
            self.btn_connect.setStyleSheet("font-size: 12px; font-weight: bold;")

        # position from counts to mm
        pos_counts = self.gripper.get_position()
        if isinstance(pos_counts, int) and self.label_cur_pose:
            self.label_cur_pose.setText(f"{pos_counts/100.0:.2f} mm")

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
    main_window = MainWindow()

    # --- Show UI ---
    main_window.window.show()

    # --- Start application ---
    app.exec()
