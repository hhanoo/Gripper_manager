# ============================================================
# KORAS Gripper Modbus RTU Configuration & Register Map
# ============================================================
#
# [RTU Configuration]
# - Baud rate : 38,400 bps
# - Data bit  : 8
# - Stop bit  : 1
# - Parity    : None ('N')
# - Slave ID  : 1 (default)
# - Port      : /dev/ttyUSB0
#
# [Output Data (Holding Registers: Address 0 ~ 9)]
# - Address 0 : Command (GrpCmd)
#     1   : Motor Enable
#     2   : Motor Stop
#     4   : Motor Disable
#     5   : Motor Position Control (degree)
#     6   : Motor Velocity Control (rpm)
#     7   : Motor Current Control (mA)
#     50  : Change Modbus Address
#     101 : Gripper Initialize
#     102 : Gripper Open
#     103 : Gripper Close
#     104 : Set Finger Position (0 ~ 1000, → 0% ~ 100%)
#     106 : Vacuum Gripper On
#     107 : Vacuum Gripper Off
#     212 : Set Motor Torque (50 ~ 100 %)
#     213 : Set Motor Speed (1 ~ 100 %)
#
# - Address 1 : Target value (명령에 따른 설정값)
# - Address 2~9 : Reserved / 확장용
#
# [Input Data (Input Registers: Address 10 ~ 17)]
# - Address 10 : Status (bit-coded)
#         Bit0 : Motor Enabled
#         Bit1 : Initialized (gripper init done)
#         Bit2 : Motor PosCtrl active
#         Bit3 : Motor VelCtrl active
#         Bit4 : Motor CurCtrl active
#         Bit5 : Gripper Opening (active)
#         Bit6 : Gripper Closing (active)
#         Bit9 : Motor Fault (error)
# - Address 11 : Motor Position [degree]
# - Address 12 : Motor Current [mA]
# - Address 13 : Motor Velocity [rpm]
# - Address 14 : Gripper Finger Position [0 ~ 1000]
# - Address 15 : Reserved
# - Address 16 : Reserved
# - Address 17 : Bus Voltage [V]
#
# ============================================================

import threading
import time

from pymodbus.client import ModbusSerialClient

from console_colors import BLUE, GREEN, NC, RED, YELLOW


# ------------------------------ constants (상수) ------------------------------
# yapf: disable
class GrpCmd:
    MotorEnable       = 1
    MotorStop         = 2
    MotorDisable      = 4
    MotorPosCtrl      = 5   # optional
    MotorVelCtrl      = 6   # optional
    MotorCurCtrl      = 7   # optional
    ChangeSlaveAddr   = 50  # optional

    Initialize        = 101
    Open              = 102
    Close             = 103
    FingerPosition    = 104
    VacuumOn          = 106
    VacuumOff         = 107
    SetMotorTorque    = 212
    SetMotorSpeed     = 213

class StatusBits:
    MotorEnable    = 1 << 0
    Initialize     = 1 << 1
    PosControl     = 1 << 2
    VelControl     = 1 << 3
    CurControl     = 1 << 4
    OpenActive     = 1 << 5
    CloseActive    = 1 << 6
    MotorFault     = 1 << 9

# yapf: enable
# ------------------------------------------------------------------------------


class KORAS:
    # Register map (레지스터 맵)
    WR_ADDR_CMD = 0  # holding write: command
    WR_ADDR_VAL = 1  # holding write: value
    RD_ADDR_BASE = 10  # holding read base
    RD_COUNT = 5  # read 10..14

    def __init__(self):
        """
        Initialize the KORAS class
        """

        #  RTU Config
        self.port_name = "/dev/ttyUSB0"
        self.baudrate = 38400
        self.data_bits = 8
        self.stop_bits = 1
        self.parity = "N"
        self.slave_id = 1

        #  Connection
        self.connected = False

        # Modbus client
        self.mb = ModbusSerialClient(method="rtu",
                                     port=self.port_name,
                                     baudrate=self.baudrate,
                                     stopbits=self.stop_bits,
                                     bytesize=self.data_bits,
                                     parity=self.parity,
                                     timeout=1)

        #  Threading
        self.run_action = False
        self.action_thread = None

        #  State / Info
        self.m_info = {
            "status_raw": 0,
            "status": {
                "motor_enable": False,
                "initialize": False,
                "pos_control": False,
                "vel_control": False,
                "cur_control": False,
                "is_open": False,
                "is_close": False,
                "motor_fault": False,
            },
            "position": 0,
            "current": 0,
            "velocity": 0,
            "grp_pos_percent": 0,
            "bus_voltage": 0,
        }
        self.mutex = threading.Lock()

        #  Motor flags
        self.power_on = False

    # ---------------- Connection ----------------
    def connect(self, port_name="/dev/ttyUSB0", slave_id=1):
        """
        Connect and Start Polling Thread to the KORAS Gripper (Motor Enable)
        """
        if self.connected:
            return

        self.port_name = port_name
        self.slave_id = slave_id
        self.mb = ModbusSerialClient(method="rtu",
                                     port=self.port_name,
                                     baudrate=self.baudrate,
                                     stopbits=self.stop_bits,
                                     bytesize=self.data_bits,
                                     parity=self.parity,
                                     timeout=1)
        self.connected = self.mb.connect()

        if self.connected is True:
            print(f"{GREEN}[KORAS]{NC} Modbus connected. & Try Motor Enable")
            self.run_action = True
            self.send_order(GrpCmd.MotorEnable, 0)
            self.init()

            self.action_thread = threading.Thread(target=self.communication_func)
            self.action_thread.daemon = True
            self.action_thread.start()
        else:
            print(f"{RED}[KORAS]{NC} Modbus connection failed.")
            exit(1)

    def disconnect(self):
        """
        Stop and Disconnect from the KORAS Gripper (Motor Disable)
        """

        if self.connected:
            self.send_order(GrpCmd.MotorStop, 0)
            self.send_order(GrpCmd.MotorDisable, 0)

        if self.run_action:
            self.run_action = False
            self.action_thread.join()
        self.mb.close()
        print(f"{GREEN}[KORAS]{NC} Disconnected.")

    # ------------------- Communication -------------------
    def send_order(self, cmd: int, value: int):
        """
        Send order to the KORAS Gripper

        Parameters:
        - cmd (int): Command
        - value (int): Value
        """
        with self.mutex:
            result = self.mb.write_registers(address=0, values=[cmd, value], unit=self.slave_id)
            if result.isError():
                print(f"{RED}[KORAS]{NC} Failed to send command {cmd}")

    def recv_data(self):
        """
        Receive data from the KORAS Gripper

        Parameters:
        - cmd (int): Command
        - value (int): Value
        """
        with self.mutex:
            rr = self.mb.read_input_registers(address=10, count=8, unit=self.slave_id)
            if rr.isError():
                print(f"{RED}[KORAS]{NC} Failed to read registers")
                return

            regs = rr.registers
            status_word = regs[0]

            # Status bit decode
            status_flags = {
                "motor_enable": bool(status_word & StatusBits.MotorEnable),
                "initialize": bool(status_word & StatusBits.Initialize),
                "pos_control": bool(status_word & StatusBits.PosControl),
                "vel_control": bool(status_word & StatusBits.VelControl),
                "cur_control": bool(status_word & StatusBits.CurControl),
                "is_open": bool(status_word & StatusBits.OpenActive),
                "is_close": bool(status_word & StatusBits.CloseActive),
                "motor_fault": bool(status_word & StatusBits.MotorFault),
            }

            # Saved information
            self.m_info = {
                "status_raw": status_word,  # Original value
                "status": status_flags,  # Status flags
                "position": regs[1],  # Motor Position [deg]
                "current": regs[2],  # Motor Current [mA]
                "velocity": regs[3],  # Motor Velocity [rpm]
                "grp_pos_percent": regs[4],  # Finger Position (0~1000)
                "bus_voltage": regs[7],  # Bus Voltage [V]
            }

    def communication_func(self):
        """
        Communication function
        """
        while self.run_action is True and self.connected is True:
            self.recv_data()
            time.sleep(0.01)

    # ------------------- Commands -------------------
    def init(self):
        """
        Initialize the KORAS Gripper
        """
        self.send_order(GrpCmd.Initialize, 0)

    def grip(self, grip_distance: int = -1):
        """
        Grip the KORAS Gripper

        Parameters:
        - grip_distance (int): Grip distance (0 ~ 1000) [0.1mm]
        """
        if grip_distance == -1:
            self.send_order(GrpCmd.Close, 0)
        else:
            if grip_distance < 0:
                self.send_order(GrpCmd.Close, 0)
            elif grip_distance > 1000:
                self.send_order(GrpCmd.Open, 0)
            else:
                self.send_order(GrpCmd.FingerPosition, grip_distance)

    def release(self, release_distance: int = -1):
        """
        Release the KORAS Gripper

        Parameters:
        - release_distance (int): Release distance (0 ~ 1000) [0.1mm]
        """
        if release_distance == -1:
            self.send_order(GrpCmd.Open, 0)
        else:
            if release_distance < 0:
                self.send_order(GrpCmd.Open, 0)
            elif release_distance > 1000:
                self.send_order(GrpCmd.Close, 0)
            else:
                self.send_order(GrpCmd.FingerPosition, release_distance)

    def vacuum(self, vacuum: bool = True):
        """
        Set the vacuum of the KORAS Gripper
        """
        if vacuum:
            self.send_order(GrpCmd.VacuumOn, 0)
        else:
            self.send_order(GrpCmd.VacuumOff, 0)

    def opt_velocity(self, velocity: int = 50):
        """
        Set the velocity(motor speed)[%] of the KORAS Gripper
        """
        self.send_order(GrpCmd.SetMotorSpeed, max(0, min(100, velocity)))

    def opt_force(self, force: int = 50):
        """
        Set the force(motor torque)[%] of the KORAS Gripper
        """
        self.send_order(GrpCmd.SetMotorTorque, max(0, min(100, force)))

    def get_position(self):
        """
        Get the position of the KORAS Gripper [mm]

        Returns:
        - float: Position [mm]
        """
        return self.m_info["position"] / 100.0

    def get_status(self):
        """
        Get the status of the KORAS Gripper [bool]

        Returns:
        - bool: True if gripper is closed, False if gripper is open
        """
        return self.m_info["status"]["is_close"]
