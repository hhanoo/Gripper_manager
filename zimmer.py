# ============================================================
# Zimmer Gripper Modbus Register Map (출력/입력 데이터 워드)
# ============================================================
#
# Output data word 0 - 0x0801 (ControlWord)
#   - UINT16, Write
#   - Execute a command (only one bit at a time)
#     0      : No action
#     1      : DataTransfer       (0x0001) – apply process data / workpiece data
#     2      : WritePDU           (0x0002) – write current dataset to tool recipe
#     4      : ResetDirectionFlag (0x0004) – reset direction flag
#     8      : Teach              (0x0008) – save current pos as TeachPosition
#     256    : MoveToBase         (0x0100) – move gripper to BasePosition
#     512    : MoveToWork         (0x0200) – move gripper to WorkPosition
#     1024   : JogToWork          (0x0400) – jog toward WorkPosition (slow move)
#     2048   : JogToBase          (0x0800) – jog toward BasePosition (slow move)
#
# Output data word 1 - 0x0802 (DeviceMode, Workpiece No)
#   - UINT8 DeviceMode + UINT8 WorkpieceNo
#   - DeviceMode: defines motion profile / operation mode (0~255)
#   - WorkpieceNo: dataset index (0~32)
#
# Output data word 2 - 0x0803 (Reserved, PositionTolerance)
#   - PositionTolerance [0.01 mm] e.g. 50 → 0.5 mm tolerance
#
# Output data word 3 - 0x0804 (GripForce, DriveVelocity)
#   - High byte : GripForce (1~100%)
#   - Low  byte : DriveVelocity (1~100%)
#
# Output data word 4 - 0x0805 (BasePosition)
#   - External gripping reference position [0.01 mm]
#
# Output data word 5 - 0x0806 (ShiftPosition)
#   - Intermediate shift position [0.01 mm]
#
# Output data word 6 - 0x0807 (TeachPosition)
#   - Position saved by Teach command [0.01 mm]
#
# Output data word 7 - 0x0808 (WorkPosition)
#   - Internal gripping target position [0.01 mm]
#
# ------------------------------------------------------------
#
# Input data word 0 - 0x0002 (StatusWord)
#   - UINT16, Read
#   - Bit-coded status flags:
#     0x4000 : MoveWorkpositionFlag
#     0x2000 : MoveBasepositionFlag
#     0x1000 : DataTransferOK
#     0x0400 : AtWorkposition
#     0x0100 : AtBaseposition
#     0x0040 : PLCActive
#     0x0008 : MovementComplete
#     0x0004 : InMotion
#     0x0002 : MotorOn
#     0x0001 : HomingPositionOK
#
# Input data word 1 - 0x0003 (Diagnosis)
#   - Diagnostic information (see device manual)
#
# Input data word 2 - 0x0004 (ActualPosition)
#   - Current jaw position [0.01 mm]
#   - 0 ~ max jaw stroke (model-dependent)
#   - Resolution: 0.01 mm / Accuracy: ±0.1 mm
#
# ============================================================

import threading
import time

from pymodbus.client import ModbusTcpClient

from console_colors import BLUE, GREEN, NC, RED, YELLOW


# ------------------------------ constants (상수) ------------------------------
# yapf: disable
class ControlWord:
    """ControlWord (0x0801) bits (명령 비트)"""
    DataTransfer       = 0x0001  # apply process data (프로세스 데이터 적용)
    WritePDU           = 0x0002  # write dataset (데이터셋 저장)
    ResetDirectionFlag = 0x0004  # reset direction flag (방향 플래그 리셋)
    Teach              = 0x0008  # save current pos as TeachPosition (교시)
    MoveToBase         = 0x0100  # move to BasePosition (베이스로 이동 = 외측 홈 이동)
    MoveToWork         = 0x0200  # move to WorkPosition (워크로 이동 (내측 홈 이동))
    JogToWork          = 0x0400  # jog toward WorkPosition (워크 방향 조그)
    JogToBase          = 0x0800  # jog toward BasePosition (베이스 방향 조그)


class DeviceMode:
    """DeviceMode high byte values (모드 값)"""
    NotSend           = 0
    Idle              = 1
    GripperReset      = 2
    MoterControlOn    = 3
    MoterControlOff   = 5
    OutsideHoming     = 10
    JogOperation      = 11
    InsideHoming      = 14
    # NOTE: vendor specific modes can be added (제조사 모드 추가 가능)


class StatusFlags:
    """StatusWord (0x0002) flags (상태 비트)"""
    MoveWorkpositionFlag = 0x4000
    MoveBasepositionFlag = 0x2000
    DataTransferOK       = 0x1000
    AtWorkposition       = 0x0400
    AtBaseposition       = 0x0100
    PLCActive            = 0x0040
    MovementComplete     = 0x0008
    InMotion             = 0x0004
    MotorOn              = 0x0002
    HomingPositionOK     = 0x0001

class DiagnosisMessage:
    """Zimmer Gripper error diagnosis messages (매뉴얼 기준)"""

    MESSAGE_MAP = {
        0x0000: "Device is ready for operation",

        0x0001: "Motor controller is switched off",

        0x0100: "Actuator power supply is not present or is too low",
        0x0101: "Temperature above maximum permitted temperature",
        0x0102: "Max. permitted temperature undershot",

        0x0206: "Motion task cannot be executed (CRC error)",

        0x0300: "ControlWord is not plausible",
        0x0301: "Positions implausible",
        0x0302: "GripForce is not plausible",
        0x0303: "DriveVelocity not plausible",
        0x0304: "PositionTolerance is not plausible",
        0x0305: "Position measuring system not referenced",
        0x0306: "DeviceMode is not plausible",
        0x0307: "Motion task cannot be executed",
        0x0308: "WorkpieceNo cannot be selected",

        0x0313: "Calculated ShiftPosition exceeded",

        0x0402: "Jam",
        0x0404: "Position sensor error",

        0x0406: "Internal error",
        0x040B: "Internal error",
        0x040C: "Internal error",
        0x040D: "Internal error",
        0x040E: "Internal error",
        0x040F: "Internal error",
    }

    @classmethod
    def get(cls, code: int) -> str:
        # Return diagnosis message for given error code (에러 코드에 따른 메시지 반환)
        return cls.MESSAGE_MAP.get(code, f"Unknown error code (0x{code:04X})")

# yapf: enable
# ------------------------------------------------------------------------------


class Zimmer:
    # 2-Finger Gripper : GEH6060IL, GEH6040IL
    # 3-Finger Gripper : GED6060IL, GED6040IL
    def __init__(self):
        """
        Initialize the Zimmer class
        """

        # Register address definition
        self.NUM_RECV_REG = 3
        self.NUM_SEND_REG = 8

        # Status word definition
        # yapf: disable
        self.MoveWorkpositionFlag = StatusFlags.MoveWorkpositionFlag
        self.MoveBasepositionFlag = StatusFlags.MoveBasepositionFlag
        self.DataTransferOK       = StatusFlags.DataTransferOK
        self.AtWorkposition       = StatusFlags.AtWorkposition
        self.AtBaseposition       = StatusFlags.AtBaseposition
        self.PLCActive            = StatusFlags.PLCActive
        self.MovementComplete     = StatusFlags.MovementComplete
        self.InMotion             = StatusFlags.InMotion
        self.MotorOn              = StatusFlags.MotorOn
        self.HomingPositionOK     = StatusFlags.HomingPositionOK
        # yapf: enable

        # Zimmer connection variables
        self.ip = ''
        self.port = 0
        self.connected = False

        # Modbus client
        self.mb = ModbusTcpClient(host=self.ip, port=self.port)

        # Zimmer Gripper ------------------------------------------------------------
        self.ADDR_RECV = 0x0011  #NOTE: Reference 'Input: IO-Link channel' from 'Modbus TCP Memory Map' in Turck documentation
        self.ADDR_SEND = 0x0811  #NOTE: Reference 'Output: IO-Link channel' from 'Modbus TCP Memory Map' in Turck documentation

        self.reg_read = 0
        self.reg_write = [0, 0, 0, 0, 0, 0, 0, 0]

        self.gripper_thread_run = False
        self.gripper_thread = threading.Thread(target=self.communication_func)

        self.gripper_gripForce = 50
        self.gripper_DriveVelocity = 50
        self.gripper_BasePosition = 0
        self.gripper_ShiftPosition = 0
        self.gripper_WorkPosition = 0

        #NOTE: Maximum stroke derived from tooltip gap & Must match gripper spec ('stroke per jaw') with additional 75 mm clearance
        self.gripper_gap_maximum = (8200 // 2) + 75  # (tooltip gap // 2) = 41.00 mm = max stroke

        self.gripper_StatusWord = 0
        self.gripper_Diagnosis = 0
        self.gripper_ActualPosition = 0

        self.gripper_send_flag = False
        self.gripper_comm_step = 0
        self.gripper_init_flag = False
        self.gripper_move_to_work_flag = False

    # ---------------- Connection ----------------
    def connect(self, ip='192.168.3.112', port=502):
        """
        Connect to the Zimmer Gripper

        Parameters:
        - ip (str): IP address of the Zimmer Gripper
        - port (int): Port number of the Zimmer Gripper
        """
        if self.connected is True:
            return

        self.ip, self.port = ip, port
        self.mb = ModbusTcpClient(host=self.ip, port=self.port)
        self.connected = self.mb.connect()

        if self.connected is True:
            print(f'{GREEN}[SUCCESS]{NC} Connected gripper')
            self.gripper_thread = threading.Thread(target=self.communication_func)
            self.gripper_thread.daemon = True
            self.gripper_thread.start()
        else:
            print(f'{RED}[ERROR]{NC} Not connected gripper')
            exit(1)

    def disconnect(self):
        """
        Disconnect from the Zimmer Gripper
        """
        if self.connected is False:
            return

        self.gripper_thread_run = False
        self.mb.close()
        self.connected = False
        print(f'{GREEN}[SUCCESS]{NC} Disconnected gripper')

    # ---------------- Communication ----------------
    def communication_func(self):
        """
        Gripper function
        """
        self.gripper_thread_run = True

        while self.gripper_thread_run is True and self.connected is True:
            # read input registers (입력 레지스터 읽기)
            self.reg_read = self.mb.read_input_registers(self.ADDR_RECV, count=self.NUM_RECV_REG, slave=16)
            self.gripper_StatusWord = self.reg_read.registers[0]
            self.gripper_Diagnosis = self.reg_read.registers[1]
            self.gripper_ActualPosition = self.reg_read.registers[2]

            if self.gripper_send_flag is True:
                # 0. PLC active bit check
                if self.gripper_comm_step == 0:
                    if bool(self.reg_read.registers[0] & self.PLCActive) is True:
                        print(f"\n{YELLOW}[ ZIMMER ]{NC} 0. PLC active bit check complete")
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1

                # 1. Data transfer ok bit & motor on bit check
                elif self.gripper_comm_step == 1:
                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is True \
                            and bool(self.reg_read.registers[0] & self.MotorOn) is True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 1. Data transfer ok bit & motor on bit check complete")
                        self.reg_write[0] = 0
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1

                # 2. Handshake
                elif self.gripper_comm_step == 2:
                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is not True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 2. Handshake is done")
                        # re-arm data transfer with a specific device mode (특정 모드로 재전송)
                        self.reg_write[0] = ControlWord.DataTransfer
                        self.reg_write[1] = (85 << 8) | 0  # NOTE: vendor-specific mode 85 (제조사 모드 85 가정)
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1

                # 3. Data transfer ok bit check
                elif self.gripper_comm_step == 3:
                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 3. Data transfer ok bit check complete")
                        self.reg_write[0] = 0
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1
                        self.gripper_init_flag = True

                # 4. Grip move to workposition/baseposition
                elif self.gripper_comm_step == 4:
                    if bool(self.reg_read.registers[0] & self.DataTransferOK) is not True:
                        if self.gripper_move_to_work_flag is True:
                            print(f"{YELLOW}[ ZIMMER ]{NC} 4. Grip move to workposition")
                            self.reg_write[0] = ControlWord.MoveToWork
                        else:
                            if bool(self.reg_read.registers[0] & self.AtBaseposition) is not True:
                                print(f"{YELLOW}[ ZIMMER ]{NC} 4. Grip move to baseposition")
                                self.reg_write[0] = ControlWord.MoveToBase

                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1

                # 5. Grip move complete check
                elif self.gripper_comm_step == 5:
                    if bool(self.reg_read.registers[0] & self.InMotion) is True \
                            and bool(self.reg_read.registers[0] & self.MovementComplete) is not True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 5. Grip move complete check")
                        self.gripper_comm_step += 1

                # 6. Move complete check
                elif self.gripper_comm_step == 6:
                    if bool(self.reg_read.registers[0] & self.InMotion) is not True \
                            and bool(self.reg_read.registers[0] & self.MovementComplete) is True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 6. Move complete")
                        self.reg_write[0] = ControlWord.ResetDirectionFlag
                        self.mb.write_registers(self.ADDR_SEND, self.reg_write)
                        self.gripper_comm_step += 1

                # 7. Move workposition flag & move baseposition flag check
                elif self.gripper_comm_step == 7:
                    if bool(self.reg_read.registers[0] & self.MoveWorkpositionFlag) is not True \
                            and bool(self.reg_read.registers[0] & self.MoveBasepositionFlag) is not True:
                        print(f"{YELLOW}[ ZIMMER ]{NC} 7. Move workposition flag & move baseposition flag check")
                        self.gripper_comm_step = -1  # reset communication step
                        self.gripper_send_flag = False

            time.sleep(0.01)

        self.reg_read = 0

    # ------------------- Commands -------------------
    def init(self):
        """
        Zimmer Gripper initialization
        """
        # build command
        self.reg_write[0] = ControlWord.DataTransfer  # ControlWord
        self.reg_write[1] = (DeviceMode.MoterControlOn << 8) | 0  # DeviceMode(3) / WorkpieceNo(0)
        self.reg_write[2] = 50  # PositionTolerance = 0.50 mm
        self.reg_write[3] = (self.gripper_gripForce << 8) | self.gripper_DriveVelocity  # GripForce / DriveVelocity
        self.reg_write[4] = 100  # BasePosition = 1.00 mm
        self.reg_write[5] = 2000  # ShiftPosition = 20.00 mm
        self.reg_write[7] = self.gripper_gap_maximum  # WorkPosition = max stroke

        self.gripper_BasePosition = self.reg_write[4]
        self.gripper_ShiftPosition = self.reg_write[5]
        self.gripper_WorkPosition = self.reg_write[7]

        self.gripper_init_flag = False
        self.gripper_comm_step = 0
        self.gripper_send_flag = True

        while self.gripper_init_flag is False:
            time.sleep(0.001)
        print(f'{BLUE}[ ZIMMER ]{NC} Initialized')

    def grip(self, jaw_gap=-1, sync=True):
        """
        Zimmer Gripper gripping (closing)

        Parameters:
        - jaw_gap (int): Jaw gap (75 ~ max distance) [0.01mm]
        - sync (bool): Synchronization flag
        """

        if jaw_gap == -1:
            actual_position = self.gripper_gap_maximum
        else:
            # convert gap[mm] to WorkPosition counts [0.01mm] (갭→워크포지션 변환)
            # actual_position = (74 - grip_distance) / 2 * 100 + 100
            actual_position = self.gripper_gap_maximum - (jaw_gap / 2) + 100
            if actual_position > self.gripper_gap_maximum:
                actual_position = self.gripper_gap_maximum

        print(f'\n{BLUE}[ ZIMMER ]{NC} jaw_gap : {jaw_gap}')
        print(f'{BLUE}[ ZIMMER ]{NC} actual_position : {actual_position}')

        if self.gripper_init_flag is True:
            self.reg_write[0] = ControlWord.DataTransfer
            self.reg_write[1] = (DeviceMode.MoterControlOn << 8) | 0
            self.reg_write[2] = 50  # PositionTolerance
            self.reg_write[3] = (self.gripper_gripForce << 8) | self.gripper_DriveVelocity
            self.reg_write[4] = 100  # BasePosition
            self.reg_write[5] = int(actual_position - 100)  # ShiftPosition
            self.reg_write[7] = int(actual_position)  # WorkPosition

            self.gripper_BasePosition = self.reg_write[4]
            self.gripper_ShiftPosition = self.reg_write[5]
            self.gripper_WorkPosition = self.reg_write[7]

            self.gripper_comm_step = 0  # reset communication step
            self.gripper_send_flag = True
            self.gripper_move_to_work_flag = True

            if sync is True:
                while self.gripper_send_flag is True:
                    time.sleep(0.001)

    def release(self, jaw_gap=-1, sync=True):
        """
        Zimmer Gripper releasing (opening)

        Parameters:
        - jaw_gap (int): Jaw gap (75 ~ max distance) [0.01mm]
        - sync (bool): Synchronization flag
        """
        if jaw_gap == -1:
            actual_position = 100
        else:
            # convert gap[mm] to WorkPosition counts [0.01mm] (갭→워크포지션 변환)
            # actual_position = (74 - jaw_gap) / 2 * 100 + 150
            actual_position = self.gripper_gap_maximum - (jaw_gap / 2) - 100
            if actual_position < 100:
                actual_position = 100

        print(f'\n{BLUE}[ ZIMMER ]{NC} jaw_gap : {jaw_gap}')
        print(f'{BLUE}[ ZIMMER ]{NC} actual_position : {actual_position}')

        if self.gripper_init_flag is True:
            self.reg_write[0] = ControlWord.DataTransfer
            self.reg_write[1] = (DeviceMode.MoterControlOn << 8) | 0
            self.reg_write[2] = 50  # PositionTolerance
            self.reg_write[3] = (self.gripper_gripForce << 8) | self.gripper_DriveVelocity
            self.reg_write[4] = int(actual_position)  # BasePosition
            self.reg_write[5] = int(actual_position + 100)  # ShiftPosition
            self.reg_write[7] = self.gripper_gap_maximum  # WorkPosition

            self.gripper_BasePosition = self.reg_write[4]
            self.gripper_ShiftPosition = self.reg_write[5]
            self.gripper_WorkPosition = self.reg_write[7]

            self.gripper_comm_step = 0  # reset communication step
            self.gripper_send_flag = True
            self.gripper_move_to_work_flag = False

            if sync is True:
                while self.gripper_send_flag is True:
                    time.sleep(0.001)

    def custom_position(self, jaw_gap, sync=True):
        """
        Zimmer Gripper custom position (move to jaw gap)

        Parameters:
        - jaw_gap (int): Jaw gap (75 ~ max distance) [0.01mm]
        - sync (bool): Synchronization flag
        """
        jaw_gap = int(jaw_gap)
        actual_position = self.gripper_gap_maximum - (jaw_gap / 2) - 100

        if actual_position < self.gripper_ActualPosition:
            self.release(jaw_gap, sync)
        elif actual_position > self.gripper_ActualPosition:
            self.grip(jaw_gap, sync)
        else:
            print(f"{YELLOW}[ ZIMMER ]{NC} Jaw gap is already at {jaw_gap}")

    def outside_homing(self):
        """
        Zimmer Gripper outside homing
        """
        self.reg_write[0] = ControlWord.DataTransfer
        self.reg_write[1] = (DeviceMode.OutsideHoming << 8) | 0
        self.reg_write[2] = 50  # PositionTolerance
        self.reg_write[3] = (self.gripper_gripForce << 8) | self.gripper_DriveVelocity
        self.reg_write[4] = 100  # BasePosition
        self.reg_write[5] = 2000  # ShiftPosition
        self.reg_write[7] = self.gripper_gap_maximum  # WorkPosition

        self.gripper_comm_step = 0  # reset communication step
        self.gripper_send_flag = True

    def inside_homing(self):
        """
        Zimmer Gripper inside homing
        """
        self.reg_write[0] = ControlWord.DataTransfer
        self.reg_write[1] = (DeviceMode.InsideHoming << 8) | 0
        self.reg_write[2] = 50  # PositionTolerance
        self.reg_write[3] = (self.gripper_gripForce << 8) | self.gripper_DriveVelocity
        self.reg_write[4] = 100  # BasePosition
        self.reg_write[5] = 2000  # ShiftPosition
        self.reg_write[7] = self.gripper_gap_maximum  # WorkPosition

        self.gripper_comm_step = 0  # reset communication step
        self.gripper_send_flag = True

    def opt_velocity(self, velocity=50):
        """
        Zimmer Gripper velocity option (1~100%)
        """
        self.gripper_DriveVelocity = velocity

    def opt_force(self, force=50):
        """
        Zimmer Gripper force option (1~100%)
        """
        self.gripper_gripForce = force

    def get_base_position(self):
        """
        Zimmer Gripper base position
        """
        return self.gripper_BasePosition

    def get_shift_position(self):
        """
        Zimmer Gripper shift position
        """
        return self.gripper_ShiftPosition

    def get_work_position(self):
        """
        Zimmer Gripper work position
        """
        return self.gripper_WorkPosition

    def get_actual_position(self):
        """
        Zimmer Gripper actual position
        """
        return self.gripper_ActualPosition

    def get_status_word(self):
        """
        Zimmer Gripper status word

        Returns:
        - int: Status word (Decimal)
        - list: Status word bits (16 bits)
        """
        binary_status_word = format(self.gripper_StatusWord, '016b')[-16:]
        return self.gripper_StatusWord, [int(bit) for bit in binary_status_word]

    def get_diagnosis(self):
        """
        Zimmer Gripper diagnosis

        Returns:
        - int: Diagnosis (Decimal)
        - str: Diagnosis message (String)
        """
        code = self.gripper_Diagnosis
        message = DiagnosisMessage.get(code)
        return code, message
