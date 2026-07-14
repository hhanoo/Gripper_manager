import sys
import time
from pathlib import Path

# 프로젝트 루트를 import 경로에 추가 (examples/ 밖의 koras.py 사용)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from koras import KORAS

# ---------------- User settings ----------------
PORT_NAME = "/dev/ttyUSB0"  # Serial port (USB-RS485 어댑터 포트)
SLAVE_ID = 1  # Modbus slave ID (장비 기본값 1)

TARGET_POS_PERCENT = 50.0  # Finger position to move to [0~100 %]
VELOCITY = 50  # Motor speed [1~100 %]
FORCE = 50  # Motor torque [1~100 %]
MOVE_WAIT_SEC = 2.0  # Wait time per motion [sec] (동작당 대기 시간)
# ---------------------------------------------------------------


def percent_to_counts(percent: float) -> int:
    return int(percent * 10)  # 0~100 % -> 0~1000


def print_position(gripper: KORAS, label: str):
    position_percent = gripper.m_info["grp_pos_percent"] / 10.0
    print(f"[ EXAMPLE ] {label} -> finger position: {position_percent:.1f} %")


def main():
    gripper = KORAS()

    # 1. Connect — 내부에서 MotorEnable + Initialize까지 수행, 실패 시 exit(1)
    gripper.connect(port_name=PORT_NAME, slave_id=SLAVE_ID)

    try:
        # NOTE: KORAS 명령은 완료 신호가 없는 비동기 방식이라 고정 대기를 사용
        time.sleep(MOVE_WAIT_SEC)
        print_position(gripper, "Initialized")

        gripper.opt_velocity(VELOCITY)
        gripper.opt_force(FORCE)

        # 2. Close (닫기)
        gripper.grip()
        time.sleep(MOVE_WAIT_SEC)
        print_position(gripper, "Closed")

        # 3. Open (열기)
        gripper.release()
        time.sleep(MOVE_WAIT_SEC)
        print_position(gripper, "Opened")

        # 4. Move to a specific position (특정 위치로 이동)
        gripper.grip(percent_to_counts(TARGET_POS_PERCENT))
        time.sleep(MOVE_WAIT_SEC)
        print_position(gripper, f"Moved to {TARGET_POS_PERCENT} %")

        # Report motor fault if any (모터 폴트 상태 확인)
        if gripper.m_info["status"]["motor_fault"]:
            print("[ EXAMPLE ] Motor fault detected! Re-initialize the gripper.")

    finally:
        # 5. Disconnect — MotorStop + MotorDisable 포함
        gripper.disconnect()


if __name__ == "__main__":
    main()
