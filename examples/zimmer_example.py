import sys
import time
from pathlib import Path

# 프로젝트 루트를 import 경로에 추가 (examples/ 밖의 zimmer.py 사용)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from zimmer import Zimmer

# ---------------- User settings ----------------
GRIPPER_IP = "192.168.0.253"  # IO-Link master IP (그리퍼가 연결된 IO-Link 마스터의 IP)
GRIPPER_PORT = 502  # Modbus TCP port (Modbus TCP 포트)

TARGET_GAP_MM = 43.0  # Jaw gap to return to [mm] (되돌아갈 목표 jaw gap [mm])
VELOCITY = 50  # Drive velocity [1~100 %]
FORCE = 50  # Grip force [1~100 %]
# ---------------------------------------------------------------


def mm_to_counts(mm: float) -> int:
    """Convert jaw gap [mm] to WorkPosition counts [0.01 mm] (mm -> 0.01mm 카운트 변환)"""
    return int(mm * 100)


def print_position(gripper: Zimmer, label: str):
    position_mm = gripper.get_actual_position() / 100.0
    print(f"[ EXAMPLE ] {label} -> actual position: {position_mm:.2f} mm")


def main():
    gripper = Zimmer()

    # 1. Connect (연결) — on failure, zimmer.py prints an error and exits the process
    gripper.connect(ip=GRIPPER_IP, port=GRIPPER_PORT)

    try:
        gripper.opt_velocity(VELOCITY)
        gripper.opt_force(FORCE)

        # 2. Initialize (초기화) — moves the gripper fully open (BasePosition)
        gripper.init()

        # NOTE: init() 이후에도 백그라운드에서 BasePosition 이동이 계속되므로 완료까지 대기
        while gripper.gripper_send_flag is True:
            time.sleep(0.01)
        print_position(gripper, "Initialized (fully open)")

        # 3. Close (닫기)
        # NOTE: 이미 열린 상태(BasePosition)에서 release() 먼저 호출하면 무한 블로킹되므로 닫기부터 시작
        gripper.grip(sync=True)
        print_position(gripper, "Closed")

        # 4. Open (열기)
        gripper.release(sync=True)
        print_position(gripper, "Opened")

        # 5. Move to a specific position (특정 위치로 이동)
        gripper.custom_position(mm_to_counts(TARGET_GAP_MM), sync=True)
        print_position(gripper, f"Moved to {TARGET_GAP_MM} mm")

        # Report diagnosis if any error occurred (에러 발생 시 진단 메시지 출력)
        error_code, error_msg = gripper.get_diagnosis()
        if error_code != 0:
            print(f"[ EXAMPLE ] Diagnosis 0x{error_code:04X}: {error_msg}")

    finally:
        # 6. Disconnect (연결 해제)
        gripper.disconnect()


if __name__ == "__main__":
    main()
