# egh_modbus.py
# (EN) IO-Link (EGH) <-> Modbus adapter keeping KetiZimmer style
# (KO) KetiZimmer 스타일을 유지한 IO-Link(EGH) <-> Modbus 어댑터

import argparse
import threading
import time

from pymodbus.client import ModbusTcpClient


class KetiEGHModbus:
    """
    (EN) Control SCHUNK EGH via an IO-Link master that exposes Modbus registers.
         You MUST set correct Modbus address/bit mapping of your IO-Link master.
    (KO) IO-Link 마스터가 노출하는 Modbus 레지스터를 통해 EGH 제어.
         현장 IO-Link 마스터의 주소/비트 매핑을 정확히 설정해야 함.
    """

    def __init__(
        self,
        ip="192.168.0.10",
        port=502,
        # --- Mapping (example defaults; replace with your master's map) ---
        # (EN) Output (controller -> gripper), Holding Registers base address
        # (KO) 출력(컨트롤러->그리퍼), 보유레지스터 베이스 주소
        out_base=0x0100,
        # (EN) Input (gripper -> controller), Input Registers base address
        # (KO) 입력(그리퍼->컨트롤러), 입력레지스터 베이스 주소
        in_base=0x0200,
        # (EN) Bit masks inside the first output word (command)
        # (KO) 첫 출력 워드(명령) 내 비트 마스크
        bit_cmd_open=0x0001,
        bit_cmd_close=0x0002,
        bit_cmd_stop=0x0004,
        bit_cmd_ack_err=0x0008,
        # (EN) Bit masks inside the first input word (status)
        # (KO) 첫 입력 워드(상태) 내 비트 마스크
        bit_st_ready=0x0001,
        bit_st_fault=0x0002,
        bit_st_moving=0x0004,
        # (EN) Process data layout (words)
        # word offsets from bases (0 = first word)
        # out: [0]=cmd bits, [1]=target_width(0..1000), [2]=velocity(0..255), [3]=reserved
        # in : [0]=status bits, [1]=actual_width(0..1000), [2]=error_code, [3]=reserved
        out_off_cmd=0,
        out_off_width=1,
        out_off_vel=2,
        in_off_status=0,
        in_off_width=1,
        in_off_error=2,
    ):
        # Conn
        self.ip, self.port = ip, port
        self.mb = ModbusTcpClient(host=self.ip, port=self.port)
        self.connected = False

        # Mapping
        self.out_base = out_base
        self.in_base = in_base
        self.bit_cmd_open = bit_cmd_open
        self.bit_cmd_close = bit_cmd_close
        self.bit_cmd_stop = bit_cmd_stop
        self.bit_cmd_ack_err = bit_cmd_ack_err
        self.bit_st_ready = bit_st_ready
        self.bit_st_fault = bit_st_fault
        self.bit_st_moving = bit_st_moving
        self.out_off_cmd = out_off_cmd
        self.out_off_width = out_off_width
        self.out_off_vel = out_off_vel
        self.in_off_status = in_off_status
        self.in_off_width = in_off_width
        self.in_off_error = in_off_error

        # Runtime
        self.thread = threading.Thread(target=self._loop)
        self.thread_run = False

        # Cached state
        self.status_word = 0
        self.actual_width = 0  # (0..1000) => 0%..100%
        self.error_code = 0

        # User options
        self.velocity = 100  # (EN) 0..255 (KO) 0..255

        # Command queue
        self._pending_cmd = None  # ('open'|'close'|'stop'|'width'|'ack', value)

    # ----- Public API (same UX as your Zimmer wrapper) -----
    def connect(self, ip=None, port=None):
        """(EN) Connect and start polling thread (KO) 연결 및 폴링 스레드 시작"""
        if ip:
            self.ip = ip
        if port:
            self.port = port
        self.mb = ModbusTcpClient(host=self.ip, port=self.port)
        self.connected = self.mb.connect()
        if not self.connected:
            print("[KetiEGHModbus] not connected")
            raise SystemExit(1)
        self.thread.daemon = True
        self.thread_run = True
        self.thread.start()
        print("[KetiEGHModbus] connected")

    def disconnect(self):
        """(EN) Stop polling and close (KO) 폴링 중단 및 연결 종료"""
        self.thread_run = False
        try:
            self.thread.join(timeout=1.0)
        except:  # (KO) 스레드 종료 보장
            pass
        self.mb.close()
        print("[KetiEGHModbus] disconnected")

    def init(self):
        """
        (EN) Basic init: clear errors, open slightly to known state.
        (KO) 기본 초기화: 에러 클리어 후 기준 폭으로 열기.
        """
        # Acknowledge error first
        self._queue_cmd(("ack", None))
        time.sleep(0.1)
        # Open to 100% as default known reference
        self.release(width_percent=100, sync=True)
        print("[KetiEGHModbus] Init")

    def grip(self, width_percent=0, sync=True):
        """
        (EN) Close to target width percentage (0% = fully closed).
        (KO) 목표 폭(%)까지 닫기. 0% = 완전 닫힘.
        """
        self.set_velocity(self.velocity)
        self._queue_cmd(("width", max(0, min(100, int(width_percent)))))
        if sync:
            self._wait_idle()

    def release(self, width_percent=100, sync=True):
        """
        (EN) Open to target width percentage (100% = fully open).
        (KO) 목표 폭(%)까지 열기. 100% = 완전 개방.
        """
        self.set_velocity(self.velocity)
        self._queue_cmd(("width", max(0, min(100, int(width_percent)))))
        if sync:
            self._wait_idle()

    def stop(self):
        """(EN) Immediate stop (KO) 즉시 정지"""
        self._queue_cmd(("stop", None))

    def set_velocity(self, vel_0_255):
        """(EN) Set motion velocity (KO) 속도 설정"""
        self.velocity = max(0, min(255, int(vel_0_255)))

    def opt_velocity(self, velocity=50):
        """
        (EN) Set velocity option (0-100%) - compatible with other grippers
        (KO) 속도 옵션 설정 (0-100%) - 다른 그리퍼와 호환
        """
        # Convert 0-100% to 0-255 range
        vel_255 = int(velocity * 255 / 100)
        self.set_velocity(vel_255)

    def opt_force(self, force=50):
        """
        (EN) Set force option (0-100%) - placeholder for EGH (no force control)
        (KO) 힘 옵션 설정 (0-100%) - EGH용 플레이스홀더 (힘 제어 없음)
        """
        # EGH doesn't have force control, so this is a no-op for compatibility
        pass

    # ----- Helpers -----
    def get_status(self):
        """
        (EN) Return dict with status flags and current width (%).
        (KO) 상태 플래그와 현재 폭(%) 반환.
        """
        return {
            "ready": bool(self.status_word & self.bit_st_ready),
            "fault": bool(self.status_word & self.bit_st_fault),
            "moving": bool(self.status_word & self.bit_st_moving),
            "width_percent": int(round(self.actual_width / 10.0)),  # 0..100
            "error_code": self.error_code,
        }

    def get_position(self):
        """
        (EN) Get current gripper position as width percentage
        (KO) 그리퍼 현재 위치를 폭 퍼센트로 반환

        Returns:
            dict: Status information including width_percent
        """
        return self.get_status()

    # ----- Internal loop -----
    def _loop(self):
        """(EN) Poll IO-Link PD via Modbus and flush commands (KO) Modbus 폴링 및 명령 처리"""
        while self.thread_run and self.connected:
            # 1) Read input PD (status/width/error)
            rr = self.mb.read_input_registers(
                self.in_base + self.in_off_status, count=3
            )
            if rr and hasattr(rr, "registers"):
                regs = rr.registers
                self.status_word = regs[0]
                self.actual_width = regs[1]  # 0..1000 (scaled 0.1%)
                self.error_code = regs[2]

            # 2) If pending command, write outputs
            if self._pending_cmd:
                kind, val = self._pending_cmd
                if kind == "ack":
                    self._write_out(cmd_bits=self.bit_cmd_ack_err, width=None, vel=None)
                elif kind == "stop":
                    self._write_out(cmd_bits=self.bit_cmd_stop, width=None, vel=None)
                elif kind == "width":
                    width_0_1000 = int(val * 10)  # 0..100 -> 0..1000
                    self._write_out(
                        cmd_bits=(
                            self.bit_cmd_open if val >= 50 else self.bit_cmd_close
                        ),
                        width=width_0_1000,
                        vel=self.velocity,
                    )
                self._pending_cmd = None

            time.sleep(0.01)

    def _write_out(self, cmd_bits=0, width=None, vel=None):
        """(EN) Compose and write holding registers (KO) 보유레지스터 작성/쓰기"""
        # Read-modify-write for cmd word
        rr = self.mb.read_holding_registers(self.out_base + self.out_off_cmd, count=3)
        if rr and hasattr(rr, "registers"):
            out = rr.registers
        else:
            out = [0, 0, 0]

        out[0] = cmd_bits  # command bits
        if width is not None:
            out[1] = max(0, min(1000, int(width)))
        if vel is not None:
            out[2] = max(0, min(255, int(vel)))

        self.mb.write_registers(self.out_base + self.out_off_cmd, out)

    def _queue_cmd(self, item):
        self._pending_cmd = item

    def _wait_idle(self, timeout=5.0):
        """(EN) Wait until not moving or timeout (KO) 정지 대기"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if not (self.status_word & self.bit_st_moving):
                return True
            time.sleep(0.01)
        return False


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--ip", default="192.168.3.113")
    p.add_argument("--port", type=int, default=502)
    # Mapping params (override here if needed)
    p.add_argument(
        "--out_base", type=lambda x: int(x, 0), default=0x0100
    )  # e.g., 0x0100
    p.add_argument(
        "--in_base", type=lambda x: int(x, 0), default=0x0200
    )  # e.g., 0x0200
    p.add_argument("--vel", type=int, default=120)
    p.add_argument(
        "--action",
        choices=["init", "grip", "release", "stop", "status"],
        default="init",
    )
    p.add_argument("--width", type=int, default=100)  # 0..100 (%)
    args = p.parse_args()

    g = KetiEGHModbus(
        ip=args.ip, port=args.port, out_base=args.out_base, in_base=args.in_base
    )
    g.connect()

    if args.action == "init":
        g.init()
    elif args.action == "grip":
        g.grip(width_percent=max(0, min(100, args.width)), sync=True)
    elif args.action == "release":
        g.release(width_percent=max(0, min(100, args.width)), sync=True)
    elif args.action == "stop":
        g.stop()
    elif args.action == "status":
        pass

    st = g.get_status()
    print(
        "[EGH] ready={ready} fault={fault} moving={moving} width%={width_percent} err={error_code}".format(
            **st
        )
    )
    g.disconnect()
