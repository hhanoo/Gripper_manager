# Zimmer Gripper Control

**Zimmer 2/3핑거 그리퍼 Modbus TCP/IP 제어 모듈 (zimmer.py) 및 예제 (examples/zimmer_example.py)**

[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)]()
[![pymodbus](https://img.shields.io/badge/pymodbus-3.6.9-4B8BBE?logo=python&logoColor=white)]()

---

## 목차

- [개요](#개요)
- [주요 기능](#주요-기능)
- [시스템 구조](#시스템-구조)
- [프로젝트 구조](#프로젝트-구조)
- [빠른 시작](#빠른-시작)
- [시스템 요구사항](#시스템-요구사항)
- [실행](#실행)
- [사용법](#사용법)
- [설정](#설정)
- [API / 인터페이스](#api--인터페이스)
- [문제 해결](#문제-해결)
- [라이선스](#라이선스)
- [Maintainer](#maintainer)

---

## 개요

### 프로젝트 목적

Zimmer 전동 그리퍼(GEH/GED 60xxIL 시리즈)를 Python으로 제어하기 위한 모듈과 최소 예제입니다.  
그리퍼는 IO-Link 장치이므로 직접 통신하지 않고, **그리퍼가 연결된 IO-Link 마스터(Turck)가 노출하는 Modbus TCP 레지스터**를 통해 제어합니다.

### 주요 구성요소

- **zimmer.py**: Zimmer 그리퍼 제어 라이브러리. 백그라운드 통신 스레드가 8단계 핸드셰이크 프로토콜을 처리
- **examples/zimmer_example.py**: 연결 → 초기화 → 닫기 → 열기 → 특정 위치 이동 → 연결 해제 시퀀스의 최소 예제

### 지원 모델

| 타입  | 모델                 |
| ----- | -------------------- |
| 2핑거 | GEH6060IL, GEH6040IL |
| 3핑거 | GED6060IL, GED6040IL |

---

## 주요 기능

- **열기/닫기 제어**: `grip()` / `release()`로 완전 닫기·열기, `custom_position()`으로 임의 jaw gap 이동
- **8단계 핸드셰이크**: PLC active 확인 → 데이터 전송 → 이동 → 완료 확인까지 백그라운드 스레드가 자동 처리
- **힘/속도 조절**: `opt_force()` / `opt_velocity()` (1~100%)
- **상태 모니터링**: StatusWord 비트, 현재 위치(0.01mm 단위), 진단 코드/메시지 실시간 조회
- **호밍**: `outside_homing()` / `inside_homing()` 지원

---

## 시스템 구조

```
┌─────────────────────────────────────┐
│                Host                 │
│  ┌───────────────────┐              │
│  │ zimmer_example.py │              │
│  └─────────┬─────────┘              │
│            ▼                        │
│  ┌───────────────────────────┐      │
│  │         zimmer.py         │      │
│  │ (comm thread / handshake) │      │
│  └─────────────┬─────────────┘      │
└────────────────┼────────────────────┘
                 │ Modbus TCP (port 502, slave 16)
                 ▼
┌─────────────────────────┐
│  IO-Link Master (Turck) │   ◀── GRIPPER_IP는 이 장비의 IP
│  Input : 0x0001~0x0003  │
│  Output: 0x0801~0x0808  │
└────────────┬────────────┘
             │ IO-Link
             ▼
┌─────────────────────────┐
│     Zimmer Gripper      │
│ (GEH/GED 60xxIL Series) │
└─────────────────────────┘
```

---

## 프로젝트 구조

```
Gripper_manager/
├── zimmer.py                 # Zimmer 그리퍼 제어 라이브러리 (Modbus TCP/IP)
├── examples/
│   └── zimmer_example.py     # 최소 예제 (열기/닫기/특정 위치 이동)
└── docs/
    └── ZIMMER_README.md      # 이 문서
```

---

## 빠른 시작

```bash
# 0. 프로젝트 루트로 이동
cd Gripper_manager

# 1. 필요 라이브러리 설치
pip install pymodbus==3.6.9

# 2. examples/zimmer_example.py 상단 상수 수정 (IO-Link 마스터 IP 등)

# 3. 예제 실행
python3 examples/zimmer_example.py
```

---

## 시스템 요구사항

### 하드웨어

| 항목           | 사양                        | 비고                            |
| -------------- | --------------------------- | ------------------------------- |
| Zimmer 그리퍼  | GEH/GED 60xxIL 시리즈       | IO-Link 장치                    |
| IO-Link 마스터 | Modbus TCP 지원 (예: Turck) | 그리퍼와 호스트 사이 게이트웨이 |
| Ethernet       | 100Mbps 이상                | 호스트 ↔ IO-Link 마스터         |

### 소프트웨어 의존성

| 패키지   | 버전  | 용도               |
| -------- | ----- | ------------------ |
| pymodbus | 3.6.9 | Modbus TCP/IP 통신 |

---

## 실행

```bash
python3 examples/zimmer_example.py
```

실행하면 아래 순서로 동작하며 각 단계 후 현재 위치를 출력합니다.

```
[ EXAMPLE ] Initialized (fully open) -> actual position: 1.00 mm
[ EXAMPLE ] Closed -> actual position: 42.00 mm
[ EXAMPLE ] Opened -> actual position: 1.00 mm
[ EXAMPLE ] Moved to 43.0 mm -> actual position: 20.50 mm
```

---

## 사용법

### 예제 워크플로우

```
connect ──▶ init ──▶ grip ──▶ release ──▶ custom_position ──▶ disconnect
   │          │        │          │              │
IO-Link    완전 열림  완전 닫기  완전 열기   TARGET_GAP_MM 으로 이동
마스터 접속   (Base)
```

### 라이브러리로 사용하기

```python
from zimmer import Zimmer

gripper = Zimmer()
gripper.connect(ip="192.168.0.253", port=502)  # IO-Link 마스터 IP

gripper.opt_velocity(50)
gripper.opt_force(50)
gripper.init()  # 완전 열림(BasePosition)으로 이동

gripper.grip(sync=True)                    # 완전 닫기
gripper.release(sync=True)                 # 완전 열기
gripper.custom_position(4300, sync=True)   # jaw gap 43.00 mm 로 이동

gripper.disconnect()
```

주의사항:

1. `init()` 리턴 후에도 백그라운드에서 BasePosition 이동이 진행되므로, 다음 명령 전에 `gripper_send_flag`가 `False`가 될 때까지 대기해야 합니다 (예제 코드 참고).
2. 이미 열린 상태(BasePosition)에서 `release()`를 호출하면 이동이 시작되지 않아 무한 대기에 빠집니다. 초기화 직후에는 닫기(`grip()`)부터 시작하세요.
3. `sync=False`로 호출하면 비동기로 동작하며, `gripper_send_flag`로 완료 여부를 확인할 수 있습니다.

---

## 설정

`examples/zimmer_example.py` 상단 상수로 설정합니다.

```python
GRIPPER_IP = "192.168.0.253"  # 그리퍼가 연결된 IO-Link 마스터의 IP (그리퍼 자체 IP 아님)
GRIPPER_PORT = 502            # Modbus TCP 포트
TARGET_GAP_MM = 43.0          # 되돌아갈 목표 jaw gap [mm]
VELOCITY = 50                 # 구동 속도 [1~100 %]
FORCE = 50                    # 그립 힘 [1~100 %]
```

### 단위

- 내부 API는 **0.01mm 카운트** 단위를 사용합니다 (예: `4300` = 43.00 mm). 예제는 mm 단위 입력을 `mm_to_counts()`로 변환합니다.
- 최대 스트로크는 `zimmer.py`의 `gripper_gap_maximum`으로 정의되며 모델에 따라 조정이 필요합니다.

---

## API / 인터페이스

### Zimmer 클래스 주요 메서드

| 메서드                                 | 파라미터                    | 설명                                                    |
| -------------------------------------- | --------------------------- | ------------------------------------------------------- |
| `connect(ip, port)`                    | IO-Link 마스터 IP, 포트     | Modbus TCP 연결. 실패 시 에러 출력 후 프로세스 종료     |
| `disconnect()`                         | —                           | 통신 스레드 정지 및 연결 해제                           |
| `init()`                               | —                           | 파라미터 전송 및 초기화. 완전 열림 상태로 이동 (블로킹) |
| `grip(jaw_gap=-1, sync=True)`          | jaw gap [0.01mm], 동기 여부 | 닫기. `-1`이면 완전 닫기                                |
| `release(jaw_gap=-1, sync=True)`       | jaw gap [0.01mm], 동기 여부 | 열기. `-1`이면 완전 열기                                |
| `custom_position(jaw_gap, sync)`       | jaw gap [0.01mm], 동기 여부 | 현재 위치와 비교해 열기/닫기 방향 자동 판단             |
| `outside_homing()` / `inside_homing()` | —                           | 외측/내측 호밍                                          |
| `opt_velocity(v)` / `opt_force(f)`     | 1~100 [%]                   | 속도/힘 설정 (다음 명령부터 적용)                       |
| `get_actual_position()`                | —                           | 현재 조 위치 [0.01mm]                                   |
| `get_status_word()`                    | —                           | StatusWord 값과 16비트 리스트 반환                      |
| `get_diagnosis()`                      | —                           | 진단 코드와 메시지 반환                                 |

### 네트워크 구성

| 항목              | 값                   | 설명                                          |
| ----------------- | -------------------- | --------------------------------------------- |
| IO-Link 마스터 IP | 192.168.0.253 (예시) | Zimmer 그리퍼가 연결된 IO-Link 마스터 (Turck) |
| Modbus TCP 포트   | 502                  | 표준 Modbus 포트                              |
| Modbus Slave ID   | 16                   | `zimmer.py` 내부 고정값                       |
| Input 레지스터    | 0x0001 ~ 0x0003      | StatusWord / Diagnosis / ActualPosition       |
| Output 레지스터   | 0x0801 ~ 0x0808      | ControlWord 및 프로세스 데이터                |

<details>
<summary>레지스터 맵 / StatusWord 비트 상세</summary>

#### Output 레지스터 (호스트 → 그리퍼)

| 주소   | 이름                      | 설명                                    |
| ------ | ------------------------- | --------------------------------------- |
| 0x0801 | ControlWord               | 명령 비트 (DataTransfer, MoveToBase 등) |
| 0x0802 | DeviceMode / WorkpieceNo  | 동작 모드 / 데이터셋 번호               |
| 0x0803 | PositionTolerance         | 위치 허용 오차 [0.01mm]                 |
| 0x0804 | GripForce / DriveVelocity | 그립 힘 / 구동 속도 [%]                 |
| 0x0805 | BasePosition              | 외측 기준 위치 [0.01mm]                 |
| 0x0806 | ShiftPosition             | 중간 시프트 위치 [0.01mm]               |
| 0x0807 | TeachPosition             | 교시 저장 위치 [0.01mm]                 |
| 0x0808 | WorkPosition              | 내측 목표 위치 [0.01mm]                 |

#### Input 레지스터 (그리퍼 → 호스트)

| 주소   | 이름           | 설명                  |
| ------ | -------------- | --------------------- |
| 0x0001 | StatusWord     | 상태 비트             |
| 0x0002 | Diagnosis      | 진단 코드             |
| 0x0003 | ActualPosition | 현재 조 위치 [0.01mm] |

#### StatusWord 주요 비트

| 비트   | 이름             | 의미                 |
| ------ | ---------------- | -------------------- |
| 0x0400 | AtWorkposition   | 내측(닫힘) 위치 도달 |
| 0x0100 | AtBaseposition   | 외측(열림) 위치 도달 |
| 0x0040 | PLCActive        | PLC 통신 활성        |
| 0x0008 | MovementComplete | 이동 완료            |
| 0x0004 | InMotion         | 이동 중              |
| 0x0002 | MotorOn          | 모터 켜짐            |
| 0x0001 | HomingPositionOK | 호밍 완료            |

전체 정의는 [zimmer.py](../zimmer.py) 상단 주석을 참고하세요.

</details>

---

## 문제 해결

### 1. 연결 실패

증상:

```
Connection to (192.168.0.253, 502) failed: timed out
[ERROR] Not connected gripper
```

해결:

```bash
# IO-Link 마스터(그리퍼 아님) 네트워크 확인
ping 192.168.0.253

# Modbus 포트 확인
nc -zv 192.168.0.253 502
```

`GRIPPER_IP`는 그리퍼 자체가 아니라 그리퍼가 연결된 IO-Link 마스터의 IP입니다. 마스터의 실제 IP와 호스트의 서브넷 설정을 확인하세요.

### 2. 명령 후 무한 대기

증상:

```
[ ZIMMER ] 4. Grip move to baseposition 출력 없이 멈춤
```

해결:

- 이미 열린 상태에서 `release()`를 호출하면 이동이 시작되지 않아 완료 대기에서 멈춥니다. 초기화 직후에는 `grip()`부터 호출하세요.
- `init()` 직후 바로 명령을 보내면 진행 중인 시퀀스를 덮어씁니다. `gripper_send_flag`가 `False`가 될 때까지 대기 후 다음 명령을 보내세요.

### 3. 그리퍼가 움직이지 않음 (진단 코드 확인)

증상:

```
이동 명령 후 위치 변화 없음
```

해결:

```python
code, msg = gripper.get_diagnosis()
print(f"0x{code:04X}: {msg}")
```

| 코드   | 의미                    | 조치                            |
| ------ | ----------------------- | ------------------------------- |
| 0x0001 | 모터 컨트롤러 꺼짐      | `init()` 재실행                 |
| 0x0305 | 위치 측정 시스템 미참조 | `outside_homing()` 등 호밍 수행 |
| 0x0402 | Jam (끼임)              | 장애물 제거 후 재시도           |

---

## 라이선스

이 프로젝트는 Apache License 2.0으로 배포됩니다. 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.

---

## Maintainer

hhanoo (woo980711@gmail.com)
