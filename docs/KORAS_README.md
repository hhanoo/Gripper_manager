# KORAS Gripper Control

**KORAS 모터 구동 그리퍼 Modbus RTU 제어 모듈 (koras.py) 및 예제 (examples/koras_example.py)**

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

KORAS 모터 구동 그리퍼를 Python으로 제어하기 위한 모듈과 최소 예제입니다.  
Zimmer와 달리 게이트웨이 없이 **USB-RS485 시리얼 포트로 Modbus RTU 직결** 통신하며, 진공 그리퍼 모드도 지원합니다.

### 주요 구성요소

- **koras.py**: KORAS 그리퍼 제어 라이브러리. 백그라운드 폴링 스레드가 상태/위치를 주기 갱신
- **examples/koras_example.py**: 연결 → 초기화 → 닫기 → 열기 → 특정 위치 이동 → 연결 해제 시퀀스의 최소 예제

---

## 주요 기능

- **열기/닫기 제어**: `grip()` / `release()`로 완전 닫기·열기, 값(0~1000)을 주면 임의 핑거 위치로 이동
- **진공 그리퍼**: `vacuum(True/False)`로 진공 on/off
- **힘/속도 조절**: `opt_force()` / `opt_velocity()` (모터 토크/속도, 1~100%)
- **상태 모니터링**: 폴링 스레드(10ms)가 상태 비트·모터 위치·전류·핑거 위치·버스 전압 갱신

---

## 시스템 구조

```
┌─────────────────────────────────────┐
│                Host                 │
│  ┌──────────────────┐               │
│  │ koras_example.py │               │
│  └────────┬─────────┘               │
│           ▼                         │
│  ┌───────────────────────────┐      │
│  │         koras.py          │      │
│  │  (polling thread / RTU)   │      │
│  └─────────────┬─────────────┘      │
└────────────────┼────────────────────┘
                 │ Modbus RTU (USB Serial)
                 │ 38400 bps, 8N1, slave 1
                 ▼
┌─────────────────────────┐
│      KORAS Gripper      │
│  (Motor-driven/Vacuum)  │
└─────────────────────────┘
```

---

## 프로젝트 구조

```
Gripper_manager/
├── koras.py                  # KORAS 그리퍼 제어 라이브러리 (Modbus RTU)
├── examples/
│   └── koras_example.py      # 최소 예제 (열기/닫기/특정 위치 이동)
└── docs/
    └── KORAS_README.md       # 이 문서
```

---

## 빠른 시작

```bash
# 0. 프로젝트 루트로 이동
cd Gripper_manager

# 1. 필요 라이브러리 설치 (RTU 시리얼 통신용 pyserial 포함)
pip install "pymodbus[serial]==3.6.9"

# 2. 시리얼 포트 권한 설정 (필요 시)
sudo chmod 666 /dev/ttyUSB0

# 3. examples/koras_example.py 상단 상수 수정 (포트, 목표 위치 등)

# 4. 예제 실행
python3 examples/koras_example.py
```

---

## 시스템 요구사항

### 하드웨어

| 항목              | 사양             | 비고                  |
| ----------------- | ---------------- | --------------------- |
| KORAS 그리퍼      | 모터 구동 / 진공 | Modbus RTU 슬레이브   |
| USB-시리얼 어댑터 | RS485            | /dev/ttyUSB0으로 인식 |

### 소프트웨어 의존성

| 패키지   | 버전  | 용도                           |
| -------- | ----- | ------------------------------ |
| pymodbus | 3.6.9 | Modbus RTU 통신                |
| pyserial | 3.5   | 시리얼 포트 (pymodbus[serial]) |

---

## 실행

```bash
python3 examples/koras_example.py
```

실행하면 아래 순서로 동작하며 각 단계 후 핑거 위치를 출력합니다.

```
[ EXAMPLE ] Initialized -> finger position: 0.0 %
[ EXAMPLE ] Closed -> finger position: 100.0 %
[ EXAMPLE ] Opened -> finger position: 0.0 %
[ EXAMPLE ] Moved to 50.0 % -> finger position: 50.0 %
```

---

## 사용법

### 예제 워크플로우

```
connect ──▶ (자동 초기화) ──▶ grip ──▶ release ──▶ grip(500) ──▶ disconnect
   │             │           │          │            │             │
시리얼 접속   MotorEnable    완전 닫기   완전 열기   50% 위치 이동     MotorStop
          + Initialize                                       + MotorDisable
```

### 라이브러리로 사용하기

```python
from koras import KORAS

gripper = KORAS()
gripper.connect(port_name="/dev/ttyUSB0", slave_id=1)  # MotorEnable + 초기화 포함

gripper.opt_velocity(50)
gripper.opt_force(50)

gripper.grip()          # 완전 닫기
gripper.release()       # 완전 열기
gripper.grip(500)       # 핑거 위치 50 % 로 이동 (0~1000)

gripper.vacuum(True)    # 진공 그리퍼 모드: 진공 on
gripper.vacuum(False)   # 진공 off

gripper.disconnect()    # MotorStop + MotorDisable 포함
```

주의사항:

1. 모든 명령은 완료 신호가 없는 **비동기(fire-and-forget)** 방식입니다. 동작이 끝나길 기다리려면 `time.sleep()` 또는 `m_info` 상태 폴링으로 대기하세요 (예제 코드 참고).
2. `connect()`가 MotorEnable과 초기화(`init()`)까지 수행하므로 별도 초기화 호출이 필요 없습니다.
3. `m_info["status"]["motor_fault"]`가 `True`면 모터 폴트 상태입니다. `init()`으로 재초기화하세요.

---

## 설정

`examples/koras_example.py` 상단 상수로 설정합니다.

```python
PORT_NAME = "/dev/ttyUSB0"  # USB-RS485 어댑터 시리얼 포트
SLAVE_ID = 1                # Modbus slave ID (장비 기본값 1)
TARGET_POS_PERCENT = 50.0   # 이동할 목표 핑거 위치 [0~100 %]
VELOCITY = 50               # 모터 속도 [1~100 %]
FORCE = 50                  # 모터 토크 [1~100 %]
MOVE_WAIT_SEC = 2.0         # 동작당 대기 시간 [sec]
```

### 단위

- 핑거 위치는 **0~1000 카운트 = 0~100 %** 단위입니다 (예: `500` = 50 %). 예제는 % 입력을 `percent_to_counts()`로 변환합니다.
- 시리얼 설정(38400 bps, 8N1)은 `koras.py` 생성자에 고정되어 있습니다.

---

## API / 인터페이스

### KORAS 클래스 주요 메서드

| 메서드                             | 파라미터              | 설명                                                      |
| ---------------------------------- | --------------------- | --------------------------------------------------------- |
| `connect(port_name, slave_id)`     | 시리얼 포트, slave ID | RTU 연결 + MotorEnable + 초기화 + 폴링 시작. 실패 시 종료 |
| `disconnect()`                     | —                     | MotorStop + MotorDisable 후 폴링 정지 및 연결 해제        |
| `init()`                           | —                     | 그리퍼 초기화 명령 전송                                   |
| `grip(grip_distance=-1)`           | 핑거 위치 (0~1000)    | 닫기. `-1`이면 완전 닫기, 값을 주면 해당 위치로 이동      |
| `release(release_distance=-1)`     | 핑거 위치 (0~1000)    | 열기. `-1`이면 완전 열기, 값을 주면 해당 위치로 이동      |
| `vacuum(vacuum=True)`              | bool                  | 진공 그리퍼 on/off                                        |
| `opt_velocity(v)` / `opt_force(f)` | 1~100 [%]             | 모터 속도/토크 설정 (즉시 명령 전송)                      |
| `get_position()`                   | —                     | 모터 위치 반환                                            |
| `get_status()`                     | —                     | 닫힘 상태 여부 반환 (True = 닫힘)                         |
| `m_info`                           | —                     | 상태 비트·위치·전류·핑거 위치(0~1000)·버스 전압 딕셔너리  |

### 통신 구성

| 항목        | 값           | 설명                         |
| ----------- | ------------ | ---------------------------- |
| 프로토콜    | Modbus RTU   | 시리얼 직결                  |
| 포트        | /dev/ttyUSB0 | USB-RS485 어댑터             |
| 통신 속도   | 38400 bps    | `koras.py` 고정값            |
| 데이터 형식 | 8N1          | 데이터 8 / 패리티 N / 스톱 1 |
| Slave ID    | 1            | 장비 기본값                  |

<details>
<summary>명령 코드 / 레지스터 맵 / 상태 비트 상세</summary>

#### 명령 코드 (GrpCmd — Holding Register 0에 기록)

| 코드 | 이름           | 설명                          |
| ---- | -------------- | ----------------------------- |
| 1    | MotorEnable    | 모터 활성화                   |
| 2    | MotorStop      | 모터 정지                     |
| 4    | MotorDisable   | 모터 비활성화                 |
| 101  | Initialize     | 그리퍼 초기화                 |
| 102  | Open           | 완전 열기                     |
| 103  | Close          | 완전 닫기                     |
| 104  | FingerPosition | 핑거 위치 이동 (값: 0~1000)   |
| 106  | VacuumOn       | 진공 on                       |
| 107  | VacuumOff      | 진공 off                      |
| 212  | SetMotorTorque | 모터 토크 설정 (값: 50~100 %) |
| 213  | SetMotorSpeed  | 모터 속도 설정 (값: 1~100 %)  |

#### 레지스터 맵

| 주소 | 방향  | 설명                   |
| ---- | ----- | ---------------------- |
| 0    | Write | 명령 (GrpCmd)          |
| 1    | Write | 명령 값 (Target value) |
| 10   | Read  | 상태 비트              |
| 11   | Read  | 모터 위치 [degree]     |
| 12   | Read  | 모터 전류 [mA]         |
| 13   | Read  | 모터 속도 [rpm]        |
| 14   | Read  | 핑거 위치 (0~1000)     |
| 17   | Read  | 버스 전압 [V]          |

#### 상태 비트 (Input Register 10)

| 비트 | 이름        | 의미             |
| ---- | ----------- | ---------------- |
| 0    | MotorEnable | 모터 활성화됨    |
| 1    | Initialize  | 초기화 완료      |
| 5    | OpenActive  | 열기 동작 중     |
| 6    | CloseActive | 닫기 동작 중     |
| 9    | MotorFault  | 모터 폴트 (에러) |

전체 정의는 [koras.py](../koras.py) 상단 주석을 참고하세요.

</details>

---

## 문제 해결

### 1. 시리얼 포트를 찾을 수 없음

증상:

```
[KORAS] Modbus connection failed.
```

해결:

```bash
# USB 장치 확인
ls /dev/ttyUSB*

# USB 연결 로그 확인
dmesg | tail -20
```

### 2. 시리얼 포트 권한 오류

증상:

```
Permission denied: '/dev/ttyUSB0'
```

해결:

```bash
# 임시 해결
sudo chmod 666 /dev/ttyUSB0

# 영구 해결 (dialout 그룹 추가 후 재로그인)
sudo usermod -aG dialout $USER
```

### 3. 명령을 보내도 반응이 없음

증상:

```
[KORAS] Failed to read registers
```

해결:

- `SLAVE_ID`가 장비 설정과 일치하는지 확인하세요 (기본값 1).
- RS485 배선(A/B 극성)과 통신 속도(38400 bps)를 확인하세요.

### 4. Motor Fault 발생

증상:

```
m_info["status"]["motor_fault"] == True
```

해결:

```python
gripper.init()  # 재초기화
```

끼임 등 물리적 원인 제거 후에도 지속되면 전원을 재인가하세요.

---

## 라이선스

이 프로젝트는 Apache License 2.0으로 배포됩니다. 자세한 내용은 [LICENSE](../LICENSE) 파일을 참조하세요.

---

## Maintainer

hhanoo (woo980711@gmail.com)
