# Gripper Manager

**Modbus TCP/IP 기반 그리퍼 제어 시스템**

[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)
[![PySide6](https://img.shields.io/badge/PySide6-Qt6-green)](https://doc.qt.io/qtforpython/)
[![Modbus](https://img.shields.io/badge/Modbus-TCP/IP-orange)](https://pymodbus.readthedocs.io/)
[![Docker](https://img.shields.io/badge/Docker-Supported-brightgreen)](docker/)

## 목차

- [데모](#데모)
- [개요](#개요)
- [주요 기능](#주요-기능)
- [빠른 시작](#빠른-시작)
- [시스템 요구사항](#시스템-요구사항)
- [설치](#설치)
- [실행](#실행)
- [사용법](#사용법)
- [설정](#설정)
- [문제 해결](#문제-해결)
- [라이선스](#라이선스)

---

## 데모

### 시스템 구조

```
┌──────────────────────────────────────────────────────────────────┐
│                    Gripper Manager System                         │
└──────────────────────────────────────────────────────────────────┘

    Industrial Gripper (Zimmer / KORAS)
              │
              │ Modbus TCP/IP
              ▼
    ┌──────────────────────┐
    │ Gripper Control      │ (Python)
    │ - Modbus TCP Client  │
    │ - Handshake Protocol │
    │ - Position Feedback  │
    └──────────┬───────────┘
               │
               ▼
    ┌──────────────────────┐
    │ GUI Application      │ (PySide6)
    │ - Connection Mgmt    │
    │ - Real-time Control  │
    │ - Position Monitor   │
    └──────────────────────┘
```

### 프로젝트 구조

```
Gripper_manager/
├── zimmer.py                       # Zimmer 그리퍼 제어 모듈
├── zimmer_window.py                # Zimmer GUI 애플리케이션
├── zimmer_window.ui                # Zimmer GUI 레이아웃 (Qt Designer)
├── koras.py                        # KORAS 그리퍼 제어 모듈
├── koras_window.py                 # KORAS GUI 애플리케이션
├── koras_window.ui                 # KORAS GUI 레이아웃 (Qt Designer)
├── console_colors.py               # 터미널 출력 색상 상수
├── pyproject.toml                  # Python 프로젝트 설정
├── uv.lock                         # uv 의존성 잠금 파일
├── docker/                         # Docker 지원
│   ├── Dockerfile                  # Docker 이미지 빌드 설정
│   ├── build.sh                    # Docker 이미지 빌드 스크립트
│   ├── run.sh                      # Docker 컨테이너 실행 스크립트
│   ├── config.sh                   # Docker 설정 (이미지/컨테이너 이름)
│   └── config.sh.example           # Docker 설정 템플릿
├── docs/                           # 문서 및 스크린샷
│   ├── zimmer_gui.png
│   └── koras_gui.png
└── README.md
```

### 스크린샷

- Zimmer GUI

  ![Zimmer GUI](docs/zimmer_gui.png)

- KORAS GUI

  ![KORAS GUI](docs/koras_gui.png)

---

## 개요

### 프로젝트 목적

Gripper Manager는 산업용 그리퍼를 Modbus TCP/IP 프로토콜로 제어하는 GUI 기반 시스템입니다. Zimmer 그리퍼와 KORAS 그리퍼를 지원하며, PySide6 기반의 직관적인 인터페이스를 통해 실시간 그리퍼 제어 및 모니터링을 제공합니다.

### 주요 구성요소

- **zimmer.py / zimmer_window.py**: Zimmer 그리퍼 제어 모듈 및 GUI (2-finger / 3-finger 지원)
- **koras.py / koras_window.py**: KORAS 그리퍼 제어 모듈 및 GUI
- **console_colors.py**: 터미널 컬러 출력 유틸리티

### 적용 가능 영역

- 산업용 로봇 그리퍼 제어
- 자동화 시스템의 그리퍼 통합
- 그리퍼 테스트 및 디버깅
- 연구 개발 및 교육

---

## 주요 기능

- **그리퍼 연결/해제** - Modbus TCP/IP 기반 통신 연결 및 관리
- **Zimmer 그리퍼 제어** - 2-finger 및 3-finger Zimmer 그리퍼 지원
- **실시간 제어** - Grip/Release 동작 및 포지션 피드백
- **위치 모니터링** - 실시간 그리퍼 조 간격(jaw position) 표시
- **힘 및 속도 제어** - 그리퍼 힘(Force)과 이동 속도(Velocity) 조절
- **다단계 통신** - 안정적인 동작을 위한 핸드셰이크 프로토콜
- **GUI 인터페이스** - PySide6 기반 직관적인 제어 인터페이스
- **Docker 지원** - 빌드 및 실행 스크립트 포함, 원클릭 컨테이너 환경

---

## 빠른 시작

### Option 1: Docker (권장)

```bash
# 1. 저장소 클론
git clone https://github.com/hhanoo/Gripper_manager.git ~/Gripper_manager
cd ~/Gripper_manager

# 2. Docker 이미지 빌드
cd docker
./build.sh

# 3. Docker 컨테이너 실행
./run.sh

# 4. 컨테이너 내에서 애플리케이션 실행
python3 zimmer_window.py    # Zimmer 그리퍼
python3 koras_window.py     # KORAS 그리퍼
```

### Option 2: Native Installation

```bash
# 1. 저장소 클론
git clone https://github.com/hhanoo/Gripper_manager.git ~/Gripper_manager
cd ~/Gripper_manager

# 2. 의존성 설치 (uv 사용)
curl -LsSf https://astral.sh/uv/install.sh | sh
uv sync --frozen

# 3. 실행
uv run python zimmer_window.py
uv run python koras_window.py
```

---

## 시스템 요구사항

### 필수

- **OS**: Ubuntu 22.04 LTS (또는 호환 Linux)
- **Python**: 3.10+

### 하드웨어

- **네트워크**: 그리퍼와 통신 가능한 Ethernet 연결
- **그리퍼**: Zimmer 또는 KORAS 그리퍼 (Modbus TCP/IP 지원)

### 소프트웨어 의존성

- **PySide6**: Qt6 기반 GUI 프레임워크
- **pymodbus 3.6.9**: Modbus TCP/IP 통신 라이브러리

---

## 설치

### Method 1: Docker (권장)

Docker를 사용하면 모든 의존성이 자동으로 설치됩니다:

```bash
cd ~/Gripper_manager/docker
./build.sh
```

빌드 완료 후 `./run.sh`로 컨테이너를 실행하면 모든 환경이 준비됩니다.

> **Docker 설정 변경**: [docker/config.sh](docker/config.sh)에서 이미지 이름, 컨테이너 이름을 변경할 수 있습니다. 최초 실행 시 `config.sh.example`에서 자동 복사됩니다.

### Method 2: uv (가상환경)

```bash
# uv 설치
curl -LsSf https://astral.sh/uv/install.sh | sh

# 의존성 설치 (자동으로 .venv 생성)
cd ~/Gripper_manager
uv sync --frozen
```

> **NOTE:** `.venv` 디렉토리가 자동 생성되며 `pyproject.toml` / `uv.lock` 기반으로 의존성이 설치됩니다.

> **VSCode 설정**: `python.defaultInterpreterPath`를 `${workspaceFolder}/.venv/bin/python`으로 설정하세요.

### Method 3: pip (글로벌)

```bash
pip install --upgrade pip
pip install PySide6
pip install pymodbus==3.6.9
```

---

## 실행

### Docker

```bash
# 1. Docker 컨테이너 실행
cd ~/Gripper_manager/docker
./run.sh

# 2. 컨테이너 내에서 애플리케이션 실행
python3 zimmer_window.py    # Zimmer 그리퍼
python3 koras_window.py     # KORAS 그리퍼
```

### Native (uv)

```bash
cd ~/Gripper_manager

# Zimmer 그리퍼
uv run python zimmer_window.py

# KORAS 그리퍼
uv run python koras_window.py
```

### Native (직접 실행)

```bash
cd ~/Gripper_manager

python3 zimmer_window.py
python3 koras_window.py
```

---

## 사용법

### Basic Workflow

1. **그리퍼 네트워크 확인**

   ```bash
   # 그리퍼 IP 접근 가능 여부 확인
   ping 192.168.3.112
   ```

2. **애플리케이션 실행**

   ```bash
   # Zimmer 그리퍼 GUI 실행
   uv run python zimmer_window.py
   ```

3. **연결 설정**
   - GUI에서 그리퍼 IP 및 포트 입력
   - Connect 버튼으로 Modbus TCP 연결

4. **그리퍼 제어**
   - Grip / Release 버튼으로 동작 제어
   - Force / Velocity 슬라이더로 힘과 속도 조절
   - 실시간 Jaw Position 모니터링

---

## 설정

### Docker 설정

[docker/config.sh](docker/config.sh) 편집:

```bash
# Docker image name (used in build.sh and run.sh)
IMAGE_NAME="project:gripper-manager"

# Docker container name (used in run.sh)
CONTAINER_NAME="Gripper_manager"
```

### 그리퍼 연결 설정

그리퍼 연결 정보는 GUI에서 설정하며, `zimmer_config.json`에 자동 저장됩니다:

```json
{
  "ip": "192.168.3.112",
  "port": "502",
  "jaw_gap": "4300",
  "velocity": "50",
  "force": "50"
}
```

### 주요 파라미터 설명

| Parameter  | Type   | Default         | Description                |
| ---------- | ------ | --------------- | -------------------------- |
| `ip`       | string | `192.168.3.112` | 그리퍼 IP 주소             |
| `port`     | string | `502`           | Modbus TCP 포트 (기본 502) |
| `jaw_gap`  | string | `4300`          | 그리퍼 조 간격 설정값      |
| `velocity` | string | `50`            | 그리퍼 이동 속도 (0-100%)  |
| `force`    | string | `50`            | 그리퍼 힘 (0-100%)         |

---

## 문제 해결

### 1. 연결 문제

#### 1-1. Problem: "그리퍼에 연결할 수 없습니다"

**Solution:**

```bash
# 네트워크 연결 확인
ping 192.168.3.112

# Modbus 포트 접근 확인
nc -zv 192.168.3.112 502

# 방화벽 확인
sudo ufw status
```

- 그리퍼 전원이 켜져 있는지 확인
- Ethernet 케이블 연결 상태 확인
- 같은 네트워크 대역에 있는지 확인

#### 1-2. Problem: "Modbus 통신 타임아웃"

**Solution:**

- 그리퍼 IP 및 포트 번호 재확인
- 다른 프로그램이 같은 포트를 사용하고 있지 않은지 확인
- 그리퍼 재부팅 후 재시도

### 2. GUI 문제

#### 2-1. Problem: "Docker에서 GUI 창이 표시되지 않음"

**Solution:**

```bash
# X11 포워딩 활성화
xhost +local:docker

# DISPLAY 환경변수 확인
echo $DISPLAY
```

#### 2-2. Problem: "PySide6 xcb plugin 오류"

**Solution:**

```bash
# 필요한 XCB 라이브러리 설치 (Docker 외부 실행 시)
sudo apt install -y \
  libxcb-xinerama0 \
  libxcb-cursor0 \
  libxkbcommon-x11-0
```

### 3. 설치 문제

#### 3-1. Problem: "PySide6 설치 실패"

```bash
# pip 업그레이드 후 재설치
pip install --upgrade pip setuptools wheel
pip install PySide6
```

#### 3-2. Problem: "pymodbus 버전 호환 문제"

```bash
# 정확한 버전 설치
pip install pymodbus==3.6.9
```

---

## 라이센스

TBD

---

**Maintainer**: hhanoo (woo980711@gmail.com)
