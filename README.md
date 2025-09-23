# Gripper Manager

## 📁 Project Structure

```
Gripper_manager/
├── console_colors.py               # Console color constants for terminal output
├── docker-compose.yml              # Docker Compose configuration
├── docker-run.sh                   # Docker management & run script
├── Dockerfile                      # Docker build configuration
├── koras.py                        # KORAS gripper control module
├── koras_window.py / .ui           # KORAS window implementation (Python, UI)
├── zimmer.py                       # Zimmer gripper control module
├── zimmer_window.py / .ui          # Zimmer window implementation (Python, UI)
└── README.md                       # Project documentation
```

## 🔧 Environment & Installation

### 🐳 Docker Environment Setup

<details>
<summary>Docker Environment Guide</summary>

#### 1. Pull Docker Image

```bash
sudo docker pull hhanoo/keti:gripper-manager
```

#### 2. Run Docker Container using Docker Compose

```bash
cd /Gripper_manager
./docker-run.sh [COMMAND]
```

> ### Options
>
> | Command   | Description                             |
> | --------- | --------------------------------------- |
> | `zimmer   | 🤖 Run Zimmer Gripper Manager GUI       |
> | `koras    | 🤖 Run KORAS Gripper Manager GUI        |
> | -         |                                         |
> | `start`   | 🟢 Start Docker Container               |
> | `stop`    | 🔴 Stop Docker Container                |
> | `restart` | 🔄 Restart Docker Container             |
> | `logs`    | 📋 Check Docker Logs                    |
> | `status`  | 📊 Check Container Status               |
> | `shell`   | 🐚 Access Docker Container (bash shell) |
> | `exit`    | ❌ Exit                                 |

</details>

## 🚀 Usage

### 🐳 Docker

#### 1. Start Services

```bash
# Start all services in background
./docker-run.sh start
```

#### 2. Run Application

```bash
# Run the GUI application
./docker-run.sh zimmer
./docker-run.sh koras
```

### 🐍 Python (Manual)

#### 1. Install Dependencies

```bash
# Install required packages
pip install PySide6 pymodbus==3.6.9
```

#### 2. Run Application Manually

```bash
# Run manually (outside Docker)
python3 zimmer_window.py
python3 koras_window.py
```

## ✨ Main Features

- **Gripper Connection/Disconnection**: Modbus TCP/IP-based gripper communication
- **Zimmer Gripper Control**: Support for 2-finger and 3-finger Zimmer grippers
- **Real-time Control**: Grip/Release operations with position feedback
- **Position Monitoring**: Real-time gripper jaw position display
- **Force & Velocity Control**: Adjustable gripper force and movement speed
- **Multi-step Communication**: Robust handshake protocol for reliable operation
- **GUI Interface**: PySide6-based intuitive control interface

## ⚙️ Configuration

### 1. Dependencies

- **Python 3.10**: Base runtime environment
- **PySide6**: Qt6-based GUI framework for Python
- **pymodbus 3.6.9**: Modbus TCP/IP communication library
- **Modbus TCP/IP**: Communication protocol for gripper control
- **Docker & Docker Compose**: Containerized development environment

## 📝 Notes

- Ensure gripper is powered on and network accessible
- X11 forwarding must be enabled for GUI in Docker

## 🖥️ Screen Shot

- Zimmer GUI

  ![Zimmer GUI](docs/zimmer_gui.png)

