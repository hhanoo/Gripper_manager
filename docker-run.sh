#!/bin/bash

# KETI Gripper Manager Docker 실행 스크립트

set -e

echo "================================================================="
echo "KETI Gripper Manager Docker Launcher"
echo "================================================================="

# 프로젝트 이름 정의
PROJECT_NAME="KETI Gripper Manager"
SERVICE_NAME="keti-gripper-system"

# Print Function------------------------------------------------------------
# 색상 정의
RED='\033[91m'
GREEN='\033[92m'
YELLOW='\033[93m'
BLUE='\033[96m'
NC='\033[0m' # No Color

# 함수 정의
print_info() {
    echo -e "${BLUE}[ INFO  ]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ ERROR ]${NC} $1"
}

# Basic Function------------------------------------------------------------
# Docker 설치 확인
check_docker() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker가 설치되지 않았습니다."
        echo "Docker 설치 방법: https://docs.docker.com/get-docker/"
        exit 1
    fi
    
    # Docker Compose v2 (docker compose) 또는 v1 (docker-compose) 확인
    if docker compose version &> /dev/null; then
        DOCKER_COMPOSE_CMD="docker compose"
        print_success "Docker 및 Docker Compose v2 확인됨"
    elif command -v docker-compose &> /dev/null; then
        DOCKER_COMPOSE_CMD="docker-compose"
        print_success "Docker 및 Docker Compose v1 확인됨"
    else
        print_error "Docker Compose가 설치되지 않았습니다."
        echo "Docker Compose 설치 방법: https://docs.docker.com/compose/install/"
        exit 1
    fi
}

# 필요한 디렉토리 생성 (필요한 경우만)
create_directories() {
    print_info "필요한 디렉토리 확인 중..."
    # mkdir -p build
    print_success "디렉토리 확인 완료"
}

# 시스템 시작
start_system() {
    print_info "${PROJECT_NAME} 시스템 시작 중..."
    print_info "X11 지원으로 시작합니다."
    xhost +local:docker
    $DOCKER_COMPOSE_CMD up -d
    print_success "${PROJECT_NAME} 시스템 시작 완료"
}

# 시스템 중지
stop_system() {
    print_info "${PROJECT_NAME} 시스템 중지 중..."
    $DOCKER_COMPOSE_CMD down
    print_success "${PROJECT_NAME} 시스템 중지 완료"
}

# 로그 확인
show_logs() {
    print_info "${PROJECT_NAME} 시스템 로그 확인 중..."
    $DOCKER_COMPOSE_CMD logs -f
}

# 상태 확인
check_status() {
    print_info "${PROJECT_NAME} 시스템 상태 확인 중..."
    $DOCKER_COMPOSE_CMD ps
    
    # X11 상태 확인
    if xhost +local:docker &> /dev/null; then
        echo ""
        print_info "X11 상태 확인:"
        xhost +local:docker
        print_success "X11 상태 확인 완료"
    else
        print_error "X11 상태 확인 실패"
        exit 1
    fi  
}

# 컨테이너 접속
connect_container() {
    print_info "${PROJECT_NAME} 컨테이너 접속 중..."
    
    # 컨테이너가 실행 중인지 확인
    cid="$($DOCKER_COMPOSE_CMD ps -q ${SERVICE_NAME})"
    if [ -z "$cid" ] || [ "$(docker inspect -f '{{.State.Running}}' "$cid")" != "true" ]; then
        print_error "${PROJECT_NAME} 컨테이너가 실행 중이 아닙니다."
        echo "먼저 시스템을 시작하세요: ./docker-run.sh start"
        return 1
    fi
    
    print_success "${PROJECT_NAME} 컨테이너에 접속합니다."
    echo "컨테이너에서 나가려면: exit 또는 Ctrl+D"
    echo "----------------------------------------"
    
    # bash shell로 컨테이너 접속
    xhost +local:docker
    $DOCKER_COMPOSE_CMD exec ${SERVICE_NAME} /bin/bash
    
    print_info "컨테이너 접속이 종료되었습니다."
}

# Run Function------------------------------------------------------------
# Project Run
run_project() {
    print_info "${PROJECT_NAME} Run 중..."
    xhost +local:docker
    $DOCKER_COMPOSE_CMD exec ${SERVICE_NAME} bash -c "
    cd ~/workspace/Gripper_manager
    python3 main_window.py
    "
    print_success "${PROJECT_NAME} Run 완료"
}

# Menu Function------------------------------------------------------------
# Main Menu
show_menu() {
    echo ""
    echo "Available Commands:"
    echo ""
    echo "  run         - Run ${PROJECT_NAME}"
    echo ""
    echo "  1) start    - Docker 컨테이너 시작"
    echo "  2) stop     - Docker 컨테이너 중지"
    echo "  3) restart  - Docker 컨테이너 재시작"
    echo "  4) logs     - 로그 확인"
    echo "  5) status   - 상태 확인"
    echo "  6) shell    - Docker 컨테이너 접속 (bash shell)"
    echo "  0) exit     - 종료"
    echo ""
}

# Main Logic
main() {
    check_docker

    create_directories

    case "${1:-menu}" in
        "run")
            run_project
            ;;
        "start"|"1")
            start_system
            ;;
        "stop"|"2")
            stop_system
            ;;
        "restart"|"3")
            stop_system
            sleep 2
            start_system
            ;;
        "logs"|"4")
            show_logs
            ;;
        "status"|"5")
            check_status
            ;;
        "shell"|"6")
            connect_container
            ;;
        "exit"|"0")
            print_info "시스템을 종료합니다."
            exit 0
            ;;
        "menu"|*)
            show_menu
            read -p "명령어를 선택하세요: " choice
            main "$choice"
            ;;
    esac
}

# 스크립트 실행
main "$@"
