# Python 3.10 기반 이미지 ------------------------------------------------------------
# https://hub.docker.com/r/python/
FROM python:3.10-slim

# 권한 변경 ------------------------------------------------------------
USER root

# 기본 시스템 도구 및 패키지 관리자  ------------------------------------------------------------
RUN apt-get clean && apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
    apt-transport-https \
    apt-utils \
    ca-certificates \
    gnupg2 \
    lsb-release \
    sudo \
    curl \
    wget \
    git \
    vim \
    locales \
    && rm -rf /var/lib/apt/lists/*

# C/C++ 컴파일 필수 도구 ------------------------------------------------------------
RUN apt-get clean && apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    pkg-config \
    libssl-dev \
    libffi-dev \
    libxml2-dev \
    libxslt-dev \
    && rm -rf /var/lib/apt/lists/*

# GUI 및 그래픽 라이브러리 (PySide6용) ------------------------------------------------------------
RUN apt-get clean && apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
    # OpenGL
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    \
    # X11 기본 라이브러리
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    libxrandr-dev \
    libxss-dev \
    libglib2.0-0 \
    \
    # X11 + XCB 관련 (Qt "xcb" plugin dependency)
    libx11-xcb1 \
    libxcb-util1 \
    libxcb-render0 \
    libxcb-shape0 \
    libxcb-shm0 \
    libxcb-xfixes0 \
    libxcb-sync1 \
    libxcb-randr0 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-icccm4 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxcb-cursor0 \
    libxkbcommon-x11-0 \
    \
    # Font & Text Rendering
    libfontconfig1-dev \
    libfreetype6-dev \
    \
    && rm -rf /var/lib/apt/lists/*

# USB 통신 ------------------------------------------------------------  
RUN apt-get clean && apt-get update && apt-get upgrade -y && apt-get install --no-install-recommends -y \
    libdbus-1-3 \
    libdbus-1-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*

# pip 업그레이드 ------------------------------------------------------------
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel

# PySide6 설치 ------------------------------------------------------------
RUN pip install --no-cache-dir \
    PySide6 \
    pymodbus==3.6.9

# 로케일 설정 ------------------------------------------------------------
RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen && \
    dpkg-reconfigure --frontend=noninteractive locales
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV LANGUAGE=en_US:en

# 워크스페이스 및 엔트리포인트 설정 ------------------------------------------------------------
WORKDIR /workspace
CMD ["bash"]