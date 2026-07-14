# =========================================================================
# Base Image
# =========================================================================
# - https://hub.docker.com/r/python/
FROM python:3.10-slim

# - Build dependencies
ARG DEBIAN_FRONTEND=noninteractive

# =========================================================================
# Locale and Font
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    && rm -rf /var/lib/apt/lists/* \
    && echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen \
    && locale-gen \
    && dpkg-reconfigure --frontend=noninteractive locales

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# =========================================================================
# Essential Packages
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-transport-https \
    apt-utils \
    ca-certificates \
    curl \
    git \
    gnupg2 \
    lsb-release \
    sudo \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# C++ Build Dependencies
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    libssl-dev \
    libffi-dev \
    libxml2-dev \
    libxslt-dev \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Qt / GUI Libraries
# =========================================================================
# - PySide6 (Qt6) runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # - OpenGL
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    \
    # - X11 base libraries
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    libxrandr-dev \
    libxss-dev \
    libglib2.0-0 \
    \
    # - X11 + XCB (Qt "xcb" platform plugin dependency)
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
    # - Font & Text Rendering
    libfontconfig1-dev \
    libfreetype6-dev \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# USB / Serial
# =========================================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    libdbus-1-3 \
    libdbus-1-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*

# =========================================================================
# Python
# =========================================================================
# - Upgrade pip
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel

# - Application dependencies
RUN pip3 install --no-cache-dir \
    PySide6 \
    "pymodbus[serial]==3.6.9"

# =========================================================================
# Workspace Setup
# =========================================================================
# - Workspace
WORKDIR /workspace

# - Commands
COPY ./commands.sh /etc/profile.d/commands.sh
RUN chmod 644 /etc/profile.d/commands.sh && \
    echo 'source /etc/profile.d/commands.sh' >> /etc/bash.bashrc

# - Entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# - Default command
CMD ["bash"]
