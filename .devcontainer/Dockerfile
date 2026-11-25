# Teljes braking_system workspace (C++ node-ok + aeb_perception) EGY
# devcontainerben, saját, friss ROS2 Humble-lel - lásd README.md
# "devcontainer" szekcióját arról, miért váltottunk a sima host-venv-ről
# erre (a host apt-csomagjaival való ütközés, pl. a régi python3-coverage
# vs. numba hiba, ami csak a venv-es próbán jött elő).
FROM osrf/ros:humble-ros-base

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONUNBUFFERED=1

# Build/colcon eszközök + a Python ML-stack system libjei (ugyanazok, mint
# korábban a venv-es INSTALL.md-ben - open3d/opencv/mmcv ezekre linkel).
RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        build-essential \
        ca-certificates \
        ffmpeg \
        ninja-build \
        git \
        libgl1 \
        libglib2.0-0 \
        libsm6 \
        libxext6 \
        libxrender1 \
        openssh-client \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool \
        sudo \
        wget \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

# A braking_system 3 külön repóból kér message-csomagokat (NEM a
# braking_system repóban vannak, nincs .gitmodules/.repos rájuk - lásd
# README.md "Amit a braking_system forrásából megerősítettem" szekció).
# Ezek stabil, külső függőségek, ezért IDE, image-buildkor bakeoljuk be -
# a braking_system maga NEM ide kerül, azt a devcontainer.json
# bind-mountolja, hogy a host-on szerkesztve azonnal látszódjon.
ENV ROS2_WS=/workspace/ros2_ws
RUN mkdir -p ${ROS2_WS}/src
WORKDIR ${ROS2_WS}/src

# Verziók rögzítve pontosan a tanári/csapat workspace jelenlegi
# állapotára (2026-11-25), hogy ne egy véletlenül frissebb upstream
# commit-tal próbáljon lefordulni valaki más gépén.
RUN git clone https://github.com/jkk-research/crp_msgs.git \
    && git -C crp_msgs checkout 6e328de \
    && git clone https://github.com/autowarefoundation/autoware_msgs.git \
    && git -C autoware_msgs checkout c0bcee7 \
    && git clone --branch tier4/universe https://github.com/tier4/tier4_autoware_msgs.git \
    && git -C tier4_autoware_msgs checkout c92a340

RUN rosdep init 2>/dev/null || true \
    && rosdep update \
    && . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths . --ignore-src -y --rosdistro humble

# A 3 külső message-repo lefordítása MÁR image-buildkor (gyorsítja a
# devcontainer első indítását) - a braking_system a postCreateCommand-ban
# épül rá, colcon incrementálisan csak az új package-eket fordítja.
RUN . /opt/ros/humble/setup.sh \
    && cd ${ROS2_WS} \
    && colcon build --symlink-install

# Python ML-stack (torch/mmcv/mmdet/mmdet3d stb.) - a SAJÁT CUDA
# runtime-ját hozza (nvidia-*-cu12 csomagok a torch-wheel-ből), a
# hosztnak csak NVIDIA driverre van szüksége (lásd devcontainer.json
# "--gpus=all"), system CUDA toolkit nem kell az image-be.
COPY aeb_perception/requirements.txt /tmp/aeb_perception_requirements.txt
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install -r /tmp/aeb_perception_requirements.txt \
    && rm /tmp/aeb_perception_requirements.txt

RUN chown -R "${USERNAME}:${USERNAME}" "${ROS2_WS}"

USER ${USERNAME}
WORKDIR ${ROS2_WS}
