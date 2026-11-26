# Teljes braking_system workspace (C++ node-ok + aeb_perception) EGY
# devcontainerben, saját, friss ROS2 Humble-lel - lásd README.md
# "devcontainer" szekcióját arról, miért váltottunk a sima host-venv-ről
# erre (a host apt-csomagjaival való ütközés, pl. a régi python3-coverage
# vs. numba hiba, ami csak a venv-es próbán jött elő).
FROM ros:humble-ros-base

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONUNBUFFERED=1

# Build/colcon eszközök + a Python ML-stack system libjei (ugyanazok, mint
# korábban a venv-es INSTALL.md-ben - open3d/opencv/mmcv ezekre linkel).
# FONTOS: itt MÉG NEM töröljük az apt-listákat (/var/lib/apt/lists) - az
# lentebb a rosdep install (ami maga is apt-get install-t hív) elhasalna
# üres csomaglistával. A törlés a fájl VÉGÉN, egyetlen helyen történik,
# az ÖSSZES apt-függő lépés (ez a blokk + a rosdep install) UTÁN.
RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        -o Acquire::Retries=5 \
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
        wget

# Defenzíven: ha a bázis image-ben már foglalt az UID/GID 1000 (előfordul
# némelyik Ubuntu-alapú image-nél), ne hasaljon el a build, csak essen
# vissza egy auto-UID-ra - a devcontainer.json "updateRemoteUserUID"-ja
# futásidőben úgyis a host user-éhez igazítja majd.
RUN (groupadd --gid "${USER_GID}" "${USERNAME}" 2>/dev/null || groupadd "${USERNAME}") \
    && (useradd --uid "${USER_UID}" --gid "${USERNAME}" --create-home --shell /bin/bash "${USERNAME}" 2>/dev/null \
        || useradd --gid "${USERNAME}" --create-home --shell /bin/bash "${USERNAME}") \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

ENV ROS2_WS=/workspace/ros2_ws
RUN mkdir -p ${ROS2_WS}/src

# ============================================================
# Python ML-stack - EZT SZÁNDÉKOSAN A message-repók lefordítása
# ELŐTT telepítjük: a rosidl_generator_py (amit a lentebbi
# autoware_msgs/tier4_autoware_msgs Python-bindingjai használnak)
# BUILD IDŐBEN numpy fejlécekre (numpy/ndarrayobject.h) szorul - ha ez a
# sorrend megfordulna, a colcon build "fatal error: numpy/ndarrayobject.h:
# No such file or directory"-val hasalna el (ez konkrétan előfordult).
# ============================================================
RUN python3 -m pip install --upgrade pip

# setuptools>=80 kidobta a pkg_resources-t, a mmcv (évekkel ezelőtti)
# setup.py-ja viszont még azt importálja - ezért pinnelt, régebbi
# setuptools kell a mmcv-forrásfordításhoz (lásd lentebb).
RUN python3 -m pip install "setuptools==69.5.1" wheel

# torch előre, külön (ugyanaz a verzió, mint a requirements.txt-ben) - a
# mmcv forrásfordítása ez ellen linkel.
RUN python3 -m pip install --resume-retries 50 \
    --extra-index-url https://download.pytorch.org/whl/cu121 \
    torch==2.1.0+cu121 torchvision==0.16.0+cu121 triton==2.1.0

# A mmcv CUDA-s prebuilt wheel-je a download.openmmlab.com-ról (Aliyun
# OSS, Kína) ehhez a hálózathoz tartósan nagyon lassú/megszakadó
# (DIREKTBEN mérve, többször: 0-80 kB/s, néha percekig 0 byte, sosem
# futott végig). A conda-forge-os alternatíva (más CDN, gyors) viszont
# a SAJÁT, conda-forge-os pytorch-buildje ellen van fordítva, ami a mi
# pip-es torch-unkkal ABI-inkompatibilisnek bizonyult.
#
# EZÉRT: a mmcv-t PyPI FORRÁSBÓL fordítjuk (471 KB-os tar.gz, nem a
# 94 MB-os wheel) - ez teljesen kiiktatja a lassú/instabil hálózati
# függőséget. EGYETLEN hátránya: a Docker build-nek NINCS GPU-elérése
# (a --gpus=all csak futásidőben érvényes), így a mmcv ops-ok CPU-only
# módban fordulnak - ezt direktben leteszteltem (8m42s, torch+mmcv
# import OK, ops.nms helyes eredményt ad CPU-n). GPU-s detekcióhoz ezt
# a mmcv-t később, futásidőben (a konténerben, amikor már van GPU)
# kell majd frissíteni a prebuilt CUDA wheel-re - lásd README.md.
# --no-build-isolation: hogy a FENTEBB pinnelt setuptools==69.5.1-et
# használja a build, ne egy friss (pkg_resources nélküli) verziót.
RUN python3 -m pip install --no-build-isolation --no-binary mmcv "mmcv==2.1.0"

# A requirements.txt innen már csak azt telepíti, ami még nincs meg - a
# torch-ot és a mmcv-t pip "Requirement already satisfied"-ként átugorja.
COPY aeb_perception/requirements.txt /tmp/aeb_perception_requirements.txt
RUN python3 -m pip install --resume-retries 50 -r /tmp/aeb_perception_requirements.txt \
    && rm /tmp/aeb_perception_requirements.txt \
    && python3 -c "import torch, mmcv, mmdet, mmdet3d; from mmcv import ops; assert callable(ops.nms); print('ML-stack smoke-teszt OK (CPU-s mmcv-vel).')"

# ============================================================
# A braking_system 3 külön repóból kér message-csomagokat (NEM a
# braking_system repóban vannak, nincs .gitmodules/.repos rájuk - lásd
# README.md "Amit a braking_system forrásából megerősítettem" szekció).
# Ezek stabil, külső függőségek, ezért IDE, image-buildkor bakeoljuk be -
# a braking_system maga NEM ide kerül, azt a devcontainer.json
# bind-mountolja, hogy a host-on szerkesztve azonnal látszódjon.
# ============================================================
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

# Itt, az ÖSSZES apt-függő lépés (system libek + rosdep install) UTÁN
# töröljük a csomaglistákat egyszer, a végén - lásd a fenti megjegyzést.
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# A 3 külső message-repo lefordítása MÁR image-buildkor (gyorsítja a
# devcontainer első indítását) - a braking_system a postCreateCommand-ban
# épül rá, colcon incrementálisan csak az új package-eket fordítja. A
# numpy (és a teljes Python ML-stack) ekkorra MÁR telepítve van (lásd
# fentebb), ezért a rosidl_generator_py Python-bindingek buildje nem
# hasal el numpy fejléc hiányában.
RUN . /opt/ros/humble/setup.sh \
    && cd ${ROS2_WS} \
    && colcon build --symlink-install

RUN chown -R "${USERNAME}:${USERNAME}" "${ROS2_WS}"

USER ${USERNAME}
WORKDIR ${ROS2_WS}
