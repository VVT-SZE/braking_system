## 0. Előfeltétel system package-ek (apt)

```bash
sudo apt update
sudo apt install --yes \
    git python3.10-venv python3-pip \
    ffmpeg libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 ninja-build
```

---

## 1. lépés: git clone

```bash
cd ~
git clone https://github.com/VVT-SZE/braking_system.git
git checkout feature/perception-node
```

Csak ez az **egy** clone kell. A `mmdet3d` Python package-kód ÉS a
hozzá tartozó configok rendes PyPI wheel-ként jönnek (lásd 2. lépés,
`mmdet3d==1.4.0` a `requirements.txt`-ben) - nincs se vendorelt forrás a
repóban, se külön letöltendő
[open-mmlab/mmdetection3d](https://github.com/open-mmlab/mmdetection3d)
repo-clone.

---

## 2. lépés: Python-környezet (venv)

A környezet neve **`aeb_env`**.

```bash
python3.10 -m venv --system-site-packages ~/aeb_env
source ~/aeb_env/bin/activate
```

A `--system-site-packages` KRITIKUS: ettől látja a venv a system
ROS2 Humble-ből jövő `rclpy`-t és a ROS message-csomagokat (azokat NEM a
`requirements.txt` telepíti, lásd ott a megjegyzést) - nélküle a
`perception_node` elindításakor `ModuleNotFoundError: rclpy` lenne.

```bash
pip install --upgrade pip
pip install -r ~/vvt_ws_src_braking_system/aeb_perception/requirements.txt
```

Ez EGYETLEN paranccsal mindent telepít, a `mmdet3d`-t is (`mmdet3d==1.4.0`,
rendes PyPI wheel, saját magában hordozza a configjait is - lásd a
`requirements.txt` megjegyzését) - nincs külön lépés, nem kell hozzá
`colcon build` sem. Smoke-teszt:

```bash
python -c "
import torch, mmcv, mmdet, mmdet3d
from mmcv import ops
assert callable(ops.nms)
print('torch', torch.__version__, 'cuda available:', torch.cuda.is_available())
print('mmdet3d', mmdet3d.__version__, mmdet3d.__file__)
"
```

Ha `torch.cuda.is_available()` `False`-t ad, de van GPU-d, lásd lentebb a
"Melyik CUDA-verziót válasszam" szekciót - valószínűleg a torch-wheel CUDA
verziója nem kompatibilis a driverrel.

Ha a `import mmdet3d` ezzel hasal el: `AttributeError: module 'coverage'
has no attribute 'types'` - ez a `--system-site-packages` egy ismert
mellékhatása: a venv látja az Ubuntu 22.04 apt-os, régi (6.2)
`python3-coverage`-ját, amiben nincs `coverage.types`, és a `numba`
(amit a `mmdet3d` importál) ezt feltétlenül használja. A
`requirements.txt`-ben lévő `coverage>=7.0` sor ezt lefedi (a venv saját,
pip-telepített verziója felülírja a system-eset) - ha régebbi
`requirements.txt`-vel telepítettél, csak futtasd újra a `pip install -r
requirements.txt`-t.

---

## 3. lépés: colcon build + futtatás

A checkpointot (`centerpoint_pillar02_nuscenes.pth`, ~24 MB) nem kell
beszerezni - a repóban van (`config/checkpoints/`), relatív útvonallal,
amit a `perception_node` a package share-dir-hez képest old fel. A
`model_config` a telepített `mmdet3d` csomag saját bundle-ölt
configjaira mutat (lásd `config/params.yaml` megjegyzése) - ezzel sincs
dolgod. Ha más checkpointot/configot akarsz használni, írd át a két
érintett sort a `config/params.yaml`-ban.

```bash
cd ~/vvt_ws_src_braking_system/..
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

./vvt_ws_src_braking_system/start.sh
```

---

## Melyik CUDA-verziót válasszam a kocsin?

A `requirements.txt` jelenleg `torch==2.1.0+cu121`-et pinnel - ez a
pip-wheel a SAJÁT CUDA 12.1 runtime-ját hozza magával (`nvidia-*-cu12`
csomagok), tehát a hoston **nem kell** külön system CUDA toolkit, csak egy
elég friss **NVIDIA driver**.

1. Nézd meg a kocsin: `nvidia-smi` - a jobb felső sarokban kiírt
   "CUDA Version: X.Y" a driver által **támogatott maximális** CUDA
   runtime verzió (nem azt mutatja, mi van telepítve).
2. Ha ez **>= 12.1**: a `requirements.txt` változatlanul jó.
3. Ha **< 12.1** (régebbi driver): válts régebbi CUDA-build-re:
   - cseréld a `requirements.txt` tetején a `--extra-index-url`-t és a
     `torch`/`torchvision` verziót egy a [pytorch.org](https://pytorch.org/get-started/locally/)
     telepítő-választójában megadott, a driverednek megfelelő `cuXXX`
     build-re,
   - cseréld a `mmcv`-hez tartozó `--find-links` URL-t UGYANARRA a
     `cuXXX/torchY.Y` kombinációra
     (`https://download.openmmlab.com/mmcv/dist/<cuXXX>/<torchY.Y>/index.html`),
   - telepítsd újra a venv-et (2. lépés) az új torch ellen.
4. Ha nincs NVIDIA GPU a célgépen: `device: "cpu"` a `params.yaml`-ban -
   ez FUT, de lásd a repo többi dokumentációját (README.md) arról, hogy ez
   valós-idejű AEB-láncban miért nem praktikus (mért ~0.35s/sweep GPU-n is,
   CPU-n ennek sokszorosa várható).
