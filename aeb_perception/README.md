# aeb_perception — összerakási útmutató

## 1. Hova kerül ez a mappa

Ez a mappa egy teljes ROS2 (ament_python) package. Bemásolod a ROS2
workspace `src/` alá:

```
ros2_ws/
└── src/
    └── aeb_perception/      <- ez a mappa, pontosan így
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/aeb_perception
        ├── aeb_perception/
        │   ├── __init__.py
        │   ├── model_wrapper.py
        │   ├── preprocessing.py
        │   ├── postprocessing.py
        │   ├── perception_node.py
        │   └── adapter_node.py
        ├── config/params.yaml
        └── launch/perception.launch.py
```

## 2. Hogy gondolta a tanár a "mehet külön"-t

ROS2-ben a node-ok önálló folyamatok, a DDS middleware hálózaton (UDP
multicast) köti őket össze topicokkal - nem közös memórián/processen.
Ebből két gyakorlati opció van, válassz:

**A) A perception node ott marad, ahol a GPU/devcontaineres env van**
(akár a Windowsod WSL2-je, akár egy másik Linux gép GPU-val), a Linux
22.04-es ROS-gép csak feliratkozik a topicjára. Feltétel: a két gép
lássa egymást a hálózaton, és ugyanazon `ROS_DOMAIN_ID`-n legyenek.
Ehhez a meglévő `.devcontainer/gpu` stackbe (ami már Python 3.10 + torch
+ mmdet3d) csak az `rclpy`-t és a hozzá tartozó message-csomagokat
(`sensor_msgs`, `nav_msgs`, `tf2_ros`, `autoware_perception_msgs`) kell
feltenni - a ROS2 disztró Python-verziójának egyeznie kell a
devcontaineres Python 3.10-zel (Humble alapból 22.04-en pont 3.10-et
használ, ez jó hír, nem kell külön konténer a disztró-ütközés miatt).

**B) Áthozod a checkpointot + configot a Linux 22.04-es gépre**, és ott
építed fel ugyanazt a `.devcontainer/gpu` image-et (ha annak a gépnek
is van GPU-ja - CPU-n valószínűleg nem fér bele a late­ncy-keretbe).
Ez a "tiszta" megoldás, de csak akkor éri meg, ha a célgépen is van
NVIDIA-kártya.

Mindkét esetben a `perception_node` és az `adapter_node` külön
folyamatként fut, akár külön gépen is - ezért mondhatta a tanár, hogy
"mehet külön": a topic-interfészen (`/perception/detections`,
`/novatel/oem7/odom`) keresztül semmi nem változik abból, hogy a kettő
fizikailag hol fut.

## 3. Build és futtatás

```bash
cd ros2_ws
colcon build --packages-select aeb_perception
source install/setup.bash

# állítsd be a config/params.yaml-ban a model_config és checkpoint
# elérési útját a tényleges helyre, MIELŐTT elindítod

ros2 launch aeb_perception perception.launch.py
```

Ha csak a perception node-ot akarod futtatni (pl. a GPU-s gépen, az
adapter máshol fut):
```bash
ros2 run aeb_perception perception_node --ros-args --params-file config/params.yaml
```

## 4. LiDAR-only egyszerűsítés (jelenlegi állapot)

A `perception_node` jelenleg **csak LiDAR-t** használ, az odomot
(`/novatel/oem7/odom`) NEM - ok: az eredeti terv a LiDAR-sweepet és az
odom-mintát időben össze kellett volna illesztenie (két aszinkron topic
szinkronizálása, pl. `message_filters.ApproximateTimeSynchronizer`-rel),
ez még nincs megépítve. Ennek két közvetlen következménye van:

1. A sweep-puffer (10 sweep) NEM kompenzál ego-mozgásra - a pontok
   simán egymás mellé kerülnek `dt`-vel megjelölve, forgatás/eltolás
   nélkül (lásd `preprocessing.py`).
2. A kimeneti `DetectedObjects.twist` NYERS, base_link-relatív sebesség -
   az ego-sebesség NINCS hozzáadva (lásd `postprocessing.py`). Ha a
   downstream (`adapter_node` / `behavior_planner`) abszolút sebességet
   vár, ezt valahol pótolni kell, amíg ez vissza nem kerül.

Konkrét példaértékekkel végigkövetve a teljes láncot (minden topic
be/kimenete) lásd a `SAMPLES.md` fájlt.

Az `adapter_node` és a `config/params.yaml` `adapter_node` szekciója
továbbra is használja az odomot - az a `crp_msgs/Ego` üzenet önálló
republikálásához kell, nem a LiDAR-ral való fúzióhoz, ezért az a
szinkron-probléma nem érinti.

## 5. Amit a braking_system forrásából megerősítettem (már NEM TBD)

A `VVT-SZE/braking_system` repó (`behavior_planner/src/behaviorPlanner.cpp`
és `config/params.yaml`) alapján:

- **`critical_distance = 200.0 méter`** (explicit kommentezve a
  params.yaml-ban, "# meters") - a spec.md nyitott kérdése lezárva.
- **`publish_rate = 50.0 Hz`** - megegyezik a spec sejtésével.
- **A behavior_planner bemeneti topicjai: `"scenario"` és `"ego"`**
  (relatív, nincs namespace, nincs top-level bringup launch a repóban) -
  ezt állítottam be a `config/params.yaml`-ban `scenario_topic`/`ego_topic`
  default értékének, NEM a korábbi `/plan/scenario`/`/plan/ego` tippemet.
- **`crp_msgs::Scenario.local_moving_objects.objects`** pontosan
  `autoware_perception_msgs::msg::PredictedObject` lista, és
  **`crp_msgs::Ego`**-nak `.pose.pose` / `.twist.twist` / `.accel.accel`
  mezői vannak - ezt az `adapter_node.py` már a tényleges struktúra
  szerint tölti ki és publikálja (nincs többé kikommentezve).

**Ami MARADT nyitva**: a `crp_msgs` csomag **forrása** nincs a
`braking_system` repóban - csak `find_package(crp_msgs REQUIRED)` és
`#include <crp_msgs/msg/...>` van benne, se `.gitmodules`, se `.repos`
fájl nem mutat rá. Valahol a ti ROS2 workspace-etekben (vagy a tanári
infrastruktúrában) már léteznie kell, különben a `braking_system` maga
sem fordulna le nálatok. Derítsd ki:
```bash
ros2 pkg prefix crp_msgs   # vagy: colcon list | grep crp_msgs
```
és onnantól tudod, melyik workspace-be/repóba kell az `aeb_perception`-t
is tenni, hogy lássa ugyanazt a `crp_msgs`-t.

**Amire nincs forrás, csak a spec.md-re hagyatkozom:**
1. `ros2 interface show autoware_perception_msgs/msg/DetectedObjects` és
   `DetectedObject` - a `postprocessing.py` egy "szokásos" autoware
   mezőstruktúrát feltételez; ha a ti verziótok eltér, ott kell javítani.
2. **A LiDAR-topic neve** (`config/params.yaml` `lidar_topic`) - ez még
   TBD a spec.md szerint (1.8-as task, nyers mérések felderítése).

## 6. Amit NEM old meg ez a kód, és neked kell eldöntened/megépítened

- A `crp_msgs::Scenario`-nak van egy `paths` mezője is (sáv-geometria:
  `left_bound`/`right_bound`), amit a `behaviorPlanner.cpp` a lane-width
  számításához használ (alapértelmezett 4.0 m-re esik vissza, ha üres).
  Az `adapter_node.py` ezt SZÁNDÉKOSAN nem tölti ki - ez nem a perception
  feladata, hanem egy route/lane-komponensé, amit valaki másnak kell
  megépítenie. Ha a `behavior_planner` furcsán szűk lane-width-del szűri
  a kritikus objektumokat, ne itt keresd a hibát.
- A `preprocessing.py` sweep-kompenzációja egyszerűsített (csak 2D, yaw
  körüli forgatás) - ha az ego számottevően dől/bólint a sweepek közötti
  rövid idő alatt, ez pontatlan lesz.
- A sebesség-kompenzáció (`model_vx + ego_vx`) feltételezi, hogy a modell
  kimenete és az ego sebessége ugyanabban az irányban van kifejezve,
  ugyanabban a pillanatban - ez a spec.md 2.9-es nyitott taskja
  ("Ego-mozgás és sebességbecslés"), amit még nem zártatok le.
- Nincs benne tesztelés - a következő lépés a spec.md 1.7-es
  (determinisztikus tesztesetek) és 2.10-es (bag-replay összevetés
  offline vs. élő) task, amivel ki lehet deríteni, hol hibázik a lánc,
  mielőtt RViz-ben keresnéd.
