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

## 7. 2026-10-29 javítások: miért "működött szarul" és mit csináltam

A `bags/` alatt lévő valódi felvételeken (`kocka_car_attack_20261007.mcap`
stb.) direktben lefuttattam a teljes láncot (TF a bag `/tf_static`-jából,
`CenterPointWrapper.infer` a tényleges checkpointtal, GPU-n) a fő node
elindítása nélkül, hogy lássam pontosan mi megy be/ki minden lépésnél. Két
konkrét, mérhető okot találtam arra, hogy a régi kód gyakorlatilag soha nem
publikált semmit, plusz egy sebességi problémát, ami a "nincs szinkronban"
panaszt okozza:

**a) Sebesség/lemaradás ("nincs szinkronban a pontfelhővel")** - a
`perception_node` minden egyes beérkező `luminar_pcl` sweepre a TELJES
10-sweepes puffert (sweep-enként ~81 000 pont, azaz ~810 000 pontot)
újra lefuttatta a modellen, **mérve kb. 1.4-4 másodpercet GPU-n**, miközben
a LiDAR ~10 Hz-cel (0.1s-enként) küld új sweepet. Az `rclpy` alap QoS-e
(`depth=10`, `RELIABLE`) miatt ez egyre nagyobb, folyamatosan növekvő
feldolgozási lemaradást okozott - innen a panasz, hogy a kirajzolt
detekciók nem a pillanatnyi pontfelhőhöz tartoznak. Két változás kellett:
  - **`point_cloud_range`-vágás** a sweep-pufferbe kerülés előtt
    (`preprocessing.py` `SweepBuffer._crop_to_range`) - a luminar_pcl
    360°-ban akár 200+ méterig lát, de a checkpoint ±51.2 m-en kívül
    semmit nem tud kezdeni a pontokkal (a voxelizáló réteg ezeket amúgy is
    csendben eldobta volna, csak már feleslegesen átmentek a teljes
    pillar-pipeline-on).
  - **`num_sweeps` 10-ről 5-re** - a checkpoint nuScenes 20 Hz-es LiDAR-ra
    van hangolva (10 sweep ≈ 0.5s előzmény), a luminar_pcl itt ~10 Hz,
    tehát 10 sweep itt ~1.0s-ot jelentett volna (kétszer kívül a tanítási
    dt-tartományon).
  - Együtt: **~1.4s -> ~0.35s/sweep** (ugyanazon a GPU-n, mérve). Ez még
    nem reál-idejű a 10 Hz-es bemenethez, de nem nő tovább a lemaradás.
  - **A LiDAR-feliratkozás QoS-e `depth=1`-re állítva** - ha a feldolgozás
    lassabb, mint a sweepek érkezése, NE gyűljön fel a sor (amitől a
    lemaradás folyamatosan nőne), hanem mindig a legfrissebb elérhető
    sweepen dolgozzon, a közbeeső, fel nem dolgozott sweepeket dobja el.

**b) A modell gyakorlatilag soha nem talált 'car'-t** - a `kocka_*` bagek
teszt-scene-jein a nyers (score-szűrés előtti) detekciók közt a `score`
tipikusan 0.3 alatt volt, és a `car` osztály (label 0) egyszer sem jelent
meg a top prediction-ök közt az eredeti (10 sweep, vágás nélküli)
beállítással - a `build_detected_objects` car-only szűrője (szándékosan,
lásd 6. szekció) ezért mindent eldobott, NÉMÁN (nem volt log arról, hány
nyers detekciót dobott el). Ennek egy konkrét, mérhető oka volt: a
luminar_pcl `intensity` mezője ezeken a bageken kb. **0-2 közötti**
(UINT8 PointField, de a driver ebben a szűk tartományban publikál), míg a
checkpoint a nuScenes natív **0-255-ös** intensity-skálájára van
kalibrálva. Hozzáadtam egy `intensity_scale` paramétert
(`perception_node.py` `_extract_points`, alapérték a kódban 1.0, de a
`config/params.yaml`-ban **100.0**-ra állítva) - ezzel a 2 tesztelt bagen
a top raw score kb. 0.3-0.4-ről 0.5-0.67-re nőtt, ÉS megjelentek 'car'
osztályú találatok is a nyers predikciók közt, ahol előtte egy sem volt.
**Ez NEM teljes validáció** (2 rövid bag, nincs ground truth) - ha más
felvételeken ez rosszabbnak bizonyulna (pl. több fals pozitív), hangoljátok
vissza a `config/params.yaml`-ban.

Logolás is bővült: minden feldolgozott sweepnél kimegy
(`throttle_duration_sec=2.0`), hány pont ment a modellbe, mennyi ideig
tartott a feldolgozás, hány NYERS detekció volt, és ebből hány lett
publikálva (score_threshold + car-only szűrés után) - ezzel a jövőben nem
kell vakon tippelni, hogy "nem lát semmit" vagy "lát valamit, csak kiszűri".

### 2026-10-29 javítások, 2. kör: "végig ott van előttem egy autó és nem detektálja"

Az első kör javítása után élesben lefuttatva a `kocka_full_20261007.mcap`
felvételen (ahol a célautó TÉNYLEGESEN mozog, nem csak áll) kiderült: az
első pár frame-en helyesen 'car'-nak classifikált, majd onnantól hosszú
szakaszokon keresztül 'truck'/'bus'-nak. Lefuttatva a láncot direktben
(score_threshold=0.3, car-only szűrés, 80 egymást követő valódi frame)
kiderült a konkrét ok: a doboz hossza 4.5m-ről (helyes, szedán alakú)
**11-13m-re ugrott**, amint a célautó elkezdett mozogni - ez a
preprocessing.py-ban már dokumentált "nincs ego-motion-kompenzáció"
hiányosság egy SÚLYOSABB, eddig nem vizsgált változata: nem csak az EGO
mozgása keni el a sweepeket, hanem a MOZGÓ CÉLOBJEKTUM sajátja is - 5
sweepen (~0.5s) át egyszerűen egymásra pakolva az ő pontjai is szétkenődnek
a mozgás útja mentén, a modell egy valóságosnál jóval nagyobb dobozt lát,
és ez átviszi az osztályozást autóról teherautóra/buszra.

Mérve (ugyanazon 80 frame-en, car-only szűrés mellett, hány frame-ben
maradt volna 'car' a kimeneten):

| num_sweeps | 'car' marad |
|---|---|
| 5 (1. kör alapértéke) | 24/80 (30%) |
| 3 | 40/80 (50%) |
| **2 (jelenlegi alapérték)** | **77/80 (96%)** |
| 1 | 80/80 (100%, de nulla dt-infó a sebességbecsléshez) |

`num_sweeps` 5-ről **2**-re csökkentve (lásd `config/params.yaml` és
`perception_node.py` megjegyzése) - majdnem olyan osztályozás-stabil, mint
1 sweep, de legalább egy dt-mérés megmarad a sebességbecsléshez. Ha a
sebesség-kimenetet egyelőre nem használjátok (az ego-kompenzáció sem kész,
lásd lentebb), érdemes lehet 1-re próbálni a saját bagjeiteken.

**FONTOS, ezt NEM oldja meg az odom-alapú ego-motion-kompenzáció** (a
4-5. szekcióban már korábban is nyitva hagyott feladat): az csak az EGO
saját mozgására korrigálna, a célobjektum SAJÁT sebességére nem - ahhoz a
célobjektum sebességét kellene sweepek közt becsülni/kompenzálni, ami
lényegében egy tracker, jóval nagyobb feladat. Amíg ilyen nincs, az
alacsony `num_sweeps` a pragmatikus kompromisszum.

**Amit ez a kör NEM old meg** (tudatosan, a csapat döntése alapján):
- A kimeneti osztályszűrés MARADT car-only (lásd 6. szekció, M1 scope) -
  a `kocka` teszttárgy és bármi más (gyalogos, kerékpáros stb.) szándékosan
  nem kerül ki a `DetectedObjects`-be, csak logolva van, hogy mennyi nyers
  találatot dobtunk el emiatt.
- Az **ego-motion-kompenzáció és az odom-szinkron** (3-4. szekció, "LiDAR-
  only egyszerűsítés") továbbra is hiányzik - ez egy MÁSIK "szinkron"
  probléma (a sweep-puffer nem igazítja egymáshoz a sweepeket ego-mozgás
  szerint), nem azonos a fent javított wall-clock lemaradással. Ha ez is
  kell, az a `message_filters.ApproximateTimeSynchronizer` hozzáadása a
  `perception_node`-hoz, ami még nincs megépítve.
