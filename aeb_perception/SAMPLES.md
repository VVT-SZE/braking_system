# Minta-üzenetek a teljes láncon (LiDAR-only verzió)

Ez a fájl nem kód - csak arra való, hogy konkrét példaértékekkel lásd, mi
megy ki-be minden node-on. Egy kitalált, de realisztikus jelenetet követ:
az ego áll/lassan halad, 35 méterre előtte egy autó áll (klasszikus
"előttem lefékező autó" AEB-eset).

Ez a verzió a **LiDAR-only** állapotot mutatja (odom kivéve a
perception_node-ból) - ezért a DetectedObjects sebessége itt NYERS,
base_link-relatív érték, nem abszolút.

---

## 1. Bemenet: `sensor_msgs/PointCloud2` (`/luminar_pcl`)

```yaml
header:
  stamp: {sec: 1732450000, nanosec: 100000000}
  frame_id: "luminar_front"
height: 1
width: 34218          # pontok száma ebben a sweepben
fields: [x, y, z, intensity]   # mind float32
point_step: 16
row_step: 547488
data: <bináris, 34218 * 16 byte>
```

A `perception_node` ezt veszi, a `_transform_to_base_link` a statikus
`luminar_front -> base_link` TF-fel forgatja/eltolja (ehhez NEM kell
odom - ez a szenzor fix beépítési helye, nem az ego mozgása).

---

## 2. Kimenet: `autoware_perception_msgs/DetectedObjects` (`/perception/detections`)

```yaml
header:
  stamp: {sec: 1732450000, nanosec: 100000000}   # a LiDAR-mérés ideje
  frame_id: "base_link"
objects:
  - existence_probability: 0.87
    classification:
      - label: 1   # ObjectClassification.CAR
        probability: 0.87
    kinematics:
      pose_with_covariance:
        pose:
          position: {x: 35.2, y: 0.4, z: 0.75}   # <- bbox-közép, NEM alsó pont
          orientation: {x: 0.0, y: 0.0, z: 0.01, w: 0.9999}  # ~1.1° yaw
        has_position_covariance: false
      twist_with_covariance:
        twist:
          linear: {x: -0.3, y: 0.0, z: 0.0}
          # ^ NYERS modell-sebesség, base_link-relatív. Ha az ego áll,
          #   ez közelítőleg az objektum valós sebessége is - de ha az
          #   ego mozog, ez a modell szemszögéből "relatív" marad, az
          #   odom-kompenzáció hiányzik (lásd postprocessing.py).
        has_twist_covariance: false
    shape:
      type: 0   # Shape.BOUNDING_BOX
      dimensions: {x: 4.6, y: 1.9, z: 1.5}   # hossz, szélesség, magasság
```

---

## 3. `adapter_node` kimenetei

### 3a. `crp_msgs/Scenario` (topic: `"scenario"`)

```yaml
header:
  stamp: {sec: 1732450000, nanosec: 100000000}
  frame_id: "base_link"
local_moving_objects:
  objects:
    - classification: [{label: 1, probability: 0.87}]
      kinematics:
        initial_pose_with_covariance:
          pose:
            position: {x: 35.2, y: 0.4, z: 0.75}
            orientation: {x: 0.0, y: 0.0, z: 0.01, w: 0.9999}
        initial_twist_with_covariance:
          twist: {linear: {x: -0.3, y: 0.0, z: 0.0}}
        initial_acceleration_with_covariance:
          accel: {linear: {x: 0.0, y: 0.0, z: 0.0}}   # mindig 0 (nincs mérve)
paths: []   # <- SZÁNDÉKOSAN üres, lásd korábbi megjegyzés (más komponens dolga)
```

### 3b. `crp_msgs/Ego` (topic: `"ego"`)

```yaml
pose:
  pose:
    position: {x: 120.0, y: 8.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
twist:
  twist:
    linear: {x: 8.5, y: 0.0, z: 0.0}   # m/s, ~30.6 km/h
accel:
  accel:
    linear: {x: 0.0, y: 0.0, z: 0.0}   # TBD, lásd adapter_node.py megjegyzés
```

Ez teljesen független az odom-szinkron kérdéstől - az `Ego` üzenet
egyenesen az `/novatel/oem7/odom`-ból jön, nincs LiDAR-lal fuzionálva.

---

## 4. `behavior_planner` kimenetei

### 4a. `tier4_planning_msgs/Scenario` (topic: `"plan/strategy"`)

```yaml
current_scenario: "LONG_EMERGENCY_AVOID"
```

(A `critical_distance = 200 m` és a 35 m-es objektum-távolság miatt ez
biztosan a figyelt tartományban van; a konkrét state a sávon-belüliségtől
és a kiszámolt lassítástól függ - itt egy AVOID-eset van feltételezve.)

### 4b. `crp_msgs/TargetSpace` (topic: `"plan/target_space"`)

```yaml
target_pose:
  pose:
    position: {x: 35.2, y: 0.4, z: 0.75}
relevant_objects:
  - classification: [{label: 1, probability: 0.87}]
    kinematics:
      initial_twist_with_covariance:
        twist: {linear: {x: -0.3, y: 0.0, z: 0.0}}
      initial_acceleration_with_covariance:
        accel: {linear: {x: 0.0, y: 0.0, z: 0.0}}
```

---

## 5. `plan_long_emergency` kimenete

### `autoware_planning_msgs/Trajectory` (topic: `"plan/longEmergency/trajectory"`)

```yaml
header:
  stamp: {sec: 1732450000, nanosec: 100000000}
points:
  - pose: {position: {x: 0.0, y: 0.0, z: 0.0}}
    longitudinal_velocity_mps: 8.5     # pillanatnyi ego-sebesség
  - pose: {position: {x: 5.0, y: 0.0, z: 0.0}}
    longitudinal_velocity_mps: 6.2     # lassítás elkezdve
  - pose: {position: {x: 10.0, y: 0.0, z: 0.0}}
    longitudinal_velocity_mps: 3.1
  - pose: {position: {x: 15.0, y: 0.0, z: 0.0}}
    longitudinal_velocity_mps: 0.0     # megállás a biztonsági távon belül
```

(A `TrajectoryCalculation::calcTrajectory` tényleges pontszáma/alakja a
kódban van, ez csak illusztráció a "csökkenő sebességű pontsor" mintára.)

---

## Mire figyelj emiatt a LiDAR-only egyszerűsítés miatt

- A 2. lépésben publikált sebesség (`twist_with_covariance`) **nem**
  abszolút - ha teszteled, és az ego is mozog, a `behavior_planner`-nek
  átadott `obj_v` torz lesz (az ego-sebességgel eltolva), amíg ez nincs
  pótolva.
- A multi-sweep aggregáció (10 sweep) nincs ego-mozgásra kompenzálva - álló
  teszteknél ez nem számít, de ha menet közben teszteled, a távolabbi
  (régebbi) sweepek pontjai "elcsúszva" kerülnek a modellbe.
- Mindkét hiányosság egy közös okra megy vissza (odom-szinkron hiánya), és
  egy helyen (egy `message_filters.ApproximateTimeSynchronizer` a
  `perception_node`-ban) orvosolható, amikor arra sor kerül.
