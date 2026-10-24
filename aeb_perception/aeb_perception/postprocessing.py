"""Predikciós eredmény -> autoware_perception_msgs/DetectedObjects.

Három dolgot KELL itt helyesen csinálni (spec.md 3. szekció):

1. bbox z: az MMDetection3D az ALSÓ pontot adja vissza, nem a geometriai
   középpontot. Ha ezt elfelejted, minden doboz "a földbe lesz süllyesztve"
   a kimeneten - csendes hiba, semmi nem fog kiabálni miatta.
2. yaw -> quaternion.
3. a modell sebessége a LiDAR-frame-hez relatív; a behavior_planner
   abszolút, globális irányú sebességet vár (dv = ego_v - obj_v, lásd
   spec.md 4. szekció) - ELVILEG ehhez hozzá kellene adni az
   ego-sebességet.

   EZ A VERZIÓ EZT NEM TESZI MEG: az odom-ot (ego_vx/ego_vy) kivettük a
   perception_node-ból, mert a LiDAR-sweep és az odom-minta
   timestamp-szinkronizációja még nincs megépítve (lásd
   perception_node.py docstring). Ami itt kimegy a `twist_with_covariance`
   mezőben, az a modell NYERS, LiDAR/base_link-RELATÍV sebessége, NEM
   abszolút/globális sebesség. Ha a downstream (adapter_node /
   behavior_planner) abszolút sebességet vár, ezt a hiányt ott kell
   pótolni (pl. az adapter_node-ban, ahol már van friss odom), amíg ide
   vissza nem kerül a szinkronizált ego-fúzió.

FIGYELEM a mezőnevekre: az autoware_perception_msgs pontos mezőnevei
csomagverziótól függően eltérhetnek. Mielőtt futtatod, ellenőrizd a
tényleges definíciót:
    ros2 interface show autoware_perception_msgs/msg/DetectedObjects
    ros2 interface show autoware_perception_msgs/msg/DetectedObject
Ez a fájl a "szokásos" autoware mezőstruktúrát feltételezi
(ObjectClassification.CAR, DetectedObjectKinematics.pose_with_covariance /
twist_with_covariance, Shape.BOUNDING_BOX) - ha a ti verziótok eltér,
itt kell javítani.
"""
import math
from typing import List

from autoware_perception_msgs.msg import (
    DetectedObject,
    DetectedObjectKinematics,
    DetectedObjects,
    ObjectClassification,
    Shape,
)
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header


# M1 hatókör: csak autó (lásd spec.md 1. szekció). Bővítés később:
# gyalogos, kerékpáros - itt kell majd új bejegyzést felvenni.
_CLASS_NAME_TO_AUTOWARE_LABEL = {
    "car": ObjectClassification.CAR,
}


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def build_detected_objects(
    pred_instances_3d,
    classes: List[str],
    score_threshold: float,
    header: Header,
) -> DetectedObjects:
    msg = DetectedObjects()
    msg.header = header

    scores = pred_instances_3d.scores_3d.detach().cpu().numpy()
    keep = scores >= score_threshold
    boxes = pred_instances_3d.bboxes_3d.tensor[keep].detach().cpu().numpy()
    labels = pred_instances_3d.labels_3d[keep].detach().cpu().numpy().astype(int)
    scores = scores[keep]

    for box, label_idx, score in zip(boxes, labels, scores):
        class_name = classes[label_idx]
        if class_name not in _CLASS_NAME_TO_AUTOWARE_LABEL:
            continue

        x, y, z_bottom, length, width, height, yaw = box[:7]
        z_center = z_bottom + height / 2.0  # <-- a leggyakoribb elfelejtett lépés

        model_vx = float(box[7]) if len(box) >= 9 else 0.0
        model_vy = float(box[8]) if len(box) >= 9 else 0.0

        obj = DetectedObject()
        obj.existence_probability = float(score)

        classification = ObjectClassification()
        classification.label = _CLASS_NAME_TO_AUTOWARE_LABEL[class_name]
        classification.probability = float(score)
        obj.classification = [classification]

        kin = DetectedObjectKinematics()
        kin.pose_with_covariance.pose.position.x = float(x)
        kin.pose_with_covariance.pose.position.y = float(y)
        kin.pose_with_covariance.pose.position.z = float(z_center)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        kin.pose_with_covariance.pose.orientation.x = qx
        kin.pose_with_covariance.pose.orientation.y = qy
        kin.pose_with_covariance.pose.orientation.z = qz
        kin.pose_with_covariance.pose.orientation.w = qw
        kin.has_position_covariance = False

        # NYERS, base_link-RELATÍV sebesség - odom NINCS hozzáadva (lásd a
        # fájl elején lévő megjegyzést). Ha a downstream abszolút sebességet
        # vár, ezt itt vagy lejjebb (adapter_node) kell majd pótolni.
        kin.twist_with_covariance.twist.linear.x = model_vx
        kin.twist_with_covariance.twist.linear.y = model_vy
        kin.has_twist_covariance = False
        obj.kinematics = kin

        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        shape.dimensions = Vector3(x=float(length), y=float(width), z=float(height))
        obj.shape = shape

        msg.objects.append(obj)

    return msg
