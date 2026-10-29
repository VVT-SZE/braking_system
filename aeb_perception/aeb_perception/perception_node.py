"""Perception node: LiDAR PointCloud2 -> autoware_perception_msgs/DetectedObjects.

Ez a node KÜLÖN futtatható a többi ROS2 node-tól - akár külön gépen, külön
konténerben, akár Windows alatt WSL2-ben - amíg ugyanazon a ROS2 domainen
van (ugyanaz a ROS_DOMAIN_ID, hálózatilag elérhetők egymásnak UDP
multicasttal). A DDS middleware nem törődik azzal, melyik folyamat/gép
publikál egy topicot.

EZ A VERZIÓ CSAK LIDAR-T HASZNÁL, odomot NEM. Az eredeti terv (odom-mal
ego-motion-kompenzált multi-sweep + abszolút sebesség) két aszinkron topicot
fuzionált volna (PointCloud2 + Odometry), ami azt jelenti, hogy minden
beérkező LiDAR-sweephez meg kellett volna találni az IDŐBEN LEGKÖZELEBBI
odom-mintát (interpoláció vagy legalább "legutóbbi előtte" illesztés) - ez a
timestamp-szinkronizáció még nincs megépítve, ezért ELSŐ KÖRBEN kivettük az
odom-függést:

- a sweep-puffer NEM kompenzál ego-mozgásra (lásd preprocessing.py) - a
  pontok simán a saját mérési idejük szerinti dt-vel kerülnek a modell
  bemenetébe, forgatás/eltolás nélkül. Ha az ego gyorsan kanyarodik/gyorsul
  a sweep-ablakon (num_sweeps) belül, ez pontatlanságot okoz a multi-sweep
  aggregációban.
- a kimeneti sebesség NEM lesz ego-sebességgel összeadva (lásd
  postprocessing.py) - ami kimegy, az a modell LiDAR/base_link-relatív
  sebessége, NEM abszolút/globális sebesség. A downstream (adapter_node /
  behavior_planner) ezt figyelembe kell vegye, amíg ez nincs pótolva.

Amikor visszatesszük az odomot: a megoldás egy time-synchronizer (pl.
`message_filters.ApproximateTimeSynchronizer` a PointCloud2 és Odometry
között), NE a jelenlegi "utolsó kapott odom-üzenet" mintázat, mert az
névleg szinkronizál, valójában csak a legutóbb érkezett, tetszőlegesen régi
odom-mintát ragasztja a jelen LiDAR-sweephez.

Paraméterek (lásd config/params.yaml): lidar_topic, output_topic,
base_frame, model_config, checkpoint, device, score_threshold, num_sweeps.
"""
import time

import numpy as np
import rclpy
from autoware_perception_msgs.msg import DetectedObjects
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

from aeb_perception.model_wrapper import CenterPointWrapper
from aeb_perception.postprocessing import build_detected_objects
from aeb_perception.preprocessing import SweepBuffer, TimedSweep


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("aeb_perception_node")

        # TBD, lásd spec.md 6. szekció, nyitott kérdés #1 - a tényleges
        # topicot a csapatnak el kell döntenie (1.8-as task).
        self.declare_parameter("lidar_topic", "/luminar_pcl")
        self.declare_parameter("output_topic", "/perception/detections")
        # FONTOS: a valódi járművön a statikus TF-fában ("lexus3/base_link
        # -> /luminar_lidar_0", lásd a felvett bagek /tf_static-ja) csak a
        # "lexus3/base_link" létezik - a namespace nélküli "base_link"
        # SEHOL nincs tényleges TF-ként broadcastolva (más node-ok csak
        # header.frame_id "base_link" LABEL-ként használják, anélkül, hogy
        # valódi transzformot publikálnának hozzá). Ha ez a paraméter
        # params-fájl nélkül futna "base_link"-kel, a TF-lookup minden
        # egyetlen sweepnél elhasalna, és a node csendben sosem publikálna
        # semmit (pontosan ez a hiba derült ki a javítás közben végzett
        # integrációs teszten) - ezért az alapérték itt már a valódi,
        # a params.yaml-ban is használt "lexus3/base_link".
        self.declare_parameter("base_frame", "lexus3/base_link")
        self.declare_parameter("model_config", "")
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("score_threshold", 0.3)
        # 2026-10-29: 10-ről 2-re csökkentve - lásd config/params.yaml
        # megjegyzése, ez NEM csak a dt-tartomány miatt van, hanem egy
        # súlyosabb okból: mozgó célobjektumnál (nem csak ego-mozgásnál) a
        # sweepek egyszerű egymásra-pakolása a doboz MÉRETÉT torzítja fel,
        # ami elrontja az osztályozást ('car' -> 'truck'/'bus').
        self.declare_parameter("num_sweeps", 2)
        # (x_min, y_min, z_min, x_max, y_max, z_max) - a modell saját
        # point_cloud_range-ével kell megegyezzen (lásd config/params.yaml
        # megjegyzése és preprocessing.py). Az alapérték a jelenlegi
        # centerpoint_pillar02_nuscenes checkpoint tanítási tartománya - ha
        # másik checkpointra váltotok, írjátok át a params.yaml-ban. Üres
        # lista = nincs vágás (NEM ajánlott, lásd a fenti megjegyzést a
        # mért lassulásról).
        self.declare_parameter(
            "point_cloud_range", [-51.2, -51.2, -5.0, 51.2, 51.2, 3.0]
        )
        # A valódi LiDAR intensity-skálája eltérhet attól, amire a checkpoint
        # tanítva van (lásd config/params.yaml megjegyzése) - 1.0 = nincs
        # skálázás.
        self.declare_parameter("intensity_scale", 1.0)

        lidar_topic = self.get_parameter("lidar_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._base_frame = self.get_parameter("base_frame").value
        self._score_threshold = self.get_parameter("score_threshold").value
        self._intensity_scale = float(self.get_parameter("intensity_scale").value)
        point_cloud_range = self.get_parameter("point_cloud_range").value
        point_cloud_range = tuple(point_cloud_range) if point_cloud_range else None

        model_config = self.get_parameter("model_config").value
        checkpoint = self.get_parameter("checkpoint").value
        if not model_config or not checkpoint:
            raise RuntimeError(
                "A 'model_config' és 'checkpoint' paramétereket kötelező "
                "beállítani (lásd config/params.yaml)."
            )

        self.get_logger().info(f"Modell betöltése: {checkpoint}")
        self._model = CenterPointWrapper(
            model_config, checkpoint, device=self.get_parameter("device").value
        )
        self.get_logger().info(
            f"Modell betöltve. Osztályok: {self._model.classes}"
        )

        self._sweep_buffer = SweepBuffer(
            max_sweeps=self.get_parameter("num_sweeps").value,
            point_cloud_range=point_cloud_range,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._publisher = self.create_publisher(DetectedObjects, self._output_topic, 10)
        # KEEP_LAST, depth=1: ha az inferencia lassabb, mint a LiDAR-sweepek
        # érkezési üteme, NE gyűljön fel a feldolgozatlan üzenetek sora (ami
        # folyamatosan növekvő késést/eltolódást okozna a pontfelhő és a
        # kirajzolt detekciók között - "nincs szinkronban" panasz) - inkább
        # mindig a legfrissebb sweepen dolgozzon, a régebbit dobja el.
        lidar_qos = QoSProfile(depth=1)
        self.create_subscription(PointCloud2, lidar_topic, self._on_pointcloud, lidar_qos)

        self.get_logger().info(
            f"Feliratkozva: {lidar_topic} (LiDAR, odom NINCS használva). "
            f"Kimenet: {self._output_topic}. "
            f"point_cloud_range={point_cloud_range}, intensity_scale={self._intensity_scale}."
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        # EZ a stamp megy ki a végén a kimeneti header.stamp-be - a
        # pontfelhő MÉRÉSÉNEK ideje, nem az inferencia befejezésének
        # ideje (spec.md 3. szekció, header.stamp sor).
        stamp = msg.header.stamp
        t_start = time.monotonic()

        try:
            points_sensor = self._extract_points(msg)
            points_base_link = self._transform_to_base_link(points_sensor, msg.header)
        except Exception as exc:  # TF hiányzik, rossz frame_id, stb.
            self.get_logger().error(f"Előfeldolgozási hiba: {exc}")
            return

        self._sweep_buffer.push(
            TimedSweep(
                points_base_link=points_base_link,
                stamp_sec=stamp.sec + stamp.nanosec * 1e-9,
            )
        )

        model_input = self._sweep_buffer.build_model_input()
        pred_instances_3d = self._model.infer(model_input)
        raw_count = int(pred_instances_3d.scores_3d.shape[0])

        header = msg.header
        header.frame_id = self._base_frame
        detections = build_detected_objects(
            pred_instances_3d,
            classes=self._model.classes,
            score_threshold=self._score_threshold,
            header=header,
        )
        self._publisher.publish(detections)

        elapsed = time.monotonic() - t_start
        # Ha ez tartósan hosszabb, mint a LiDAR-sweepek közötti idő, a
        # detekciók folyamatosan egyre jobban le fognak maradni a pontfelhő
        # mögött ("nincs szinkronban" panasz) - ez a szám azt mutatja meg,
        # mennyire vagyunk ettől távol.
        self.get_logger().info(
            f"{model_input.shape[0]} pont a modellbemenetben, feldolgozás {elapsed:.3f}s, "
            f"{raw_count} nyers detekció -> {len(detections.objects)} publikálva "
            f"(score_threshold={self._score_threshold}, csak 'car' osztály kerül ki).",
            throttle_duration_sec=2.0,
        )

    def _extract_points(self, msg: PointCloud2) -> np.ndarray:
        structured = point_cloud2.read_points(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )
        intensity = structured["intensity"].astype(np.float32)
        if self._intensity_scale != 1.0:
            # Lásd config/params.yaml `intensity_scale` megjegyzése - a
            # checkpoint nuScenes 0-255-ös intensity-skálájához van
            # kalibrálva, a valódi LiDAR ettől eltérő tartományban publikál.
            intensity = np.clip(intensity * self._intensity_scale, 0.0, 255.0)
        points = np.stack(
            [
                structured["x"].astype(np.float32),
                structured["y"].astype(np.float32),
                structured["z"].astype(np.float32),
                intensity,
            ],
            axis=-1,
        )
        return points

    def _transform_to_base_link(self, points: np.ndarray, header) -> np.ndarray:
        if header.frame_id == self._base_frame:
            return points

        transform: TransformStamped = self._tf_buffer.lookup_transform(
            self._base_frame, header.frame_id, header.stamp
        )
        # EGYSZERŰSÍTVE: csak transzláció + yaw körüli forgatás. Ha a
        # szenzor dőlve van a base_link-hez képest (roll/pitch), ide
        # teljes 3D forgatás kell (pl. tf_transformations vagy
        # scipy.spatial.transform.Rotation a kvaternionból).
        #
        # FONTOS: ez a TF a szenzor<->base_link STATIKUS extrinsicje (pl.
        # /tf_static-ból), NEM az ego mozgása - ehhez nem kell odom, ezt a
        # LiDAR odom-mentes üzemmódban is meg lehet oldani.
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = 2.0 * np.arctan2(q.z, q.w)
        cos_y, sin_y = np.cos(yaw), np.sin(yaw)
        out = points.copy()
        out[:, 0] = points[:, 0] * cos_y - points[:, 1] * sin_y + t.x
        out[:, 1] = points[:, 0] * sin_y + points[:, 1] * cos_y + t.y
        out[:, 2] = points[:, 2] + t.z
        return out


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
