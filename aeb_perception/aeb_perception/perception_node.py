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
import numpy as np
import rclpy
from autoware_perception_msgs.msg import DetectedObjects
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
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
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("model_config", "")
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("score_threshold", 0.3)
        self.declare_parameter("num_sweeps", 10)

        lidar_topic = self.get_parameter("lidar_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._base_frame = self.get_parameter("base_frame").value
        self._score_threshold = self.get_parameter("score_threshold").value

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
            max_sweeps=self.get_parameter("num_sweeps").value
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._publisher = self.create_publisher(DetectedObjects, self._output_topic, 10)
        self.create_subscription(PointCloud2, lidar_topic, self._on_pointcloud, 10)

        self.get_logger().info(
            f"Feliratkozva: {lidar_topic} (LiDAR, odom NINCS használva). "
            f"Kimenet: {self._output_topic}."
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        # EZ a stamp megy ki a végén a kimeneti header.stamp-be - a
        # pontfelhő MÉRÉSÉNEK ideje, nem az inferencia befejezésének
        # ideje (spec.md 3. szekció, header.stamp sor).
        stamp = msg.header.stamp

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

        header = msg.header
        header.frame_id = self._base_frame
        detections = build_detected_objects(
            pred_instances_3d,
            classes=self._model.classes,
            score_threshold=self._score_threshold,
            header=header,
        )
        self._publisher.publish(detections)

    @staticmethod
    def _extract_points(msg: PointCloud2) -> np.ndarray:
        structured = point_cloud2.read_points(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )
        points = np.stack(
            [
                structured["x"].astype(np.float32),
                structured["y"].astype(np.float32),
                structured["z"].astype(np.float32),
                structured["intensity"].astype(np.float32),
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
