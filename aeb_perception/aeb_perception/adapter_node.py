"""Adapter: autoware_perception_msgs/DetectedObjects + Odometry
-> crp_msgs::msg::Scenario (local_moving_objects) + crp_msgs::msg::Ego.

A mezőleképezés és a topic-nevek a braking_system repó forrásából vannak
leellenőrizve (behavior_planner/src/behaviorPlanner.cpp), NEM csak a
spec.md táblázatából kitalálva:

- a behavior_planner "scenario" és "ego" topicra iratkozik fel
  (behaviorPlanner.cpp: declare_parameter("input_topic_scenario", "scenario"),
  declare_parameter("input_topic_ego", "ego")) - nincs namespace-elés,
  nincs top-level bringup launch, ezért ezek a tényleges, teljes topicnevek.
- crp_msgs::msg::Scenario.local_moving_objects.objects pontosan
  autoware_perception_msgs::msg::PredictedObject lista
  (scenarioCallback: `for (const auto &obj : msg->local_moving_objects.objects)`)
- crp_msgs::msg::Ego.pose.pose / .twist.twist / .accel.accel
  (egoCallback: `m_ego_pose_ = msg->pose.pose;` stb.)
- PredictedObject.kinematics.initial_pose_with_covariance /
  initial_twist_with_covariance / initial_acceleration_with_covariance
  (run(): `closest_obj->kinematics.initial_pose_with_covariance...`)

MARADÉK BLOKKOLÓ: a crp_msgs csomag FORRÁSA nincs a braking_system
repóban - csak `find_package(crp_msgs REQUIRED)` és
`#include <crp_msgs/msg/scenario.hpp>` van benne, nincs .gitmodules,
nincs .repos fájl, ami megmondaná, honnan jön. Valahol a ti ROS2
workspace-etekben (vagy a tanári infrastruktúra egy külön csomagjaként)
már ott kell lennie, különben a braking_system maga sem fordulna le.
Keresd meg: `ros2 pkg prefix crp_msgs` vagy `colcon list | grep crp_msgs`
a saját workspace-eden - amint megvan, importáld ide is.
"""
from typing import Optional

import rclpy
from autoware_perception_msgs.msg import DetectedObjects, PredictedObject
from nav_msgs.msg import Odometry
from rclpy.node import Node

from crp_msgs.msg import Ego, Scenario  # noqa: E402  (lásd a fenti blokkoló megjegyzést)


class AdapterNode(Node):
    def __init__(self):
        super().__init__("aeb_adapter_node")

        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("odom_topic", "/novatel/oem7/odom")
        # A behavior_planner pontos default bemeneti topicjai (lásd a
        # modul docstringjét - forrásból ellenőrizve, nem TBD már).
        self.declare_parameter("scenario_topic", "scenario")
        self.declare_parameter("ego_topic", "ego")

        self._scenario_pub = self.create_publisher(
            Scenario, self.get_parameter("scenario_topic").value, 1
        )
        self._ego_pub = self.create_publisher(
            Ego, self.get_parameter("ego_topic").value, 1
        )

        self.create_subscription(
            DetectedObjects,
            self.get_parameter("detections_topic").value,
            self._on_detections,
            10,
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10
        )
        self._latest_ego_odom: Optional[Odometry] = None

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_ego_odom = msg

        ego = Ego()
        ego.pose.pose = msg.pose.pose
        ego.twist.twist = msg.twist.twist
        # Gyorsulás: ha van mérés, ide; ha nincs, 0 (spec.md 3. szekció "ax" sora).
        # A kormányszöget (tire_angle_front) a spec.md 4. szekciója említi, de a
        # crp_msgs::Ego pontos mezőnevét nem tudom ellenőrizni innen - ha van
        # ilyen mérésed, `ros2 interface show crp_msgs/msg/Ego`-val nézd meg a
        # tényleges mezőnevet, mielőtt kitöltöd.
        self._ego_pub.publish(ego)

    def _on_detections(self, msg: DetectedObjects) -> None:
        if self._latest_ego_odom is None:
            return

        predicted_objects = []
        for obj in msg.objects:
            predicted = PredictedObject()
            predicted.classification = obj.classification

            kin = predicted.kinematics
            kin.initial_pose_with_covariance = obj.kinematics.pose_with_covariance
            kin.initial_twist_with_covariance = obj.kinematics.twist_with_covariance
            # ax: ha nincs mérve, 0 (spec.md 3. szekció).
            kin.initial_acceleration_with_covariance.accel.linear.x = 0.0

            predicted_objects.append(predicted)

        scenario = Scenario()
        scenario.header.stamp = msg.header.stamp
        scenario.header.frame_id = "base_link"
        scenario.local_moving_objects.objects = predicted_objects
        # scenario.paths: a sáv-/útvonalgeometriát (left_bound/right_bound,
        # lásd behaviorPlanner.cpp scenarioCallback) EZ a node nem tölti ki -
        # az egy más komponens (route/lane planner) felelőssége, nem a
        # perceptioné. Ha a behavior_planner lane_width-számítása ettől
        # hibásan 4.0 m-es alapértékre esik vissza, az NEM ennek a node-nak
        # a hibája.
        self._scenario_pub.publish(scenario)

        self.get_logger().info(f"{len(predicted_objects)} objektum publikálva a Scenario-ban.")


def main(args=None):
    rclpy.init(args=args)
    node = AdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
