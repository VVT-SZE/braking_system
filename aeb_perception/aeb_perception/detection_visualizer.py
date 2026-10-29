"""Publish perception detections as boxes and labels for Foxglove."""

from copy import deepcopy

import rclpy
from autoware_perception_msgs.msg import DetectedObjects
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class DetectionVisualizer(Node):
    def __init__(self):
        super().__init__("aeb_detection_visualizer")

        self.declare_parameter("input_topic", "/perception/detections")
        self.declare_parameter("output_topic", "/perception/markers")

        self.publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter("output_topic").value,
            10,
        )
        self.subscription = self.create_subscription(
            DetectedObjects,
            self.get_parameter("input_topic").value,
            self.on_detections,
            10,
        )

        self.get_logger().info(
            "Megjelenítés: /perception/detections -> /perception/markers"
        )

    def on_detections(self, msg):
        markers = MarkerArray()

        # Remove previous boxes, including when the new detection list is empty.
        clear = Marker()
        clear.header = deepcopy(msg.header)
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, obj in enumerate(msg.objects):
            box = Marker()
            box.header = deepcopy(msg.header)
            box.ns = "detection_boxes"
            box.id = index
            box.type = Marker.CUBE
            box.action = Marker.ADD

            # Perception already publishes the geometric center of the box.
            box.pose = deepcopy(obj.kinematics.pose_with_covariance.pose)
            box.scale.x = float(obj.shape.dimensions.x)
            box.scale.y = float(obj.shape.dimensions.y)
            box.scale.z = float(obj.shape.dimensions.z)

            box.color.r = 0.1
            box.color.g = 1.0
            box.color.b = 0.2
            box.color.a = 0.35

            markers.markers.append(box)

            label = Marker()
            label.header = deepcopy(msg.header)
            label.ns = "detection_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = deepcopy(box.pose.position)
            label.pose.position.z += box.scale.z / 2.0 + 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.5

            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0

            # Current perception postprocessing forwards only the car class.
            label.text = (
                f"car score={obj.existence_probability:.2f}"
            )
            markers.markers.append(label)

        self.publisher.publish(markers)

        self.get_logger().info(
            f"{len(msg.objects)} detekció megjelenítése",
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
