import quaternion

import rclpy
from rclpy.node import Node

from rclpy.duration import Duration
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Empty, Header
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    VisionInfo,
)
from visualization_msgs.msg import Marker


class DummyDetectionPublisher(Node):
    def __init__(self):
        super().__init__("dummy_detection_publisher")

        # Class of the object to track
        self.declare_parameter("class_id", "")
        # Distance from camera sensor
        self.declare_parameter("distance_from_cam", 0.3)
        # Initial roll, pitch, yaw rotation of the object
        self.declare_parameter("object_rpy", [0.0, 0.0, 0.0])
        # Frame ID of the camera's color sensor
        self.declare_parameter("frame_id", "")
        # Global path to a mesh file to send to RViz
        self.declare_parameter("mesh_file", "")
        # Global path to a mesh file to send to RViz
        self.declare_parameter("mesh_scale", 0.001)
        # Modified color of the mesh. By default mesh will be red-ish
        self.declare_parameter("mesh_color", [1.0, 0.3, 0.3])

        class_id = self.get_parameter("class_id").get_parameter_value().string_value
        distance_from_cam = (
            self.get_parameter("distance_from_cam").get_parameter_value().double_value
        )
        object_rpy = (
            self.get_parameter("object_rpy").get_parameter_value().double_array_value
        )
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        mesh_scale = self.get_parameter("mesh_scale").get_parameter_value().double_value
        mesh_file = self.get_parameter("mesh_file").get_parameter_value().string_value
        mesh_color = (
            self.get_parameter("mesh_color").get_parameter_value().double_array_value
        )

        object_quat = quaternion.from_euler_angles(object_rpy)

        self._detections_pub = self.create_publisher(
            Detection2DArray, "reference/detections", 10
        )
        self._detection_msg = Detection2DArray(
            header=Header(frame_id=frame_id),
            detections=[
                Detection2D(
                    header=Header(frame_id=frame_id),
                    id="0",
                    results=[
                        ObjectHypothesisWithPose(
                            hypothesis=ObjectHypothesis(
                                class_id=class_id,
                                score=1.0,
                            ),
                            pose=PoseWithCovariance(
                                pose=Pose(
                                    position=Point(z=distance_from_cam),
                                    orientation=Quaternion(
                                        x=object_quat.x,
                                        y=object_quat.y,
                                        z=object_quat.z,
                                        w=object_quat.w,
                                    ),
                                )
                            ),
                        )
                    ],
                )
            ],
        )

        self._vision_info_pub = self.create_publisher(
            VisionInfo, "reference/vision_info", 10
        )
        self._vision_info_msg = VisionInfo(
            header=Header(frame_id=frame_id),
            method="hand-crafted-detection",
            database_location="mesh_file",
            database_version=-1,
        )

        # Publish markers at 10 Hz
        marker_pub_time = 1.0 / 10.0

        self._init_marker_pub = self.create_publisher(
            Marker, "tracker_seeder/marker", 10
        )
        self._marker_msg = Marker(
            id=0,
            mesh_resource="file://" + mesh_file,
            mesh_use_embedded_materials=True,
            type=Marker.MESH_RESOURCE,
            header=Header(frame_id=frame_id),
            scale=Vector3(**dict(zip("xyz", [mesh_scale] * 3))),
            color=ColorRGBA(
                **dict(zip("rgb", mesh_color)),
                a=1.0,
            ),
            lifetime=Duration(seconds=marker_pub_time * 2.0).to_msg(),
            pose=self._detection_msg.detections[0].results[0].pose.pose,
        )

        self._empty_sub = self.create_subscription(
            Empty, "/tracker/key_pressed", self._key_press_cb, 10
        )

        self._marker_pub_timer = self.create_timer(marker_pub_time, self._marker_pub_cb)
        self.get_logger().info("Node initialized, waiting for keyboard node.")

    def _marker_pub_cb(self):
        self._marker_msg.header.stamp = self.get_clock().now().to_msg()
        self._init_marker_pub.publish(self._marker_msg)

    def _key_press_cb(self, _: Empty) -> None:
        self.get_logger().info("Keyboard even received. Sending seed to tracker.")
        self._marker_pub_timer.destroy()
        now = self.get_clock().now().to_msg()
        self._detection_msg.header.stamp = now
        self._vision_info_msg.header.stamp = now
        self._detections_pub.publish(self._detection_msg)
        self._vision_info_pub.publish(self._vision_info_msg)


def main(args=None):
    rclpy.init(args=args)
    dummy_detection_publisher = DummyDetectionPublisher()
    rclpy.spin(dummy_detection_publisher)
    dummy_detection_publisher.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
