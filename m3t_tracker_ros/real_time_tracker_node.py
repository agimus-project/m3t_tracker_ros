import rclpy

from m3t_tracker_ros.tracker_node_base import TrackerNodeBase


class RealTimeTrackerNode(TrackerNodeBase):
    pass


def main() -> None:
    rclpy.init()
    real_time_tracker_node = RealTimeTrackerNode()
    try:
        rclpy.spin(real_time_tracker_node)
    except KeyboardInterrupt:
        pass
    real_time_tracker_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
