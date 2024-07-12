import sys
import termios
import tty

import rclpy

from std_msgs.msg import Empty


def main(args=None):
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)

    rclpy.init(args=args)
    node = rclpy.create_node("keyboard_monitor")
    node.get_logger().warn("Press 'q' to quit or 'space bar' to start tracking.")

    try:
        publisher = node.create_publisher(Empty, "/tracker/key_pressed", 10)

        key = None

        # Press 'q' or 'space bar'
        while key not in ["q", " "]:
            key = sys.stdin.read(1)[0]

        if key == " ":
            node.get_logger().info("Tracker was initialized.")
            publisher.publish(Empty())
        else:
            node.get_logger().info("Nothing was published. Au revoir.")

    except KeyboardInterrupt:
        pass

    except Exception as err:
        print(str(err))

    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)


if __name__ == "__main__":
    main()
