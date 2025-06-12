import os
import sys
import time
import signal
import subprocess
import logging
import argparse
import select

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SerialNumberListener(Node):
    def __init__(self):
        super().__init__('serial_number_listener')
        self.subscription = self.create_subscription(String, '/camera_front_center/serial_number', self.listener_callback, 10)
        self.serial_number = None

    def listener_callback(self, msg):
        self.serial_number = msg.data


def get_camera_serial_number(timeout=5) -> str:
    rclpy.init()
    node = SerialNumberListener()
    serial_number = None
    start_time = time.time()

    try:
        while rclpy.ok() and (time.time() - start_time < timeout):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.serial_number:
                serial_number = node.serial_number
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not serial_number:
        logging.error("Could not get camera serial number from /camera_front_center/serial_number.")
        sys.exit(1)

    return serial_number


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def user_input(prompt: str):
    print(prompt)
    print("Press Enter to continue or Ctrl+C to cancel...")
    try:
        if sys.stdin in select.select([sys.stdin], [], [], None)[0]:
            input()
    except KeyboardInterrupt:
        print("\nRecording aborted.")
        sys.exit(0)


def record_rosbag(bag_path: str, topics: list[str]):
    logging.info(f"Recording rosbag to: {bag_path}")
    cmd = ["ros2", "bag", "record", "-o", bag_path] + topics

    try:
        proc = subprocess.Popen(cmd)
        user_input("Recording started...")
        proc.send_signal(signal.SIGINT)
        proc.wait()
        logging.info(f"Rosbag saved at: {bag_path}")
    except Exception as e:
        logging.error(f"Failed to record rosbag: {e}")
        proc.kill()
        sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Record ROS 2 bags for specified steps.")
    parser.add_argument("steps", nargs="*", choices=["step_a", "step_b"],
                        help="Steps to record (default: step_a and step_b)")
    return parser.parse_args()


def main():
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    args = parse_args()

    steps = args.steps or ["step_a", "step_b"]

    topics = [
        "/camera_front_center/imu",
        "/camera_front_center/left/image_mono",
        "/camera_front_center/right/image_mono",
        "/camera_front_center/left/image_mono_raw",
        "/camera_front_center/right/image_mono_raw"
    ]

    serial_number = get_camera_serial_number()
    base_dir = f"/big_disk/{serial_number}"
    ensure_dir(base_dir)

    for step in steps:
        print("\nCamera topics to be recorded:")
        for t in topics:
            print(f"  {t}")
        user_input(f"\nReady to record bag for {step}?")
        bag_path = os.path.join(base_dir, step)
        record_rosbag(bag_path, topics)
        logging.info(f"Finished recording {step}.")


if __name__ == "__main__":
    main()
