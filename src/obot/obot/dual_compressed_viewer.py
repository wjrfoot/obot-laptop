#!/usr/bin/env python3

import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage


class DualCompressedViewer(Node):

    def __init__(self):
        super().__init__('dual_compressed_viewer')

        # Camera topics on OBot.
        self.declare_parameter(
            'topic_left',
            '/front_camera/image_raw/compressed'
        )
        self.declare_parameter(
            'topic_right',
            '/rear_camera/image_raw/compressed'
        )

        self.declare_parameter('window_left', 'Front Camera')
        self.declare_parameter('window_right', 'Rear Camera')
        self.declare_parameter('show_side_by_side', True)
        self.declare_parameter('combined_window', 'OBot Cameras')
        self.declare_parameter('max_display_hz', 30.0)
        self.declare_parameter('waitkey_ms', 1)

        self.topic_left = self.get_parameter('topic_left').value
        self.topic_right = self.get_parameter('topic_right').value
        self.window_left = self.get_parameter('window_left').value
        self.window_right = self.get_parameter('window_right').value

        self.show_side_by_side = bool(
            self.get_parameter('show_side_by_side').value
        )

        self.combined_window = self.get_parameter(
            'combined_window'
        ).value

        self.max_display_hz = float(
            self.get_parameter('max_display_hz').value
        )

        self.waitkey_ms = int(
            self.get_parameter('waitkey_ms').value
        )

        if self.waitkey_ms < 1:
            self.waitkey_ms = 1

        # Latest decoded camera images.
        self.last_left = None
        self.last_right = None

        self.decode_fail_left = 0
        self.decode_fail_right = 0

        self.left_msg_count = 0
        self.right_msg_count = 0

        self.last_draw_time = 0.0

        # Protect camera images because ROS callbacks execute in another
        # thread while the OpenCV GUI remains in the main thread.
        self._image_lock = threading.Lock()

        # The OpenCV window is deliberately created here.
        #
        # Creating it before the first imshow() allows us to detect when
        # the user closes it. Otherwise imshow() simply recreates a
        # closed window on the next frame.
        cv2.namedWindow(
            self.combined_window,
            cv2.WINDOW_NORMAL
        )

        # resizeWindow() will be called once after the first real
        # composite image is available. This restores the natural
        # initial image size while still allowing the window to be
        # resized or maximized afterward.
        self._window_sized = False

        self.sub_left = self.create_subscription(
            CompressedImage,
            self.topic_left,
            self.on_left,
            qos_profile_sensor_data
        )

        self.sub_right = self.create_subscription(
            CompressedImage,
            self.topic_right,
            self.on_right,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Subscribing front: {self.topic_left}'
        )
        self.get_logger().info(
            f'Subscribing rear : {self.topic_right}'
        )
        self.get_logger().info(
            "Press 'q' or close the window to quit."
        )

    @staticmethod
    def _decode(msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)

    def on_left(self, msg: CompressedImage):
        img = self._decode(msg)

        if img is None:
            self.decode_fail_left += 1
            self.get_logger().warn(
                'Failed to decode FRONT image'
            )
            return

        with self._image_lock:
            self.last_left = img

        self.left_msg_count += 1

    def on_right(self, msg: CompressedImage):
        img = self._decode(msg)

        if img is None:
            self.decode_fail_right += 1
            self.get_logger().warn(
                'Failed to decode REAR image'
            )
            return

        with self._image_lock:
            self.last_right = img

        self.right_msg_count += 1

    def draw(self):
        now = time.time()

        if self.max_display_hz > 0.0:
            period = 1.0 / self.max_display_hz

            if (now - self.last_draw_time) < period:
                return

        self.last_draw_time = now

        with self._image_lock:
            left = (
                None
                if self.last_left is None
                else self.last_left.copy()
            )

            right = (
                None
                if self.last_right is None
                else self.last_right.copy()
            )

        # Wait for at least one camera before displaying anything.
        if left is None and right is None:
            return

        if left is None:
            left = np.zeros(
                (240, 320, 3),
                dtype=np.uint8
            )

            cv2.putText(
                left,
                'NO FRONT IMAGE',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        if right is None:
            right = np.zeros(
                (240, 320, 3),
                dtype=np.uint8
            )

            cv2.putText(
                right,
                'NO REAR IMAGE',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        if not self.show_side_by_side:

            cv2.imshow(
                self.window_left,
                left
            )

            cv2.imshow(
                self.window_right,
                right
            )

            key = cv2.waitKey(
                self.waitkey_ms
            ) & 0xFF

            if key == ord('q'):
                rclpy.shutdown()

            return

        # Make both images the same height before combining them.
        h = min(
            left.shape[0],
            right.shape[0]
        )

        if left.shape[0] != h:
            scale = h / left.shape[0]

            left = cv2.resize(
                left,
                (
                    int(left.shape[1] * scale),
                    h
                )
            )

        if right.shape[0] != h:
            scale = h / right.shape[0]

            right = cv2.resize(
                right,
                (
                    int(right.shape[1] * scale),
                    h
                )
            )

        combo = np.hstack([
            left,
            right
        ])

        # IMPORTANT:
        # Check whether the window was closed BEFORE imshow().
        # Calling imshow() first would recreate a closed window.
        if cv2.getWindowProperty(
            self.combined_window,
            cv2.WND_PROP_VISIBLE
        ) < 1:
            rclpy.shutdown()
            return

        # The window was created during __init__, before we knew the
        # dimensions of the incoming images. Size it exactly once when
        # the first composite image is ready.
        if not self._window_sized:

            cv2.resizeWindow(
                self.combined_window,
                combo.shape[1],
                combo.shape[0]
            )

            self._window_sized = True

        cv2.imshow(
            self.combined_window,
            combo
        )

        key = cv2.waitKey(
            self.waitkey_ms
        ) & 0xFF

        if key == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = DualCompressedViewer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # ROS callbacks run in the background. OpenCV GUI operations remain
    # in the main thread, which is required for reliable GTK/OpenCV
    # window behavior.
    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )

    spin_thread.start()

    try:

        period = 0.0

        if node.max_display_hz > 0.0:
            period = 1.0 / node.max_display_hz

        while rclpy.ok():

            node.draw()

            if period > 0.0:
                time.sleep(period)

    except KeyboardInterrupt:
        pass

    finally:

        executor.remove_node(node)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
