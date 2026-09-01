#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class TripleCompressedViewer(Node):
    def __init__(self):
        super().__init__('triple_compressed_viewer')

        self.window_name = 'Triple Compressed Viewer'
        self.large_size = (640, 480)
        self.small_size = (320, 240)
        self.control_bar_height = 60

        self.declare_parameter('topic1', '/cam0/image_raw/compressed')
        self.declare_parameter('topic2', '/cam1/image_raw/compressed')
        self.declare_parameter('topic3', '/cam2/image_raw/compressed')

        self.topic1 = self.get_parameter('topic1').value
        self.topic2 = self.get_parameter('topic2').value
        self.topic3 = self.get_parameter('topic3').value

        self.img1 = None
        self.img2 = None
        self.img3 = None
        self.active_stream = 0

        self.stream_labels = ['Camera 1', 'Camera 2', 'Camera 3']
        self.button_rects = []

        self.create_subscription(CompressedImage, self.topic1, self.cb1, 10)
        self.create_subscription(CompressedImage, self.topic2, self.cb2, 10)
        self.create_subscription(CompressedImage, self.topic3, self.cb3, 10)

        cv2.namedWindow(
            self.window_name,
            cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_EXPANDED,
        )
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        self.timer = self.create_timer(0.05, self.draw)

        self.get_logger().info(f'Stream 1: {self.topic1}')
        self.get_logger().info(f'Stream 2: {self.topic2}')
        self.get_logger().info(f'Stream 3: {self.topic3}')

    def decode(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def cb1(self, msg):
        self.img1 = self.decode(msg)

    def cb2(self, msg):
        self.img2 = self.decode(msg)

    def cb3(self, msg):
        self.img3 = self.decode(msg)

    def on_mouse(self, event, x, y, flags, userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        for index, rect in enumerate(self.button_rects):
            left, top, right, bottom = rect
            if left <= x <= right and top <= y <= bottom:
                self.active_stream = index
                self.get_logger().info(
                    f'Selected large stream: {self.stream_labels[index]}'
                )
                break

    def make_panel(self, img, label, size):
        w, h = size

        if img is None:
            panel = np.zeros((h, w, 3), dtype=np.uint8)
            cv2.putText(panel, 'NO IMAGE', (80, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        else:
            panel = cv2.resize(img, (w, h))

        cv2.putText(panel, label, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return panel

    def make_control_bar(self):
        bar = np.full((self.control_bar_height, 640, 3), 28, dtype=np.uint8)
        button_width = 180
        button_height = 36
        gap = 20
        total_width = button_width * 3 + gap * 2
        start_x = (640 - total_width) // 2
        top = (self.control_bar_height - button_height) // 2

        # y-offset of the control bar within the full composed frame
        y_offset = self.large_size[1] + self.small_size[1]

        self.button_rects = []

        for index, label in enumerate(self.stream_labels):
            left = start_x + index * (button_width + gap)
            right = left + button_width
            bottom = top + button_height
            # store absolute coords so on_mouse can compare directly
            self.button_rects.append(
                (left, y_offset + top, right, y_offset + bottom)
            )

            active = index == self.active_stream
            fill_color = (70, 120, 70) if active else (55, 55, 55)
            border_color = (140, 220, 140) if active else (180, 180, 180)

            cv2.rectangle(bar, (left, top), (right, bottom), fill_color, -1)
            cv2.rectangle(bar, (left, top), (right, bottom), border_color, 2)

            circle_center = (left + 22, top + button_height // 2)
            cv2.circle(bar, circle_center, 9, border_color, 2)
            if active:
                cv2.circle(bar, circle_center, 5, border_color, -1)

            cv2.putText(
                bar,
                label,
                (left + 40, top + 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

        return bar

    def get_stream_image(self, stream_index):
        if stream_index == 0:
            return self.img1
        if stream_index == 1:
            return self.img2
        return self.img3

    def draw(self):
        selected_image = self.get_stream_image(self.active_stream)
        selected_label = self.stream_labels[self.active_stream]
        large_panel = self.make_panel(
            selected_image,
            selected_label,
            self.large_size,
        )

        other_streams = [index for index in range(3)
                         if index != self.active_stream]
        small_left = self.make_panel(
            self.get_stream_image(other_streams[0]),
            self.stream_labels[other_streams[0]],
            self.small_size,
        )
        small_right = self.make_panel(
            self.get_stream_image(other_streams[1]),
            self.stream_labels[other_streams[1]],
            self.small_size,
        )
        control_bar = self.make_control_bar()

        top_row = large_panel
        bottom_row = np.hstack((small_left, small_right))
        combined = np.vstack((top_row, bottom_row, control_bar))

        cv2.rectangle(combined, (0, 0), (639, 479), (90, 180, 90), 2)

        cv2.imshow(self.window_name, combined)
        cv2.waitKey(1)

        if cv2.getWindowProperty(
            self.window_name, cv2.WND_PROP_VISIBLE
        ) < 1:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = TripleCompressedViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
