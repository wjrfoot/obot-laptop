from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    common_parameters = {
        "image_width": 320,
        "image_height": 240,
        "framerate": 30.0,
        "pixel_format": "mjpeg2rgb",
    }

    cameras = [
        {
            "namespace": "camera_1",
            "camera_name": "camera_1",
            "device": "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.1:1.0-video-index0",
        },
        {
            "namespace": "camera_2",
            "camera_name": "camera_2",
            "device": "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.2:1.0-video-index0",
        },
        {
            "namespace": "camera_3",
            "camera_name": "camera_3",
            "device": "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.3:1.0-video-index0",
        },
    ]

    nodes = []

    for camera in cameras:
        params = dict(common_parameters)
        params["video_device"] = camera["device"]
        params["camera_name"] = camera["camera_name"]

        nodes.append(
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam",
                namespace=camera["namespace"],
                output="screen",
                parameters=[params],
            )
        )

    return LaunchDescription(nodes)
