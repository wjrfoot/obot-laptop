"""Convert calibrated fisheye image coordinates into robot-relative bearings."""

import json
import math

import cv2
import numpy as np


class FisheyeBearing:
    def __init__(self, calibration_path, yaw_deg=0.0,
                 pitch_deg=0.0, roll_deg=0.0):
        with open(calibration_path, encoding='utf-8') as file:
            calibration = json.load(file)

        if calibration['distortion_model'] != 'equidistant':
            raise ValueError('Expected equidistant fisheye calibration')

        self.width = int(calibration['width'])
        self.height = int(calibration['height'])
        self.K = np.asarray(calibration['K'], dtype=np.float64)
        self.D = np.asarray(calibration['D'], dtype=np.float64).reshape(4, 1)

        self.yaw = math.radians(yaw_deg)
        self.pitch = math.radians(pitch_deg)
        self.roll = math.radians(roll_deg)

    def bearing_deg(self, x, y):
        """Return horizontal bearing in degrees; positive means right."""
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise ValueError('Pixel outside calibrated image')

        pixel = np.array([[[x, y]]], dtype=np.float64)
        undistorted = cv2.fisheye.undistortPoints(
            pixel, self.K, self.D
        )[0, 0]

        # Camera coordinates: X right, Y down, Z forward.
        ray = np.array(
            [undistorted[0], undistorted[1], 1.0],
            dtype=np.float64
        )
        ray /= np.linalg.norm(ray)

        # Robot coordinates: X forward, Y left, Z up.
        forward, left, up = ray[2], -ray[0], -ray[1]

        # Extrinsic rotations; zero values are our initial assumption.
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        left, up = cr * left - sr * up, sr * left + cr * up

        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        forward, up = cp * forward + sp * up, -sp * forward + cp * up

        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        forward, left = (
            cy * forward - sy * left,
            sy * forward + cy * left
        )

        return math.degrees(math.atan2(-left, forward))
