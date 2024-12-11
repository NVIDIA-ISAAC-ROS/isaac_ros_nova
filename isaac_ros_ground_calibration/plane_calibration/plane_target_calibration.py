#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from abc import ABC, abstractmethod
import collections
from types import SimpleNamespace

import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as scipy_rotation
from sensor_msgs.msg import CameraInfo, Image
import yaml

DEFAULT_NUM_STILL_FRAMES = 10
DEFAULT_STILL_PIXEL_THRESHOLD = 5


def to_4x4_matrix(rotation_matrix, translation=np.array([0, 0, 0])):
    """Convert 3d rotation matrix and translation vector to homogeneous matrix in SE3."""
    homogeneous_transform = np.eye(4, dtype=rotation_matrix.dtype)
    homogeneous_transform[:3, :3] = rotation_matrix
    homogeneous_transform[:3, 3] = translation.reshape((-1, ))
    return homogeneous_transform


def rodriquez_to_4x4_matrix(rotation_rodrigez, translation=np.array([0, 0, 0])):
    """Convert a rodriguez vector in radians and translation to homogeneous matrix in SE3."""
    rotation_matrix = scipy_rotation.from_rotvec(rotation_rodrigez.reshape((-1, ))).as_matrix()
    return to_4x4_matrix(rotation_matrix, translation)


def quaternion_to_4x4_matrix(rotation_qauternion_xyzw, translation=np.array([0, 0, 0])):
    """Convert qyaterbuib (in xyzw format) and translation to homogeneous matrix in SE3."""
    rotation_matrix = scipy_rotation.from_quat(rotation_qauternion_xyzw).as_matrix()
    return to_4x4_matrix(rotation_matrix, translation)


class TargetDetector(ABC):
    """
    Target detector base class.

    Yaml input expected to have the following format:

    target_type: 'checkerboard'     # [checkerboard]
    num_squares_height: 9           # Number of squares in Y dimension
    num_squares_width: 12           # Number of squares in X dimension
    longer_side_m: 0.36             # Length in meters of the longest side of the checker pattern
    thickness_m: 0.006              # Thickness if the board in meters
    num_still_frames: 10            # Number of consecutive still frames to use for pose estimation
    still_image_pixel_threshold: 5  # Maximum pixel detection difference to consider still frame
    """

    def __init__(self, target_description, logger):
        self.logger = logger
        self.num_still_frames = target_description.get('num_still_frames',
                                                       DEFAULT_NUM_STILL_FRAMES)
        self.still_image_pixel_threshold = target_description.get('still_image_pixel_threshold',
                                                                  DEFAULT_STILL_PIXEL_THRESHOLD)
        self.thickness_m = target_description['thickness_m']
        self.camera_matrix = None
        self.optical_T_rectified = None
        self.ready = False

    @abstractmethod
    def detect(self, image_bgr):
        """Detect target in image and return both camera_optical_T_target and 2D corners."""
        pass

    @abstractmethod
    def plot_corners(self, image_bgr, corners_2d, camera_optical_T_target, text_origen):
        """Plot corners and axes of transformation, or the text 'Target not detected!'."""
        pass

    def set_intrinsics(self, camera_matrix, optical_T_rectified):
        self.camera_matrix = camera_matrix
        self.optical_T_rectified = optical_T_rectified
        self.ready = True

    def is_ready(self):
        return self.ready

    def get_target_thickness(self):
        return self.thickness_m

    def get_num_still_frames(self):
        return self.num_still_frames

    def get_still_image_pixel_threshold(self):
        return self.still_image_pixel_threshold


def create_target_detector(target_description_path, logger):
    with open(target_description_path, 'r', encoding='utf8') as fh:
        target_description = yaml.safe_load(fh)
        target_type = target_description['target_type']
        if target_type == 'checkerboard':
            return CheckerBoardDetector(target_description, logger)
        raise ValueError('Unexpected target_type. Supported types: checkerboard')


class CheckerBoardDetector(TargetDetector):

    def __init__(self, target_description, logger):
        super().__init__(target_description, logger)
        num_squares_height = target_description['num_squares_height']
        num_squares_width = target_description['num_squares_width']
        longer_side_m = target_description['longer_side_m']
        self.corners_3d = self._compute_checkerboard_3d_points(num_squares_height,
                                                               num_squares_width, longer_side_m)
        self.checkerboard_dimensions = (num_squares_width-1, num_squares_height-1)

        self.target_T_target_up = to_4x4_matrix(
            scipy_rotation.from_euler('x', 180, degrees=True).as_matrix())

    def _compute_checkerboard_3d_points(self, num_squares_height, num_squares_width,
                                        longer_side_m):
        """Compute points in the format [[0,0,0],[1,0,0],[2,0,0],...,[M,N,0]] * square_size."""
        num_inner_height = num_squares_height - 1
        num_inner_width = num_squares_width - 1
        num_squares_longer_side = max(num_squares_width, num_squares_height)
        checkerboard_square_size_m = longer_side_m / num_squares_longer_side
        corners_3d = np.zeros((num_inner_width * num_inner_height, 3), np.float32)
        corners_3d[:, :2] = np.mgrid[0:num_inner_width, 0:num_inner_height].T.reshape(-1, 2)
        return corners_3d * checkerboard_square_size_m

    def _detect_target(self, image_bgr, refine_subpixel=True):
        if len(image_bgr.shape) < 3:
            image_gray = image_bgr
        else:
            image_gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        ret, corners_2d = cv2.findChessboardCorners(image_gray, self.checkerboard_dimensions, None)
        if not ret:
            self.logger.warning('Target not detected')
            return None
        if refine_subpixel:
            detect_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            detect_search_window = (5, 5)
            disable_zero_zone = (-1, -1)
            corners_2d = cv2.cornerSubPix(image_gray, corners_2d, winSize=detect_search_window,
                                          zeroZone=disable_zero_zone, criteria=detect_criteria)

        return corners_2d

    def _estimate_pose(self, corners_2d, refine_pnp='VVS', pnp_flags=cv2.SOLVEPNP_ITERATIVE):
        ret, rvec, tvec = cv2.solvePnP(self.corners_3d, corners_2d, self.camera_matrix, None,
                                       flags=pnp_flags)
        if not ret:
            self.logger.warning('PnP failed')
            return None
        if refine_pnp:
            stop_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            if refine_pnp == 'LM':
                rvec, tvec = cv2.solvePnPRefineLM(self.corners_3d, corners_2d, self.camera_matrix,
                                                  None,  rvec, tvec, stop_criteria)
            elif refine_pnp == 'VVS':
                rvec, tvec = cv2.solvePnPRefineVVS(self.corners_3d, corners_2d, self.camera_matrix,
                                                   None, rvec, tvec, stop_criteria)
            else:
                raise ValueError('Unexpected PNP refine type. Supported: LM, VVS')
        camera_rect_T_target = rodriquez_to_4x4_matrix(rvec, tvec)
        camera_optical_T_target = self.optical_T_rectified @ camera_rect_T_target
        camera_optical_T_target_up = camera_optical_T_target @ self.target_T_target_up
        return camera_optical_T_target_up

    def detect(self, image_bgr):
        """Detect target in image and return both camera_optical_T_target and 2D corners."""
        if not self.is_ready():
            self.logger.warning('TargetDetector not ready. Skipping detect()')
            return None, None

        corners_2d = self._detect_target(image_bgr, refine_subpixel=True)
        if corners_2d is None:
            return None, None

        camera_optical_T_target = self._estimate_pose(corners_2d)
        return camera_optical_T_target, corners_2d

    def plot_corners(self, image_bgr, corners_2d, camera_optical_T_target, text_origen=30):
        """Plot corners and axes of transformation, or the text 'Target not detected!'."""
        if not self.is_ready():
            self.logger.warning('TargetDetector not ready. Skipping plot_corners()')
            return None, None

        if corners_2d is not None:
            cv2.drawChessboardCorners(image_bgr, self.checkerboard_dimensions, corners_2d, True)
            if camera_optical_T_target is not None:
                tvec = camera_optical_T_target[:3, 3]
                rvec = scipy_rotation.from_matrix(camera_optical_T_target[:3, :3]).as_rotvec()
                cv2.drawFrameAxes(image_bgr, self.camera_matrix, None, rvec, tvec, length=0.3,
                                  thickness=3)
        else:
            cv2.putText(image_bgr, 'Target not detected!',
                        org=(text_origen, image_bgr.shape[0] - text_origen),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, color=(0, 255, 0),
                        thickness=2)


class PlaneTargetCalibration(Node):

    def __init__(self):
        super().__init__('plane_target_calibration')

        self.declare_parameter('target_description_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('output_path', rclpy.Parameter.Type.STRING)
        self.declare_parameter('shutdown_on_detection', False)

        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.shutdown_on_detection = self.get_parameter(
            'shutdown_on_detection').get_parameter_value().bool_value
        target_description_path = self.get_parameter(
            'target_description_path').get_parameter_value().string_value
        self.target_detector = create_target_detector(target_description_path, self.get_logger())

        self.detections_queue = collections.deque(
            maxlen=self.target_detector.get_num_still_frames())

        self.detections_image_publisher = self.create_publisher(Image, 'target_detection', 10)
        self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, 'image_rect', self.rectified_image_callback, 10)

        self.cv_bridge = cv_bridge.CvBridge()

        self.optical_T_canonical = to_4x4_matrix(
            np.array([0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0]).reshape(3, 3))
        self.canonical_T_optical = to_4x4_matrix(
            np.array([0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0]).reshape(3, 3))
        self.ground_z_up = np.array([0.0, 0.0, 1.0])

        self.running = True

    def store(self, timestamp_ns, corners_2d, camera_T_target):
        """Save and detections, remove moving frames, and return True if ready to calibrate."""
        def similar_corners_2d(lhs, rhs):
            all_close = np.allclose(lhs, rhs,
                                    atol=self.target_detector.get_still_image_pixel_threshold())
            if not all_close:
                self.get_logger().info('Target moving or uncertain. Please keep target still. If '
                                       'it is not moving, place the target closer to the camera '
                                       'or print a larger target.')
            return all_close

        # Clear queue if detections moved since last frame or if messages are too old
        if self.detections_queue:
            # tail_timestamp_ns = self.detections_queue[-1].timestamp_ns
            tail_corners_2d = self.detections_queue[-1].corners_2d
            if not similar_corners_2d(corners_2d, tail_corners_2d):
                self.detections_queue.clear()

        # Discard older elements or with different detections, starting at the beginning of queue
        while self.detections_queue:
            # head_timestamp_ns = self.detections_queue[0].timestamp_ns
            head_corners_2d = self.detections_queue[0].corners_2d
            if not similar_corners_2d(corners_2d, head_corners_2d):
                self.detections_queue.popleft()
            else:
                break

        self.detections_queue.append(SimpleNamespace(
            timestamp_ns=timestamp_ns,
            corners_2d=corners_2d,
            camera_T_target=camera_T_target))

        return len(self.detections_queue) == self.detections_queue.maxlen

    def estimate_transform_to_plane(self, frame_id):
        """Estimate 3DoF ground_T_camera_canonical transformation. Results in terminal and file."""
        if not self.detections_queue:
            raise ValueError('Unexpected empty queue')
        # Get transform of the middle frame of the buffer
        idx_middle = len(self.detections_queue) // 2
        camera_optical_T_target = self.detections_queue[idx_middle].camera_T_target
        # transform from optical to canonical frame
        camera_canonical_T_target = self.canonical_T_optical @ camera_optical_T_target

        # Estimate translation in Z
        camera_z_up = camera_canonical_T_target[:3, :3] @ self.ground_z_up
        camera_z_up /= np.linalg.norm(camera_z_up)  # Ground normal in camera frame
        target_t_ground = np.array([0.0, 0.0, -self.target_detector.get_target_thickness(), 1.0])
        camera_canonical_t_ground = (camera_canonical_T_target @ target_t_ground)[:3]  # p in plane
        ground_t_camera_canonical = -1 * camera_canonical_t_ground
        distance_to_ground_plane = np.dot(ground_t_camera_canonical, camera_z_up)  # intercept

        # Estimate pitch and roll
        ground_axis_camera = np.cross(camera_z_up, self.ground_z_up)
        ground_axis_camera /= np.linalg.norm(ground_axis_camera)
        ground_angle_camera = np.arccos(np.dot(self.ground_z_up, camera_z_up))
        euler_rotation = scipy_rotation.from_rotvec(
            ground_axis_camera * ground_angle_camera).as_euler('XYZ', degrees=False)  # rpy format
        pitch = euler_rotation[1]
        roll = euler_rotation[0]

        # Try to crop _optical from end of frame:
        idx_find = frame_id.rfind('_optical')
        if idx_find < 0:
            camera_frame_id = frame_id
            self.get_logger().warning('Transformation estimated for canonical frame of the camera'
                                      f'and assuming {frame_id} is in optical frame')
        else:
            camera_frame_id = frame_id[:idx_find]

        output_str = self.get_output_string(camera_frame_id, distance_to_ground_plane, pitch, roll)
        self.get_logger().info(output_str)
        if self.output_path:
            with open(self.output_path, 'w', encoding='utf8') as fh:
                fh.write(f'{output_str}\n\n')

    def get_output_string(self, frame_id, translation_z, pitch, roll):
        return ('Plane calibration results (3-DoF):\n'
                f'plane_T_{frame_id}: z: {translation_z:.6f} m,  roll: {roll:.6f} rad, '
                f'pitch: {pitch:.6f} rad\n'
                '\nCalibration can be substituted as follows in a URDF file (using nominal '
                'values for the remaining non-estimated 3 DoF):\n'
                f'<joint name="{frame_id}_joint" type="fixed">\n'
                f'    <origin xyz="<nominal_x> <nominal_y> {translation_z:.6f}" '
                f'rpy="{roll:.6f} {pitch:.6f} <nominal_yaw>"/>\n'
                f'    <parent link="base_link"/>\n'
                f'    <child link="{frame_id}"/>\n'
                '</joint>')

    def rectified_image_callback(self, image_msg):
        if self.running and self.target_detector.is_ready():
            image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            camera_T_target, corners_2d = self.target_detector.detect(image)
            self.publish_detection(image, corners_2d, camera_T_target)
            if camera_T_target is not None and corners_2d is not None:
                image_timestamp_ns = rclpy.time.Time.from_msg(image_msg.header.stamp).nanoseconds
                ready_to_calibrate = self.store(image_timestamp_ns, corners_2d, camera_T_target)
                if ready_to_calibrate:
                    self.estimate_transform_to_plane(image_msg.header.frame_id)
                    if self.shutdown_on_detection:
                        self.get_logger().info('All done. Shutting down node')
                        self.running = False

    def publish_detection(self, image, corners_2d, camera_T_target):
        # Draw and display the corners
        if self.target_detector.is_ready():
            self.target_detector.plot_corners(image, corners_2d, camera_T_target)
            msg_to_publish = self.cv_bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self.detections_image_publisher.publish(msg_to_publish)

    def camera_info_callback(self, camera_info_msg):
        if self.running and not self.target_detector.is_ready():
            camera_intrinsics = np.array(camera_info_msg.k, dtype=np.float64).reshape(3, 3)
            optical_T_rectified = np.eye(4, dtype=np.float64)
            optical_T_rectified[:3, :3] = np.array(camera_info_msg.r).reshape(3, 3).transpose()
            self.get_logger().debug(f'Intrinsics received:\n{camera_intrinsics}')
            self.get_logger().debug(f'optical_T_rectified received:\n{optical_T_rectified}')
            self.target_detector.set_intrinsics(camera_intrinsics, optical_T_rectified)


def main(args=None):
    rclpy.init(args=args)
    plane_target_calibration = PlaneTargetCalibration()
    while plane_target_calibration.running and rclpy.ok():
        try:
            rclpy.spin_once(plane_target_calibration)
        except KeyboardInterrupt:
            # If user presses CTRL-C, return here to avoid calling rclpy.shutdown() twice
            return
    plane_target_calibration.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
