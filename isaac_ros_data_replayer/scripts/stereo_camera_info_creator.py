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

from argparse import ArgumentParser
import os
import sys
from typing import List, Tuple

from camera_info_publisher import load_camera_info
from camera_info_writer import write_camera_info
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as scipy_rotation
from sensor_msgs.msg import CameraInfo


def update_rectification_parameters(left_msg: CameraInfo, right_msg: CameraInfo, alpha: float,
                                    right_T_left: np.ndarray) -> Tuple[CameraInfo, CameraInfo]:
    """
    Update R and P fields in the input CameraInfo messages according to extrinsics and alpha.

    Compute rectification parameters using opencv and updates rotation and projection matrices in
    the input messages.

    Parameters
    ----------
    left_msg : CameraInfo
        Left CameraInfo message.
    right_msg : CameraInfo
        Right CameraInfo message.
    right_T_left : np.ndarray
        right_T_left transformation as a 4x4 rotation matrix.
    alpha : float
        Opencv's stereoRectify alpha parameter.

    Returns
    -------
    Tuple[CameraInfo, CameraInfo]
        Tuple (left_msg, right_msg) packing the two updated messages.

    """
    rotation_left, rotation_right, projection_left, projection_right, _, _, _ = cv2.stereoRectify(
        cameraMatrix1=left_msg.k.reshape((3, 3)),
        distCoeffs1=np.asarray(left_msg.d, dtype=np.float64),
        cameraMatrix2=right_msg.k.reshape((3, 3)),
        distCoeffs2=np.asarray(right_msg.d, dtype=np.float64),
        imageSize=(left_msg.width, left_msg.height),
        R=right_T_left[:3, :3],
        T=right_T_left[:3, 3],
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=alpha)

    left_msg.r = rotation_left.reshape(9,)
    left_msg.p = projection_left.reshape(12,)
    right_msg.r = rotation_right.reshape(9,)
    right_msg.p = projection_right.reshape(12,)

    return left_msg, right_msg


def compute_right_T_left(left_msg: CameraInfo, right_msg: CameraInfo) -> np.ndarray:
    """
    Compute extrinsic calibration between left and right images from the two camera info messages.

    Parameters
    ----------
    left_msg : CameraInfo
        Left CameraInfo message.
    right_msg : CameraInfo
        Right CameraInfo message.

    Returns
    -------
    np.ndarray
        right_T_left transformation as a 4x4 rotation matrix.

    """
    def matrix4x4(rotation, translation):
        pose_3d = np.eye(4, dtype=np.float64)
        if rotation is not None:
            pose_3d[:3, :3] = rotation.reshape((3, 3))
        if translation is not None:
            pose_3d[:3, 3] = translation
        return pose_3d

    # Transpose is inverse when translation is zero
    right_T_rectified_right = matrix4x4(right_msg.r, None).transpose()
    f_x = right_msg.p[0]
    rectified_right_T_rectified_left = matrix4x4(None, (1.0/f_x) * right_msg.p.reshape(3, 4)[:, 3])
    rectified_left_T_left = matrix4x4(left_msg.r, None)
    return right_T_rectified_right @ rectified_right_T_rectified_left @ rectified_left_T_left


def get_right_T_left(left_msg: CameraInfo, right_msg: CameraInfo,
                     right_T_left_quat_xyzw_list: List[float],
                     right_T_left_rot_vec_rad_list: List[float],
                     verbose: bool = False, tolerance: float = 1e-06) -> np.ndarray:
    """
    Help assemble the right_T_left transformation based on input arguments.

    If extrinsics are not explicitly provided, it computes them from camera_info messages.

    Parameters
    ----------
    left_msg : CameraInfo
        Left CameraInfo message.
    right_msg : CameraInfo
        Right CameraInfo message.
    right_T_left_quat_xyzw_list : List[float]
        right_T_left transformation in the format [x y z qx qy qz qw].
    right_T_left_rot_vec_rad_list : List[float]
        right_T_left transformation in the format [x y z rx ry rz].
    verbose : bool
        Whether to print extra traces or not.
    tolerance : float
        Epsilon to consider two values equal.


    Returns
    -------
    np.ndarray
        right_T_left transformation as a 4x4 rotation matrix.

    """
    if right_T_left_quat_xyzw_list:
        right_T_left = np.eye(4, dtype=np.float64)
        rotation_quat_xyzw = np.array(right_T_left_quat_xyzw_list[3:])
        right_T_left[:3, :3] = scipy_rotation.from_quat(rotation_quat_xyzw).as_matrix()
        right_T_left[:3, 3] = np.array(right_T_left_quat_xyzw_list[:3])
    elif right_T_left_rot_vec_rad_list:
        right_T_left = np.eye(4, dtype=np.float64)
        rotation_rot_vec_radians = np.array(right_T_left_rot_vec_rad_list[3:])
        right_T_left[:3, :3] = scipy_rotation.from_rotvec(rotation_rot_vec_radians).as_matrix()
        right_T_left[:3, 3] = np.array(right_T_left_rot_vec_rad_list[:3])
    else:
        print('Estimating stereo extrinsics from camera_info messages...')
        right_T_left = compute_right_T_left(left_msg, right_msg)

    baseline = np.linalg.norm(right_T_left[:3, 3])
    if verbose or baseline == 0.0:
        with np.printoptions(precision=10, suppress=True):
            print(f'right_T_left matrix:\n{right_T_left}')
            rotation = right_T_left[:3, :3]
            print(f'quaternion:\n{scipy_rotation.from_matrix(rotation).as_quat()}')
            print(f'rotation vector:\n{scipy_rotation.from_matrix(rotation).as_rotvec()}')
            print(f'baseline: {baseline}')
            print(f'translation: {right_T_left[:3, 3]}')
    if baseline < tolerance:
        raise ValueError(f'Baseline cannot be close to zero: {baseline}. If estimating intrinsics '
                         'from camera_info messages, please make sure R and P are not empty or '
                         'identity.')
    return right_T_left


def parse_args():
    parser = ArgumentParser(prog='stereo_camera_info_creator.py')
    parser.add_argument('--input_left',
                        help='Input path to the left camera info',
                        type=str,
                        required=True)
    parser.add_argument('--input_right',
                        help='Input path to the right camera info',
                        type=str,
                        required=True)
    parser.add_argument('--alpha',
                        help='Rectification alpha parameter, Default: 0',
                        default=0,
                        type=float,
                        required=False)
    parser.add_argument('--output_left',
                        help='Output path to the left camera info',
                        type=str,
                        required=True)
    parser.add_argument('--output_right',
                        help='Output path to the right camera info',
                        type=str,
                        required=True)
    parser.add_argument('--right_T_left_quat',
                        help='Optional input extrinsics in the format "x y z qx qy qz qw". '
                        'The x y z values is the translation in meters, and qx qy qz qw is the '
                        'rotation quaternion',
                        nargs=7,
                        default=[],
                        type=float,
                        required=False)
    parser.add_argument('--right_T_left_rot_vec_rad',
                        help='Optional input extrinsics in the format "x y z rx ry rz". The '
                        'x y z values is the translation in meters, and rx ty rz is the rodrigez '
                        'vector in radians',
                        nargs=6,
                        default=[],
                        type=float,
                        required=False)
    parser.add_argument('--verbose',
                        '-v',
                        help='Whether to print extra traces or not',
                        action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()

    if not os.path.exists(args.input_left) or not os.path.exists(args.input_right):
        print(f'Input paths must exist:\n - {args.input_left}\n - {args.input_right}')
        sys.exit(1)

    if os.path.exists(args.output_left) or os.path.exists(args.output_right):
        print(f'Output paths must not exist:\n - {args.output_left}\n - {args.output_right}')
        sys.exit(1)

    if args.right_T_left_rot_vec_rad and args.right_T_left_quat:
        print('Please only provide one of: right_T_left_rot_vec_rad, right_T_left_quat (or none)')
        sys.exit(1)

    left_msg = load_camera_info(args.input_left)
    right_msg = load_camera_info(args.input_right)

    right_T_left = get_right_T_left(left_msg, right_msg, args.right_T_left_quat,
                                    args.right_T_left_rot_vec_rad, verbose=args.verbose)

    left_msg, right_msg = update_rectification_parameters(left_msg, right_msg, args.alpha,
                                                          right_T_left)
    write_camera_info(left_msg, args.output_left)
    write_camera_info(right_msg, args.output_right)
    print(f'Files successfully created:\n - {args.output_left}\n - {args.output_right}')


if __name__ == '__main__':
    main()
