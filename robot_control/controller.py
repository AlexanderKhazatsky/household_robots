import time

import numpy as np
from dm_control.utils import transformations
from dm_control.utils.transformations import (euler_to_quat, euler_to_rmat,
                                              mat_to_quat, quat_to_euler,
                                              quat_to_mat, rmat_to_euler)
from oculus_reader import OculusReader

from bridge_mujoco.robots.control_mode import ControlMode
"""README
1) place the headset exactly in the same orientation as the robot base;
2) headset coordinate system w.r.t to the robot base coordinate system:
oculus => robot:
x_robot = 0 * x + 0 * y - 1 * z
y_robot = -1 * x + 0 * y + 0 * z 
z_robot = 0 * x + 1 * y + 0 * z
3) robot coordinate system w.r.t global (only rotation)
robot => global:
x_global = 0 * x + -1 * y + 0 * z
y_global = 1 * x + 0 * y + 0 * z
z_global = 0 * x + 0 * y + 1 * z
"""

oculus_to_robot_mat_4d = np.asarray(
    [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]],
    dtype=np.float32)
oculus_to_robot_mat_3d = oculus_to_robot_mat_4d[:3, :3]

controller_off_message = '\n\nATTENTION: Controller is off! Press enter when you have turned it back on :)'


# https://stackoverflow.com/a/22167097
def quat_diff(quat1, quat2):
    # diff @ quat2 = quat1
    diff = transformations.quat_mul(quat1, transformations.quat_inv(quat2))
    return diff


class VRPolicy(object):

    def __init__(self,
                 spacial_gain: float = 1,
                 pos_gain: float = 20,
                 rot_gain: float = 5,
                 gripper_gain: float = 0.25):
        self.oculus_reader = OculusReader()
        self.hand_origin = {'pos': None, 'quat': None}
        self.vr_origin = {'pos': None, 'quat': None}
        self.reset_origin = True

        self.spacial_gain = spacial_gain
        self.pos_gain = pos_gain
        self.rot_gain = rot_gain
        self.gripper_gain = gripper_gain

    def _set_robot_orientation(self, info):
        # For all of them Det == 1 => proper rotation matrices.
        robot_euler = info.observation['info/robot_deg'] / 180.0 * np.pi
        self.robot_to_global_rmat = euler_to_rmat(robot_euler)
        self.robot_to_global_mat_4d = np.eye(4)
        self.robot_to_global_mat_4d[:3, :3] = self.robot_to_global_rmat
        self.global_to_robot_mat_4d = np.linalg.inv(
            self.robot_to_global_mat_4d)
        self.global_to_robot_mat_rmat = self.global_to_robot_mat_4d[:3, :3]

    def _read_sensor(self, wait=True, controller_id='r'):
        # Read Controller
        start_time = time.time()
        while True:
            poses, buttons = self.oculus_reader.get_transformations_and_buttons(
            )
            if poses == {} and time.time() - start_time > 5:
                input(controller_off_message)
            if poses != {} and not buttons['RG']:
                print('Pausing\rPausing.\rPausing...', end='\r', flush=True)
                self.reset_origin = True
                if not wait: break
            if poses != {} and buttons['RG']:
                break

        # Put Rotation Matrix In Robot Frame
        rot_mat = oculus_to_robot_mat_4d @ np.asarray(poses[controller_id])
        vr_pos, vr_quat = rot_mat[:3, 3], mat_to_quat(rot_mat)
        vr_pos *= self.spacial_gain

        # Get Handle Orientation For Visualization
        vis_rot_mat = self.robot_to_global_mat_4d @ rot_mat @ euler_to_rmat(
            [-0.25 * np.pi, np.pi, 0], 'XYZ', full=True)
        vis_quat = euler_to_quat(rmat_to_euler(vis_rot_mat, 'XYZ'))

        return vr_pos, vr_quat, buttons, {'visual_quat': vis_quat}

    def _read_observation(self, info):
        # Read Environment Observation
        hand_pos = info.observation['wx250s_tcp_pos']
        hand_quat = info.observation['wx250s_tcp_quat']

        # Put Info In Robot Frame
        hand_pos = self.global_to_robot_mat_4d[:3, :3] @ hand_pos
        hand_quat = mat_to_quat(
            self.global_to_robot_mat_4d @ quat_to_mat(hand_quat))

        return hand_pos, hand_quat

    def get_info(self):
        start_time = time.time()
        while True:
            poses, buttons = self.oculus_reader.get_transformations_and_buttons(
            )
            if poses == {} and time.time() - start_time > 5:
                input(controller_off_message)
            if poses != {}: break
        if buttons['A'] or buttons['B']: self.reset_origin = True
        return {
            'save_episode': buttons['A'],
            'delete_episode': buttons['B'],
            'pause': not buttons['RG']
        }

    def __call__(self, info, wait=True):
        # Set Robot Orientation
        self._set_robot_orientation(info)

        # Read Sensor
        vr_pos, vr_quat, buttons, oculus_info = self._read_sensor(wait=wait)
        gripper = np.array([1 - 2 * buttons['rightTrig'][0]], dtype=np.float32)

        # Read Observation
        hand_pos, hand_quat = self._read_observation(info)

        # Set Origins When Button Released Or Episode Started
        if self.reset_origin:
            self.hand_origin = {'pos': hand_pos, 'quat': hand_quat}
            self.vr_origin = {'pos': vr_pos, 'quat': vr_quat}
            self.reset_origin = False

        # Calculate Positional Action
        hand_pos_offset = hand_pos - self.hand_origin['pos']
        target_pos_offset = vr_pos - self.vr_origin['pos']
        pos_action = target_pos_offset - hand_pos_offset

        # Calculate Euler Action
        hand_quat_offsert = quat_diff(hand_quat, self.hand_origin['quat'])
        target_quat_offset = quat_diff(vr_quat, self.vr_origin['quat'])
        quat_action = quat_diff(target_quat_offset, hand_quat_offsert)
        euler_action = quat_to_euler(quat_action)

        # Organize Action
        action = {
            'arm':
            np.concatenate(
                [pos_action * self.pos_gain, euler_action * self.rot_gain]),
            'hand':
            gripper * self.gripper_gain,
            'vr_pos':
            self.robot_to_global_rmat @ pos_action,
            'vr_quat':
            oculus_info['visual_quat']
        }

        action_spec = info.observation['info/action_spec'].item()

        action['arm'] = np.clip(action['arm'], action_spec['arm'].minimum,
                                action_spec['arm'].maximum)
        action['hand'] = np.clip(action['hand'], action_spec['hand'].minimum,
                                 action_spec['hand'].maximum)
        return action