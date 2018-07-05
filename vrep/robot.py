from ur10_tf_dh import TF
import numpy as np
import ros
from ik import ik
from robot_dynamics import ArmDynamics
from ur10_controller import UR10JointPositionController
from profiler import timeit


ikdamped = ik.damped_least_squares
tf = ros.tf.transformations


class UR10(object):

    def __init__(self, base_frame_pose=np.eye(4)):
        self.r = [-0.0927, 0.6121, 0.5722, 0.0573, 0.0573, 0.0]
        self.alpha = [-np.pi/2., 0.0, 0.0, np.pi/2., np.pi/2, 0.0]
        self.d = [0.0833, 0.0194, -0.0066, 0.0584, 0.0586, 0.0]
        self.offset = [0.0, 0.0, 0.0, 0.0, -np.pi, 0.0]

        self._base_frame_pose = base_frame_pose
        self._base_frame = 0
        self._ee_frame = 6
        self._tf = TF(self.r, self.alpha, self.d, self.offset)
        self._q = np.zeros((6,))

        self._ee_pose = np.eye(4)
        TF._prepareZ(self._ee_pose, np.pi, 0.0274)

        self._jacobian = np.zeros((6, 6))
        self._controller = UR10JointPositionController()
        self._dynamics_model = ArmDynamics(self._tf,
                                           self._base_frame, self._ee_frame,
                                           None, None)

    @property
    def q(self):
        return self._q

    @q.setter
    def q(self, v):
        for i in range(v.shape[0]):
            self._q[i] = v[i]

        self._controller.set_angles(self._q)

    def J(self, q=None):
        if q is None:
            q = self._q

        vec = np.eye(4)
        ee = self._tf.get_pose(q, vec, self._ee_frame, self._base_frame)
        ee_pos, ee_rot = UR10.vec_rot_from_matrix(ee)

        self._jacobian.fill(0)

        for i in range(self._base_frame, self._ee_frame):
            pose = self._tf.get_pose(q, vec, i, self._base_frame)

            position, orientation = UR10.vec_rot_from_matrix(pose)

            radius_vector = np.subtract(ee_pos, position)
            rot_axis = np.dot(orientation, [0, 0, 1])    # Z axis

            jacobian_vec = np.cross(rot_axis, radius_vector)

            self._jacobian[:3, i-self._base_frame] = jacobian_vec[:]
            self._jacobian[3:, i-self._base_frame] = rot_axis[:]

        return self._jacobian

    def solve_ik(self, target_position, targer_quaternion):
        source_pose = self.get_pose()
        target_pose = [target_position, targer_quaternion]

        res = ikdamped(self.J, self._q, source_pose, target_pose,
                       self.get_pose, alpha=0.1, eps=0.001, damping_ratio=0.3,
                       kp=1.0, ko=1.0, max_iter=5000)

        if res[0]:
            return res[1], res[2]

        return None, None

    def get_pose(self, q=None):
        ee_pose = np.eye(4)
        if q is None:
            q = self.q

        pose_relative_to_base = self._tf.get_pose(q, ee_pose,
                                                  self._ee_frame,
                                                  self._base_frame)

        world_pose = np.dot(self._base_frame_pose, pose_relative_to_base)
        ee_world_pose = np.dot(world_pose, self._ee_pose)

        return UR10.vec_quat_from_matrix(ee_world_pose)

    @staticmethod
    def vec_quat_from_matrix(mat):
        return (mat[:, 3])[:-1],\
            np.asarray(tf.quaternion_from_matrix(mat))

    @staticmethod
    def vec_rot_from_matrix(mat):
        return (mat[:, 3])[:-1], mat[:-1, :-1]

