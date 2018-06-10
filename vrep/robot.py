from ur10_tf_dh import TF
import numpy as np
import ros
from ik import ik

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

        self._jacobian = np.zeros((6, 6))

    @property
    def q(self):
        return self._q

    @q.setter
    def q(self, v):
        for i in range(v.shape[0]):
            self._q[i] = v[i]

    def J(self, q=None):
        if q is None:
            q = self._q

        vec = [0, 0, 0]
        ee = self._tf.get_pose(q, vec, self._ee_frame, self._base_frame)
        ee_pos, ee_rot = UR10.vec_rot_from_matrix(ee)

        self._jacobian.fill(0)

        for i in range(self._base_frame, self._ee_frame):
            pose = self._tf.get_pose(q, vec, self._ee_frame, i)

            position, orientation = UR10.vec_rot_from_matrix(pose)

            radius_vector = np.subtract(ee_pos, position)
            rot_axis = (orientation[:, 2])    # Z axis

            jacobian_vec = np.cross(rot_axis, radius_vector)

            self._jacobian[:3, i-self._base_frame] = jacobian_vec[:]
            self._jacobian[3:, i-self._base_frame] = rot_axis[:]

        return self._jacobian

    def solve_ik(self, target_position, targer_quaternion):
        source_pose = self.get_pose()
        target_pose = [target_position, targer_quaternion]

        res = ikdamped(self.J, self._q, source_pose, target_pose,
                       self.get_pose, alpha=0.1, eps=0.1, damping_ratio=0.3,
                       kp=1.0, ko=1.0, max_iter=5000)

        if res[0]:
            return res[1]

        return None

    def get_pose(self, q=None):
        vec = [0, 0, 0]
        if q is None:
            q = self.q

        pose_relative_to_base = self._tf.get_pose(q, vec,
                                                  self._ee_frame,
                                                  self._base_frame)

        world_pose = np.dot(self._base_frame_pose, pose_relative_to_base)
        return UR10.vec_quat_from_matrix(world_pose)

    @staticmethod
    def vec_quat_from_matrix(mat):
        return (mat[:, 3])[:-1],\
            np.asarray(tf.quaternion_from_matrix(mat))

    @staticmethod
    def vec_rot_from_matrix(mat):
        return (mat[:, 3])[:-1], mat[:-1, :-1]


def test_ur10():
    ur10_world_position = np.asarray([-0.25, 0.25, 0.0447, 1])
    base_frame_pose = np.eye(4)
    base_frame_pose[:, 3] = ur10_world_position[:]

    ur10 = UR10(base_frame_pose=base_frame_pose)

    # ####################### FK #######################

    # First position
    q = np.zeros(6)
    ur10.q = q
    position, orientation = ur10.get_pose()
    print(position)
    print(np.asarray(tf.euler_from_quaternion(orientation, axes="szyx")) *
          180./np.pi)

    # Second position
    q = np.asarray([np.pi/4., np.pi/2., -np.pi/4., -np.pi/2., -np.pi/6., np.pi])
    ur10.q = q
    position, orientation = ur10.get_pose()
    print(position)
    print(np.asarray(tf.euler_from_quaternion(orientation, axes="szyx")) *
          180./np.pi)

    return 0
    # ################## IK #######################
    ik_required_position = position
    ik_required_orientation = orientation
    ik_required_pose = [ik_required_position, ik_required_orientation]

    ur10.q = np.zeros(6)
    ik_q = ur10.solve_ik(ik_required_position, ik_required_orientation)

    if ik_q is None:
        print("FAILED TO SOLVE IK")
    else:
        ik_pose = ur10.get_pose(q=ik_q)

        print("IK SOLVED SUCCESSFULLY")
        print("Source angles: " + str(np.zeros((6,))))
        print("Required angles: " + str(q))
        print("Source pose: " + str(ur10.get_pose()))
        print("Required pose: " + str(ik_required_pose))
        print("IK pose: " + str(ik_pose))


test_ur10()
