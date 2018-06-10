from ur10_tf_dh import TF
import numpy as np
import ros


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

    @property
    def q(self):
        return self._q

    @q.setter
    def q(self, v):
        for i in range(v.shape[0]):
            self._q[i] = v[i]

    def get_pose(self):
        vec = [0, 0, 0]
        print(self.q)
        pose_relative_to_base = self._tf.get_pose(self.q, vec,
                                                  self._ee_frame,
                                                  self._base_frame)

        world_pose = np.dot(self._base_frame_pose, pose_relative_to_base)

        position = (world_pose[:, 3])[:-1]
        orientation = tf.quaternion_from_matrix(world_pose)

        return position, orientation


def test_ur10():
    ur10_world_position = np.asarray([-0.25, 0.25, 0.0447, 1])
    base_frame_pose = np.eye(4)
    base_frame_pose[:, 3] = ur10_world_position[:]

    ur10 = UR10(base_frame_pose=base_frame_pose)

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


test_ur10()
