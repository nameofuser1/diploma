

class ArmDynamics(object):

    def __init__(self, arm_tf, base_frame, ee_frame,
                 arm_com_poses, com_intertial_properties):
        self._arm_tf = arm_tf
        self._arm_com_poses = arm_com_poses
        self._com_ineratial_properties = com_intertial_properties
        self._base_frame = base_frame
        self._ee_frame = ee_frame
        self._frames_range = range(self._base_frame, self._ee_frame)
        self._com_poses = [None]*len(arm_com_poses)

    def __compute_com_poses(self, q):
        i = 0
        for frame, rel_com_pose in zip(self._frames_range, self._arm_com_poses):
            com_pose = self._arm_tf.get_pose(q, rel_com_pose,
                                             frame, self._base_frame)
            self._com_poses[i] = com_pose
            i += 1

        for i, pose in enumerate(self._com_poses):
            print(("COM %d pose is " % (i)) + str(pose))
