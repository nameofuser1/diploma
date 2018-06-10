import numpy as np
import ros

tf = ros.tf.transformations


class TF(object):

    ROT_AXIS_X = 'x'
    ROT_AXIS_Y = 'y'
    ROT_AXIS_Z = 'z'

    def __init__(self, r, alpha, d, offset):
        self.ndof = len(r)
        self.r = r
        self.alpha = alpha
        self.d = d
        self.offset = offset

        # X transforms
        self.X = [np.zeros((4, 4)) for i in range(self.ndof)]
        # self.__prepareXfull()

        # Z transform
        self.Z = [np.zeros((4, 4)) for i in range(self.ndof)]

        self.tf_mat = np.zeros((4, 4))
        self.tf_mat[3, 3] = 1.0

        self.shift_rot_mat = np.zeros((3, 3))

    def get_pose(self, q, vec, from_frame, to_frame):
        vec_homo = np.zeros((4,))
        vec_homo[:3] = vec[:]
        vec_homo[-1] = 1.0

        _q = np.add(self.offset, np.asarray(q))
        frame_tf = self.get_frame(_q, from_frame, to_frame)

        # print("Q with offset: " + str(_q))
        # print("Frame tf is:\r\n" + str(frame_tf))

        vec_pose = np.dot(frame_tf, vec_homo)
        frame_tf[:, 3] = vec_pose[:]

        return frame_tf

    def get_frame(self, q, from_frame, to_frame):
        frame_tf = self.__get_frame_transform(q, from_frame, to_frame)
        return frame_tf

    def __get_frame_transform(self, q, from_frame, to_frame):
        Z = np.zeros((4, 4))
        X = np.zeros((4, 4))

        tf_m = np.zeros((4, 4))
        tf_m[0, 0] = 1.0
        tf_m[1, 1] = 1.0
        tf_m[2, 2] = 1.0
        tf_m[3, 3] = 1.0

        if from_frame > to_frame:
            for i in range(to_frame, from_frame):
                self.__prepareZ(Z, q[i], self.d[i])
                self.__prepareX(X, self.alpha[i], self.r[i])

                tf_m = np.dot(tf_m, Z)
                tf_m = np.dot(tf_m, X)

        elif from_frame == to_frame:
            return np.eye(4)

        else:
            raise NotImplementedError("Transformation from parent to child"
                                      " is not implemeted yet")

        return tf_m

    def __prepareZ(self, Z, theta, d):
        self.__tf_rotation_config(Z, TF.ROT_AXIS_Z, theta)

        Z[0, 3] = 0.0
        Z[1, 3] = 0.0
        Z[2, 3] = d
        Z[3, 3] = 1.0

    def __prepareX(self, X, alpha, r):
        self.__tf_rotation_config(X, TF.ROT_AXIS_Y, alpha)

        X[0, 3] = r
        X[1, 3] = 0.0
        X[2, 3] = 0.0
        X[3, 3] = 1.0

    @staticmethod
    def __tf_rotation_config(M, axis, q):
        """
        @brief      Configures rotation part of transformation matrix

        @param[out] M       -   transformation matrix at least [3x3]
        @param[in]  axis    -   rotation axis
        @param[in]  q       -   joint angles
        """
        cs = np.cos
        sn = np.sin

        if axis == TF.ROT_AXIS_X:
            M[0, 0] = 1.
            M[1, 1] = cs(q)
            M[1, 2] = -sn(q)
            M[2, 1] = sn(q)
            M[2, 2] = cs(q)

        elif axis == TF.ROT_AXIS_Y:
            M[0, 0] = cs(q)
            M[0, 2] = sn(q)
            M[1, 1] = 1.
            M[2, 0] = -sn(q)
            M[2, 2] = cs(q)

        elif axis == TF.ROT_AXIS_Z:
            M[0, 0] = cs(q)
            M[0, 1] = -sn(q)
            M[1, 0] = sn(q)
            M[1, 1] = cs(q)
            M[2, 2] = 1.
