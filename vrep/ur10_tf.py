import numpy as np
import matplotlib.pyplot as plt
import vrep
from profiler import timeit
import ros

tf = ros.tf.transformations

# from utils import *
from ik import ik


ROT_AXIS_X = 'x'
ROT_AXIS_Y = 'y'
ROT_AXIS_Z = 'z'

AXIS_X = np.asarray([1, 0, 0])
AXIS_Y = np.asarray([0, 1, 0])
AXIS_Z = np.asarray([0, 0, 1])

UR10_DOF = 6

# Create symbols for each angle
# q = [sym.Symbol('q' + str(i)) for i in range(UR10_DOF)]

# Origins
O0 = np.asarray([0.0,     0.0,    0.0,    0.])
O1 = np.asarray([-0.2500, 0.2500, 0.0447, 0.])
O2 = np.asarray([-0.3427, 0.2500, 0.1280, 0.])
O3 = np.asarray([-0.3621, 0.2506, 0.7401, 0.])
O4 = np.asarray([-0.3555, 0.2500, 1.3123, 0.])
O5 = np.asarray([-0.4139, 0.2500, 1.3696, 0.])
O6 = np.asarray([-0.4712, 0.2500, 1.4282, 0.])
Oee = np.asarray([-0.4986, 0.2500, 1.4280, 0.])

# Relative origins
O01 = O1 - O0
O12 = O2 - O1
O23 = O3 - O2
O34 = O4 - O3
O45 = O5 - O4
O56 = O6 - O5
O6ee = Oee - O6

# Lengths of links
O01_len = np.linalg.norm(O01[:3])
O12_len = np.linalg.norm(O12[:3])
O23_len = np.linalg.norm(O23[:3])
O34_len = np.linalg.norm(O34[:3])
O45_len = np.linalg.norm(O45[:3])
O56_len = np.linalg.norm(O56[:3])
O6ee_len = np.linalg.norm(O6ee[:3])

WORLD_FRAME = 0
JOINT1_FRAME = 1
JOINT2_FRAME = 2
JOINT3_FRAME = 3
JOINT4_FRAME = 4
JOINT5_FRAME = 5
JOINT6_FRAME = 6
EE_FRAME = 7

TF01_ID = 0
TF12_ID = 1
TF23_ID = 2
TF34_ID = 3
TF45_ID = 4
TF56_ID = 5
TF67_ID = 6
TF_NUM = 7

# 1 for WORLD frame and 1 for EE FRAME
FRAMES_NUM = UR10_DOF + 2

# One additional trasformation is from World frame to UR10 base frame
TF = [np.zeros((4, 4), dtype=np.float64) for i in range(FRAMES_NUM)]

# Rotation signs for each transformation
# TF_SIGN = [1., 1., 1., 1., 1., 1., 1.]
TF_SIGN = [1., 1., -1., -1., -1., 1., -1.]

# List of relative origins
relative_origins = [O01, O12, O23, O34, O45, O56, O6ee]

# Rotation axis for each transformation
rotations_axis = [ROT_AXIS_Z, ROT_AXIS_Z, ROT_AXIS_X, ROT_AXIS_X,
                  ROT_AXIS_X, ROT_AXIS_Z, ROT_AXIS_X]

# Rotation unit vectors
rotation_axis_vec = [AXIS_Y, AXIS_Z, AXIS_X, AXIS_X, AXIS_X, AXIS_Z, AXIS_X]

# Jacobian placeholder
jacobian = np.zeros((6, 6), dtype=np.float64)

# Matrix used when rotating shift vector. Used in order not to allocate
# new on on each computation
shift_rotation_matrix = np.zeros((4, 4), dtype=np.float64)


def __tf_rotation_config(M, axis, q):
    """
    @brief      Configures rotation part of transformation matrix

    @param[out] M       -   transformation matrix at least [3x3]
    @param[in]  axis    -   rotation axis
    @param[in]  q       -   joint angles
    """
    cs = np.cos
    sn = np.sin

    if axis == ROT_AXIS_X:
        M[0, 0] = 1.
        M[1, 1] = cs(q)
        M[1, 2] = -sn(q)
        M[2, 1] = sn(q)
        M[2, 2] = cs(q)

    elif axis == ROT_AXIS_Y:
        M[0, 0] = cs(q)
        M[0, 2] = sn(q)
        M[1, 1] = 1.
        M[2, 0] = -sn(q)
        M[2, 2] = cs(q)

    elif axis == ROT_AXIS_Z:
        M[0, 0] = cs(q)
        M[0, 1] = -sn(q)
        M[1, 0] = sn(q)
        M[1, 1] = cs(q)
        M[2, 2] = 1.

    else:
        raise ValueError("Unknown rotation axis")


def __tf_shift_config(M, axis, q, ee_vec):
    shift_rotation_matrix.fill(0.)
    __tf_rotation_config(shift_rotation_matrix, axis, q)
    shift_vec = np.dot(shift_rotation_matrix, ee_vec)

    M[:, 3] = shift_vec[:]
    M[3, 3] = 1


def __tf_matrix_config(M, axis, q, ee_vec):
    """
    @brief      Configures transformation matrix

    @param[out] M       -   transformation matrix
    @param[in]  axis    -   axis of rotation
    @param[in]  q       -   angle of rotation
    @param[in]  ee_vec  -   end effector vector(from one joint to another)

    @raise      ValueError in case of wrong axis rotation
    @return     Nothing

    """
    M.fill(0.)
    __tf_rotation_config(M, axis, q)
    __tf_shift_config(M, axis, q, ee_vec)


def __prepare_transformation_matrices(q, from_frame, to_frame):
    """
    @brief      Prepares transformation matrices for given q.
                After matrices are configured transformation can be calculated
                with __transform call

    @param[in]  q           -   rotation angles
    @param[in]  frame_id    -   frame id from which we transform.
                                Transformation is to WORLD_FRAME
    """
    for i in range(to_frame, from_frame):
        if i == WORLD_FRAME:
            _q = 0.0
        else:
            _q = q[i-1]

        tf_m = TF[i]
        tf_sign = TF_SIGN[i]
        axis = rotations_axis[i]
        ee_vec = relative_origins[i]

        __tf_matrix_config(tf_m, axis, tf_sign*_q, ee_vec)


def __transform(vec, from_frame, to_frame):
    """
    Calculates transformation for given end effector

    @param[in] frame_id   -   specifies in which frame vector is placed
                              Transformation will be performed from this
                              frame to world one.
    """

    if from_frame == to_frame:
        tf_m = np.eye(4)
    elif from_frame - to_frame == 1:
        tf_m = TF[to_frame]
    else:
        tf_m = np.linalg.multi_dot(TF[to_frame:from_frame])

    if from_frame == EE_FRAME:
        tf_add_rot = np.zeros((4, 4))
        __tf_matrix_config(tf_add_rot, ROT_AXIS_Y,
                            -np.pi/2, [0., 0., 0., 1.])

        tf_m = np.dot(tf_m, tf_add_rot)

    position = np.dot(tf_m, vec)[:-1]
    orientation = tf_m

    return position, orientation


def transform(q, from_frame=EE_FRAME, to_frame=WORLD_FRAME):
    """
    """
    vec = np.asarray([0., 0., 0., 1.])
    __prepare_transformation_matrices(q, from_frame, to_frame)

    P, rot =  __transform(vec, from_frame, to_frame)
    return P, tf.quaternion_from_matrix(rot)


def J(q, use_orientation=True):
    ee = transform(q, from_frame=EE_FRAME, to_frame=WORLD_FRAME)

    for i in range(1, EE_FRAME):
        pose = transform(q, to_frame=WORLD_FRAME, from_frame=i)

        radius_vector = np.subtract(ee[0], pose[0])
        rot_axis = TF_SIGN[i]*rotation_axis_vec[i]

        jacobian_vec = np.cross(rot_axis, radius_vector)
        jacobian[:3, i-1] = jacobian_vec[:]

        if use_orientation:
            jacobian[3:, i-1] = TF_SIGN[i]*rot_axis[:]

    return jacobian


def vrep_connect_and_get_handles():
    vrep.simxFinish(-1)
    clientID = vrep.simxStart('127.0.0.1', 5450, True, True, 5000, 5)
    joints = []

    joint_base_name = "UR10_joint"
    if clientID == -1:
        raise ValueError()

    for i in range(1, 7):
        joint_name = joint_base_name + str(i)
        hndl = vrep.simxGetObjectHandle(clientID, joint_name,
                                        vrep.simx_opmode_blocking)

        if hndl[0] == -1:
            raise ValueError()

        joints.append(hndl[1])

    return clientID, joints


def __analyse_history(history):
    Jinv_mag = [np.linalg.norm(Jinv) for Jinv in history['Jinv']]

    print("J inverse max: " + str(np.max(Jinv_mag)))

    plt.xlabel("Iteration number, N")
    plt.ylabel("Magnitude, L2 norm")

    l1, = plt.plot(history['diff'], '-', label='Pose error magnitude')
    plt.legend(handles=[l1])
    plt.show()

    l3, = plt.plot(Jinv_mag, '-', label="J inverse magnitude")
    plt.legend(handles=[l3])
    plt.show()


def test_ik_damped():
    print("\r\nTesting IK via damped least squares")
    q_source = np.array([0., 0., 0., 0., 0., 0.])
    q_target = [np.pi/4., np.pi/2., -np.pi/4., -np.pi/2., -np.pi/6., np.pi]
    source = transform(q_source)
    target = transform(q_target)

    ikdls = timeit(ik.damped_least_squares)
    ik_status, ik_val, history = ikdls(J, q_source, source, target, transform,
                                       alpha=0.1, eps=0.1, damping_ratio=0.6,
                                       max_iter=5000, kp=1.0, ko=0.5)

    if not ik_status:
        print("FAILED TO SOLVE IK")

    ik_ee = transform(ik_val, from_frame=EE_FRAME, to_frame=WORLD_FRAME)

    print("Source pose is " + str(source[0]) + ", " +
          str(np.asarray(tf.euler_from_quaternion(source[1], axes="szyx")) *
              180./np.pi))
    print("Target pose is " + str(target[0]) + ", " +
          str(np.asarray(tf.euler_from_quaternion(target[1], axes="szyx")) *
              180./np.pi))
    print("IK pose is: " + str(ik_ee[0]) + ", " +
          str(np.asarray(tf.euler_from_quaternion(ik_ee[1], axes="szyx")) *
              180./np.pi))
    print("Target angles are: " + str(q_target))
    print("IK angles are: " + str(ik_val))

    __analyse_history(history)
    clientID, joints = vrep_connect_and_get_handles()

    for joint, q in zip(joints, ik_val):
        vrep.simxSetJointPosition(clientID, joint, q,
                                  vrep.simx_opmode_blocking)

    vrep.simxSynchronousTrigger(clientID)


def test_fk():
    print("Testing FK")

    q = [np.pi/4., np.pi/2., -np.pi/4., -np.pi/2., -np.pi/6., np.pi]

    src = transform(np.zeros((6,)), from_frame=EE_FRAME, to_frame=WORLD_FRAME)
    dst = transform(q, from_frame=EE_FRAME, to_frame=WORLD_FRAME)

    src_euler = np.asarray(tf.euler_from_quaternion(src[1], axes='szyx'),
                           dtype=np.float32)
    dst_euler = np.asarray(tf.euler_from_quaternion(dst[1], axes="szyx"),
                           dtype=np.float32)

    print("Source position: " + str(src[0]) + "; source orientation: " +
          str(src_euler*180./np.pi))

    print("Destination position: " + str(dst[0]) + "; destination orientation: "
          + str(dst_euler*180./np.pi))

    clientID, joints = vrep_connect_and_get_handles()
    for joint, _q in zip(joints, q):
        vrep.simxSetJointPosition(clientID, joint, _q,
                                  vrep.simx_opmode_blocking)

    vrep.simxSynchronousTrigger(clientID)

test_fk()
# test_ik_damped()
# test_ik_pseudo()
