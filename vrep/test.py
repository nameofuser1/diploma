import numpy as np
from robot import UR10
from ur10_tf_dh import TF
from profiler import timeit
from matplotlib import pyplot as plt


def __analyse_history(history):
    Jinv_mag = [np.linalg.norm(Jinv) for Jinv in history['Jinv']]

    print("J inverse max: " + str(np.max(Jinv_mag)))

    plt.xlabel("Iteration number, N")
    plt.ylabel("Magnitude, L2 norm")

    plot1 = plt.subplot(211)
    l1, = plot1.plot(history['diff'], '-', label='Pose error magnitude')
    plt.legend(handles=[l1])

    plot2 = plt.subplot(212)
    l3, = plot2.plot(Jinv_mag, '-', label="J inverse magnitude")
    plt.legend(handles=[l3])
    plt.show()


def test_fk(ur10):
    print("---------------- TESTING FK -----------------")

    # First position
    q = np.zeros(6)
    ur10.q = q
    position, orientation = ur10.get_pose()
    print("Angles: " + str(q))
    print("Position: " + str(position))
    print("Orientation: " + str(TF.quat2euler(orientation,
                                              axes="szyx", deg=True)))
    print("")

    # Second position
    q = np.asarray([np.pi/4., np.pi/2., -np.pi/4., -np.pi/2., -np.pi/6., np.pi])
    ur10.q = q
    position, orientation = ur10.get_pose()
    print("Angles: " + str(q))
    print("Position: " + str(position))
    print("Orientation: " + str(TF.quat2euler(orientation,
                                              axes="szyx", deg=True)))


def test_ik(ur10):
    print("\r\n----------------- TESTING IK ------------------")
    ur10.q = np.zeros((6,))

    q = np.asarray([np.pi/4., np.pi/2., -np.pi/4., -np.pi/2., -np.pi/6., np.pi])
    position, orientation = ur10.get_pose(q=q)

    ik_required_position = position
    ik_required_orientation = orientation

    print("Required pose: " + str(ik_required_position) +
          " " + str(TF.quat2euler(ik_required_orientation, deg=True)))
    from time import sleep
    sleep(2)

    ur10.q = np.zeros((6,))
    ik_fcn = timeit(ur10.solve_ik)
    ik_q, history = ik_fcn(ik_required_position, ik_required_orientation)

    if ik_q is None:
        print("FAILED TO SOLVE IK")
    else:
        ur10.q = ik_q
        ik_pose = ur10.get_pose()

        print("IK SOLVED SUCCESSFULLY")
        print("IK angles: " + str(ik_q))
        print("IK pose: " + str(ik_pose[0]) + " " +
              str(TF.quat2euler(ik_pose[1], deg=True)))

        __analyse_history(history)


def test_ur10():
    ur10_world_position = np.asarray([-0.25, 0.25, 0.0447, 1])
    base_frame_pose = np.eye(4)
    base_frame_pose[:, 3] = ur10_world_position[:]

    ur10 = UR10(base_frame_pose=base_frame_pose)

    # ####################### FK #######################
    # test_fk(ur10)

    # ################## IK #######################
    test_ik(ur10)

test_ur10()
