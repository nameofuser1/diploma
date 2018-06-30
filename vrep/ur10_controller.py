import vrep


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


class UR10JointPositionController(object):

    def __init__(self):
        client_id, joint_handles = vrep_connect_and_get_handles()
        self._client_id = client_id
        self._joint_handles = joint_handles

    def set_angles(self, angles):
        for q, handle in zip(angles, self._joint_handles):
            vrep.simxSetJointPosition(self._client_id, handle, q,
                                      vrep.simx_opmode_blocking)

        vrep.simxSynchronousTrigger(self._client_id)
