import numpy as np
import quaternion as quat


class IKFailedException(Exception):

    def __init__(self, msg):
        super(IKFailedException, self).__init__(msg)


def __create_history():
    history = {'J': [],
               'Jinv': [],
               'diff': [],
               'q': [],
               's': []}

    return history


def __update_history(history, J, Jinv, diff, q, s):
    history['J'].append(np.asarray(J))
    history['Jinv'].append(np.asarray(Jinv))
    history['diff'].append(diff)
    history['q'].append(np.remainder(q, 2*np.pi))
    history['s'].append(np.asarray(s))


def __compute_pseudoinverse_jacobian(J):
    return np.linalg.pinv(J)


def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])


def __quat2arr(quat):
    return np.asarray([quat.x, quat.y, quat.z, quat.w])


def __compute_quat_error(quat1, quat2):
    S = skew(quat2[0:3])
    e = np.dot(quat1[3], quat2[0:3]) - np.dot(quat2[3], quat1[0:3]) -\
        np.dot(S, quat1[0:3])

    return e


def damped_least_squares(J_fcn, q_start, s, t, fk_fcn,
                         alpha=0.01,
                         max_iter=10000, eps=0.01,
                         damping_ratio=0.1):
    history = __create_history()
    _alpha = alpha
    q = np.asarray(q_start)

    dmp_sq = np.square(damping_ratio)

    ee_pos_des = t[0]
    ee_quat_des = __quat2arr(t[1])
    ee_des = np.concatenate([ee_pos_des, ee_quat_des])

    dq = 0
    for i in range(max_iter):
        J = J_fcn(q)

        damped_mat = np.add(np.dot(J, J.T), np.eye(J.shape[0])*dmp_sq)
        damped_mat_inv = np.linalg.inv(damped_mat)

        ee_pos = s[0]
        ee_quat = __quat2arr(s[1])
        ee = np.concatenate([ee_pos, ee_quat])

        e_pos = np.subtract(ee_pos_des, ee_pos)
        e_quat = __compute_quat_error(ee_quat, ee_quat_des)
        # e = np.concatenate([e_pos, e_quat])
        e = e_pos

        dq = np.dot(np.dot(J.T, damped_mat_inv), e)

        q += dq*alpha
        s = fk_fcn(q)

        # ??? diff = np.linalg.norm(np.subtract(ee_des, ee))
        diff = np.linalg.norm(e)
        __update_history(history, J, damped_mat_inv, diff, q, s)

        if diff <= eps:
            return np.remainder(q, 2*np.pi), history
        elif i % 10 == 0:
            print("Quat error: " + str(e_quat))
            print("Pos error: " + str(e_pos))
            # print("Diff is " + str(diff))
            # alpha = min(max(_alpha*diff/start_diff, min_alpha), max_alpha)
