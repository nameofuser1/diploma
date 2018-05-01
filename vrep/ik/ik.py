import numpy as np


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


def ik_pseudoinverse_jacobian(J_fcn, q_start, s, t, fk_fcn,
                              alpha=0.1, max_iter=10000, eps=0.01):
    """
    Computes inverse kinematics using pseudoinverse Jacobian

    @param[in]      J       -   Function which produces Jacobian J_fnc(q)
    @param[in]      q_start -   source joint values
    @param[in]      s       -   source position
    @param[in]      t       -   target position
    @param[in]      fk_fcn  -   function that computes forward kinematics
                                takes q as a parameter and returns end effector
                                position
    @param[in]      alpha   -   descent step
    @param[in]      max_iter -  maximum iterations number
    @param[in]      eps     -   error tolerance

    @raise          IKFailedException in case of solution failure

    @returns        q which corresponds to target position in case of success
                    None in case of failure
    """
    history = __create_history()
    _alpha = alpha
    q = np.asarray(q_start)
    start_diff = np.linalg.norm(np.subtract(t, s))

    dq = 0
    for i in range(max_iter):
        J = J_fcn(q)
        J_pinv = __compute_pseudoinverse_jacobian(J)

        e = np.subtract(t, s)
        dq = np.dot(J_pinv, e)
        q += dq*alpha

        s = fk_fcn(q)

        diff = np.linalg.norm(np.subtract(t, s))
        __update_history(history, J, J_pinv, diff, q, s)

        if diff <= eps:
            return np.remainder(q, 2*np.pi), history
        elif i % 10 == 0:
            print("Diff is " + str(diff))
            # alpha = min(max(_alpha*diff/start_diff, 0.001), 0.2)

    raise IKFailedException("Failed to compute IK with pseudo inverse Jacobian")


def damped_least_squares(J_fcn, q_start, s, t, fk_fcn,
                         alpha=0.3, min_alpha=0.05, max_alpha=0.35,
                         max_iter=10000, eps=0.01,
                         damping_ratio=0.3):
    history = __create_history()
    _alpha = alpha
    q = np.asarray(q_start)
    start_diff = np.linalg.norm(np.subtract(t, s))

    dmp_sq = np.square(damping_ratio)

    dq = 0
    for i in range(max_iter):
        J = J_fcn(q)

        damped_mat = np.add(np.dot(J, J.T), np.eye(3)*dmp_sq)
        damped_mat_inv = np.linalg.inv(damped_mat)

        e = np.subtract(t, s)
        dq = np.dot(np.dot(J.T, damped_mat_inv), e)

        q += dq*alpha
        s = fk_fcn(q)

        diff = np.linalg.norm(np.subtract(t, s))
        __update_history(history, J, damped_mat_inv, diff, q, s)

        if diff <= eps:
            return np.remainder(q, 2*np.pi), history
        elif i % 10 == 0:
            print("Diff is " + str(diff))
            # alpha = min(max(_alpha*diff/start_diff, min_alpha), max_alpha)
