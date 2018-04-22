# import sympy as sym
import numpy as np


class IKFailedException(Exception):

    def __init__(self, msg):
        super(IKFailedException, self).__init__(msg)


def __create_subdict(q_sym, q_val):
    subdict = {}
    for _sym, val in zip(q_sym, q_val):
        subdict[_sym] = val

    return subdict


def __compute_pseudoinverse_jacobian(J, q_sym, q_val):
    subdict = __create_subdict(q_sym, q_val)
    J_val = np.array(J.subs(subdict)).astype(np.float64)

    return np.linalg.pinv(J_val)


def ik_pseudoinverse_jacobian(J, q_sym, q_val, s, t, fk_fcn,
                              alpha=0.1, max_iter=10000, eps=0.01):
    """
    Computes inverse kinematics using pseudoinverse Jacobian

    @param[in]      J       -   symbolic Jacobian
    @param[in]      q_sym   -   symbols used in Jacobian
    @param[in]      q_val   -   current joint values
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
    print("Entry")
    q = np.asarray(q_val)
    start_diff = np.linalg.norm(np.subtract(t, s))

    dq = 0
    for i in range(max_iter):
        J_pinv = __compute_pseudoinverse_jacobian(J, q_sym, q)
        # print(J_pinv)

        e = np.subtract(t, s)
        # print(e)
        dq = np.dot(J_pinv, e)
        # print(dq)
        # print(q)
        q += dq*alpha
        # print(q)

        s = fk_fcn(q)

        diff = np.linalg.norm(np.subtract(t, s))
        if diff <= eps:
            return np.remainder(q, 2*np.pi)
        elif i % 10 == 0:
            print("Diff is " + str(diff))
            # alpha = max(alpha*diff/start_diff, 0.01)

    raise IKFailedException("Failed to compute IK with pseudo inverse Jacobian")
