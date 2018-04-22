import numpy as np
import sympy as sym

from utils import *
from ..ik import ik

UR10_DOF = 6

# Create symbols for each angle
q = [sym.Symbol('q' + str(i)) for i in range(UR10_DOF)]

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

# Transform expressions
tf01 = homo_shift_rot_z(0, O01)
tf12 = homo_shift_rot_z(q[0], homo_rot_z(q[0])*O12)
tf23 = homo_shift_rot_x(-q[1], homo_rot_x(-q[1])*O23)
tf34 = homo_shift_rot_x(-q[2], homo_rot_x(-q[2])*O34)
tf45 = homo_shift_rot_x(-q[3], homo_rot_x(-q[3])*O45)
tf56 = homo_shift_rot_z(q[4], homo_rot_z(q[4])*O56)
tf67 = homo_shift_rot_x(-q[5], homo_rot_x(-q[5])*O6ee)

tf07 = tf01*tf12*tf23*tf34*tf45*tf56*tf67

vec = np.asarray([0, 0, 0, 1])
ee_vec = tf07*vec


def transform(_q):
    subdict = {}
    for i in range(UR10_DOF):
        subdict[q[i]] = _q[i]

    val = np.array(ee_vec.subs(subdict)).astype(np.float64)
    return val.reshape(val.shape[0])[:-1]


J = ee_vec.jacobian(sym.Matrix(q))
print("Jacobian shape is " + str(J.shape))
J.row_del(-1)
print("Without row - " + str(J.shape))


def test():
    print(transform(sym.Matrix([0, 0, 0, 0, 0, 0])))
    print(transform(sym.Matrix([3.14/2, 0, 0, 0, 0, 0])))
    print(transform([0, 3.14/2, 0, 0, 0, 0]))
    print(transform([0, 0, 3.14/2, 0, 0, 0]))
    print(transform([0, 0, 0, 3.14/2, 0, 0]))
    print(transform([0, 0, 0, 0, 3.14/6, 0]))
    print(transform([0, 0, 0, 0, 0, 3.14]))

    print(transform([3.14/4, 3.14/2, -3.14/4, -3.14/2, -3.14/6, 3.14]))


def test_ik():
    q_source = np.array([0., 0., 0., 0., 0., 0.])
    q_target = np.array([3.14/4, 3.14/2, -3.14/4, -3.14/2, -3.14/6, 3.14])
    source = transform(q_source)
    target = transform(q_target)

    ik_val = ik.ik_pseudoinverse_jacobian(J, q, q_source, source, target, transform)
    ik_ee = transform(ik_val)

    print("Target angles are: " + str(q_target))
    print("Target position is " + str(target))
    print("IK solution is: " + str(ik_val))
    print("IK FK position is: " + str(ik_ee))
    print("IK EE vector error: " + str(np.subtract(target, ik_ee)))
    print("IK angles error: " + str(np.subtract(q_target, ik_val)))

test()
test_ik()
