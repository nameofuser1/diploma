import sympy as sym


def homo_shift_rot_x(q, dv):
    cs = sym.cos
    sn = sym.sin

    return sym.Matrix([[1,  0,      0,      dv[0]],
                       [0,  cs(q),  -sn(q), dv[1]],
                       [0,  sn(q),  cs(q),  dv[2]],
                       [0,  0,      0,      1]])


def homo_shift_rot_y(q, dv):
    cs = sym.cos
    sn = sym.sin

    return sym.Matrix([[cs(q),  0,  sn(q),  dv[0]],
                       [0,      1,  0,      dv[1]],
                       [-sn(q), 0,  cs(q),  dv[2]],
                       [0,      0,  0,      1]])


def homo_shift_rot_z(q, dv):
    cs = sym.cos
    sn = sym.sin

    return sym.Matrix([[cs(q), -sn(q),      0,  dv[0]],
                       [sn(q),  cs(q),      0,  dv[1]],
                       [0,      0,          1,  dv[2]],
                       [0,      0,          0,  1]])


def homo_rot_x(q):
    cs = sym.cos
    sn = sym.sin

    return sym.Matrix([[1,  0,      0,      0],
                       [0,  cs(q),  -sn(q), 0],
                       [0,  sn(q),  cs(q),  0],
                       [0,  0,      0,      1]])


def homo_rot_y(q):
    cs = sym.cos
    sn = sym.sin

    return sym.Matrix([[cs(q),  0,  sn(q),  0],
                       [0,      1,  0,      0],
                       [-sn(q), 0,  cs(q),  0],
                       [0,      0,  0,      1]])


def homo_rot_z(q):
    cs = sym.cos
    sn = sym.sin

    return sym.Matrix([[cs(q),  -sn(q),     0,  0],
                       [sn(q),  cs(q),      0,  0],
                       [0,       0,         1,  0],
                       [0,       0,         0,  1]])
