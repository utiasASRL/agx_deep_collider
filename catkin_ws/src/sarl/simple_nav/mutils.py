import numpy as np

def carrot(xbar):
    """Overloaded operator. converts 3x1 vectors into a member of Lie Alebra so(3)
        Also, converts 6x1 vectors into a member of Lie Algebra se(3)
    Args:
        xbar (np.ndarray): if 3x1, xbar is a vector of rotation angles, if 6x1 a vector of 3 trans and 3 rot angles.
    Returns:
        np.ndarray: Lie Algebra 3x3 matrix so(3) if input 3x1, 4x4 matrix se(3) if input 6x1.
    """
    x = xbar.squeeze()
    if x.shape[0] == 3:
        return np.array([[0, -x[2], x[1]],
                         [x[2], 0, -x[0]],
                         [-x[1], x[0], 0]])
    elif x.shape[0] == 6:
        return np.array([[0, -x[5], x[4], x[0]],
                         [x[5], 0, -x[3], x[1]],
                         [-x[4], x[3], 0, x[2]],
                         [0, 0, 0, 1]])
    print('WARNING: attempted carrot operator on invalid vector shape')
    return xbar
    
def quaternionToRot(qin):
    """Converts a quaternion to a rotation  matrix
    Args:
        qin (np.ndarray) (4,) [qx, qy, qz, qw] quaternion
    Returns:
        C (np.ndarray) (3,3) rotation matrix
    """
    q = qin.copy().reshape(4, 1)
    if np.matmul(q.transpose(), q) < 1e-14:
        return np.identity(3)
    xi = q[:3].reshape(3, 1)
    eta = q[3, 0]
    C = (eta**2 - np.matmul(xi.transpose(), xi)) * np.identity(3) + \
        2 * np.matmul(xi, xi.transpose()) - 2 * eta * carrot(xi)
    return C

def rotToQuaternion(C):
    """Converts a rotation matrix to a quaternion
    Note that the space of unit-length quaternions is a double-cover of SO(3)
    which means that, C maps to +/- q, so q --> C --> +/- q
    Args:
        C (np.ndarray) (3,3) rotation matrix
    Returns:
        q (np.ndarray) (4,1) [qx, qy, qz, qw] quaternion
    """
    eta = 0.5 * np.sqrt((1 + np.trace(C)))
    if np.abs(eta) < 1e-14:
        eta = 0
        xi = np.sqrt(np.diag(0.5 * (C + np.identity(3))))
        q = np.array([xi[0], xi[1], xi[2], eta]).reshape(4, 1)
    else:
        phi = wrapto2pi(2 * np.arccos(max(min(eta, 1.0), -1.0)))
        eta = np.cos(phi / 2)
        xi_cross = (C.T - C) / (4 * eta)
        q = np.array([xi_cross[2, 1], xi_cross[0, 2], xi_cross[1, 0], eta]).reshape(4, 1)
    return q

def roll(r):
    return np.array([[1, 0, 0], [0, np.cos(r), np.sin(r)], [0, -np.sin(r), np.cos(r)]], dtype=np.float64)


def pitch(p):
    return np.array([[np.cos(p), 0, -np.sin(p)], [0, 1, 0], [np.sin(p), 0, np.cos(p)]], dtype=np.float64)


def yaw(y):
    return np.array([[np.cos(y), np.sin(y), 0], [-np.sin(y), np.cos(y), 0], [0, 0, 1]], dtype=np.float64)


def yawPitchRollToRot(y, p, r):
    return roll(r) @ pitch(p) @ yaw(y)

def rotToYawPitchRoll(C):
    i = 2
    j = 1
    k = 0
    c_y = np.sqrt(C[i, i]**2 + C[j, i]**2)
    if c_y > 1e-14:
        r = np.arctan2(C[j, i], C[i, i])
        p = np.arctan2(-C[k, i], c_y)
        y = np.arctan2(C[k, j], C[k, k])
    else:
        r = 0
        p = np.arctan2(-C[k, i], c_y)
        y = np.arctan2(-C[j, k], C[j, j])
    return y, p, r
