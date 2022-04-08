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
