# Take the product of two quaternions
import numpy as np
def quatProduct(q1, q2):
    r1 = q1[0]
    r2 = q2[0]
    v1 = np.array([q1[1], q1[2], q1[3]])
    v2 = np.array([q2[1], q2[2], q2[3]])

    r = r1 * r2 - np.dot(v1, v2)
    v = r1 * v2 + r2 * v1 + np.cross(v1, v2)
    q = np.array([r, v[0], v[1], v[2]])

    return q

a = quatProduct((0.707, 0.707, 0, 0),(0.707,0,0,0.707))
# a = [a[0], -a[1], -a[2], a[3]]
print(a)
# todo 第一个旋转不对