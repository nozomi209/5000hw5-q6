"""
Q6-2b 闭环极点配置 (重复极点 0.5, 0.5)
使用单输入 Ackermann 公式以允许重复极点。
"""

import numpy as np

A = np.array([[1.0, 1.0],
              [0.0, 1.0]])
B = np.array([[0.5],
              [1.0]])
xd = np.array([3.0, 0.0])
x0 = np.array([0.0, 0.0])
N = 50


def controllability(A, B):
    ctrb = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(A.shape[0])])
    return ctrb


def ackermann_gain(A, B, desired_poles):
    ctrb = controllability(A, B)
    if np.linalg.matrix_rank(ctrb) < A.shape[0]:
        raise ValueError("系统不可控，无法极点配置")
    poly = np.poly(desired_poles)  # lambda^n + a1 lambda^{n-1}+...
    phiA = sum(poly[i] * np.linalg.matrix_power(A, A.shape[0] - i) for i in range(len(poly)))
    e_nT = np.zeros((1, A.shape[0]))
    e_nT[0, -1] = 1.0
    K = e_nT @ np.linalg.inv(ctrb) @ phiA
    return K


def simulate_cl(A, B, K, x0, xd, steps=50):
    xs, us = [x0], []
    for _ in range(steps):
        u = (K @ (xs[-1] - xd)).item()
        us.append(u)
        xs.append(A @ xs[-1] + B.flatten() * u)
    return np.array(xs), np.array(us)


if __name__ == "__main__":
    desired_poles = [0.5, 0.5]
    K = ackermann_gain(A, B, desired_poles)
    xs, us = simulate_cl(A, B, K, x0, xd, steps=N)

    print("K (Ackermann, poles=0.5,0.5) =", K)
    print("前 5 步状态:\n", xs[:5])

