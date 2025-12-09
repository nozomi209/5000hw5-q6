
import numpy as np

A = np.array([[1.0, 1.0],
              [0.0, 1.0]])
B = np.array([[0.5],
              [1.0]])
x0 = np.array([0.0, 0.0])
N = 50
xf = np.array([2.0, 0.0])


def controllability(A, B):
    ctrb = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(A.shape[0])])
    return np.linalg.matrix_rank(ctrb), ctrb


def min_energy_sequence(A, B, x_target, N):
    C = np.hstack([np.linalg.matrix_power(A, N - 1 - i) @ B for i in range(N)])
    gram = C @ C.T
    u = C.T @ np.linalg.pinv(gram) @ x_target
    return u.reshape(N, 1)


def simulate_open(A, B, u_seq, x0):
    xs = [x0]
    for u in u_seq:
        xs.append(A @ xs[-1] + B.flatten() * u)
    return np.array(xs), np.array(u_seq).flatten()


if __name__ == "__main__":
    rank_ctrb, ctrb = controllability(A, B)
    u_seq = min_energy_sequence(A, B, xf, N)
    xs, us = simulate_open(A, B, u_seq, x0)

    print("Controllability rank =", rank_ctrb)
    print("Final state x50 =", xs[-1])
    print("First 3 controls u0,u1,u2 =", us[:3])

