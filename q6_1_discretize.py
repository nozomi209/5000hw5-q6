"""
Q6-1 Discretization Verification and Numerical Matrices A, B
Model: Ts=1, m=1, g=10, F=mg+m*u_k => x_{k+1} = A x_k + B u_k
"""

import numpy as np

A = np.array([[1.0, 1.0],
              [0.0, 1.0]])
B = np.array([[0.5],
              [1.0]])

if __name__ == "__main__":
    print("A =\n", A)
    print("B =\n", B)

