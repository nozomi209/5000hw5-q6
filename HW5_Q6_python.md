# HW5 Problem 6 (UAV Vertical Height Control) Python Solution Documentation

> According to the problem: Ts=1, m=1, g=10, input F = mg + m·u_k, discrete model  
> \(x_{k+1} = A x_k + B u_k,\; x = [z,\ \dot z]^T\), initial \(x_0 = [0,0]^T\).  
> Matrices  
> \(A = \begin{bmatrix}1 & 1\\ 0 & 1\end{bmatrix},\;
> B = \begin{bmatrix}0.5\\ 1\end{bmatrix}\).

Python code and explanations for each subproblem (requires `numpy scipy matplotlib`).

---

## Common Code (Imports and Model)
```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import place_poles
from scipy.linalg import solve_discrete_are

A = np.array([[1., 1.],
              [0., 1.]])
B = np.array([[0.5],
              [1.]])
x0 = np.array([0., 0.])
```

---

## 1) Discretization Verification and Numerical A, B
```python
print("A=\n", A)
print("B=\n", B)
```

---

## 2a) Controllability and Minimum Energy Open-Loop (to \(x_{50} = [2,0]^T\))
```python
N = 50
xf = np.array([2., 0.])

# Controllability
ctrb = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(2)])
print("rank ctrb =", np.linalg.matrix_rank(ctrb))  # Expected to be 2

# Minimum norm open-loop sequence (C u = xf -> minimum norm solution)
C = np.hstack([np.linalg.matrix_power(A, N-1-i) @ B for i in range(N)])
gram = C @ C.T
u_seq = (C.T @ np.linalg.pinv(gram) @ xf).reshape(N, 1)  # u0...u49

def simulate_open(A, B, u_seq, x0):
    xs = [x0]
    for u in u_seq:
        xs.append(A @ xs[-1] + B.flatten() * u)
    return np.array(xs), np.array(u_seq).flatten()

x_open, u_open = simulate_open(A, B, u_seq, x0)
print("Final state x50 =", x_open[-1])
```

---

## 2b) Closed-Loop Pole Placement (Poles 0.5, 0.5)
```python
K_place = place_poles(A, B, [0.5, 0.5]).gain_matrix  # 1x2
print("K (place) =", K_place)

xd = np.array([3., 0.])  # Desired hover height

def simulate_cl(A, B, K, x0, xd, steps=50):
    xs, us = [x0], []
    for _ in range(steps):
        u = (K @ (xs[-1] - xd)).item()
        us.append(u)
        xs.append(A @ xs[-1] + B.flatten() * u)
    return np.array(xs), np.array(us)

x_cl, u_cl = simulate_cl(A, B, K_place, x0, xd)
print("Closed-loop first few states x:", x_cl[:5])
```

---

## 2c) Open-Loop vs Closed-Loop Simulation Comparison (No Noise)
```python
t = np.arange(N+1)
plt.figure(figsize=(10,4))
plt.subplot(1,2,1)
plt.plot(t, x_open[:,0], label="Open-loop Height")
plt.plot(t, x_cl[:N+1,0], label="Closed-loop Height")
plt.axhline(xd[0], ls='--', c='k', label='Desired Height')
plt.legend(); plt.grid(); plt.title("Height Comparison")

plt.subplot(1,2,2)
plt.plot(np.arange(N), u_open, label="Open-loop u")
plt.plot(np.arange(N), u_cl[:N], label="Closed-loop u")
plt.legend(); plt.grid(); plt.title("Control Input Comparison")
plt.tight_layout()
plt.show()
```

---

## 2d) Robustness Comparison with Noise (Velocity Equation Adds \(0.02\,w_k\))
```python
def simulate_noise(A, B, K=None, u_seq=None, x_ref=None, steps=50, seed=0):
    rng = np.random.default_rng(seed)
    xs, us = [x0], []
    for k in range(steps):
        x = xs[-1]
        if u_seq is not None:
            u = u_seq[k]
        else:
            xref = x_ref if x_ref is not None else np.zeros_like(x)
            u = (K @ (x - xref)).item()
        us.append(u)
        w = rng.normal(0, 1)
        x_next = A @ x + B.flatten()*u + np.array([0., 0.02])*w
        xs.append(x_next)
    return np.array(xs), np.array(us)

x_open_n, _ = simulate_noise(A, B, u_seq=u_open, steps=N, seed=42)
x_cl_n,   _ = simulate_noise(A, B, K=K_place, x_ref=xd, steps=N, seed=42)

t = np.arange(N+1)
plt.plot(t, x_open_n[:,0], label="Open-loop Height (Noise)")
plt.plot(t, x_cl_n[:,0],   label="Closed-loop Height (Noise)")
plt.axhline(xd[0], ls='--', c='k')
plt.legend(); plt.grid(); plt.title("Height Comparison Under Noise")
plt.show()
```

---

## 3) LQR (Infinite Horizon) Three Cases of \(Q,R\)
```python
def dlqr(A, B, Q, R):
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A)
    return K

lqr_cases = [
    ("Q=I, R=1",      np.eye(2), 1.0),
    ("Q=100I, R=1",   100*np.eye(2), 1.0),
    ("Q=I, R=0.1",    np.eye(2), 0.1),
]

results = []
for name, Q, R in lqr_cases:
    K = dlqr(A, B, Q, R)
    x_lqr, u_lqr = simulate_cl(A, B, K, x0, np.array([2.,0.]), steps=N)
    results.append((name, x_lqr, u_lqr, K))
    print(name, "K =", K)

t = np.arange(N+1)
fig, axes = plt.subplots(1,2, figsize=(10,4))
for name, x_lqr, u_lqr, _ in results:
    axes[0].plot(t, x_lqr[:,0], label=f"{name} Height")
    axes[1].plot(np.arange(N), u_lqr, label=f"{name} u")
axes[0].axhline(2, ls='--', c='k')
axes[0].set_title("LQR Height"); axes[0].grid(); axes[0].legend()
axes[1].set_title("LQR Control Input"); axes[1].grid(); axes[1].legend()
plt.tight_layout(); plt.show()
```

---

## Quick Run Order
1. Run "Common Code".  
2. Execute 1), 2a), 2b), 2c), 2d), 3) in sequence.  
3. Generated plots can be directly used in reports, printed K values and final states can be used to fill in numerical answers.
