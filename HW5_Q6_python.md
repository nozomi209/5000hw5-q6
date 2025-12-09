# HW5 第 6 题（UAV 纵向高度控制）Python 解答文档

> 依题意：Ts=1，m=1，g=10，输入 F = mg + m·u_k，离散模型  
> \(x_{k+1} = A x_k + B u_k,\; x = [z,\ \dot z]^T\)，初始 \(x_0 = [0,0]^T\)。
> 矩阵  
> \(A = \begin{bmatrix}1 & 1\\ 0 & 1\end{bmatrix},\;
> B = \begin{bmatrix}0.5\\ 1\end{bmatrix}\)。

依次给出各小问的 Python 代码与说明（依赖 `numpy scipy matplotlib`）。

---

## 公共代码（导入与模型）
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

## 1) 离散化检验与数值 A, B
```python
print("A=\n", A)
print("B=\n", B)
```

---

## 2a) 可控性与最小能量开环（到 \(x_{50} = [2,0]^T\)）
```python
N = 50
xf = np.array([2., 0.])

# 可控性
ctrb = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(2)])
print("rank ctrb =", np.linalg.matrix_rank(ctrb))  # 期望为 2

# 最小范数开环序列（C u = xf -> 最小范数解）
C = np.hstack([np.linalg.matrix_power(A, N-1-i) @ B for i in range(N)])
gram = C @ C.T
u_seq = (C.T @ np.linalg.pinv(gram) @ xf).reshape(N, 1)  # u0...u49

def simulate_open(A, B, u_seq, x0):
    xs = [x0]
    for u in u_seq:
        xs.append(A @ xs[-1] + B.flatten() * u)
    return np.array(xs), np.array(u_seq).flatten()

x_open, u_open = simulate_open(A, B, u_seq, x0)
print("终态 x50 =", x_open[-1])
```

---

## 2b) 闭环极点配置（极点 0.5, 0.5）
```python
K_place = place_poles(A, B, [0.5, 0.5]).gain_matrix  # 1x2
print("K (place) =", K_place)

xd = np.array([3., 0.])  # 期望悬停高度

def simulate_cl(A, B, K, x0, xd, steps=50):
    xs, us = [x0], []
    for _ in range(steps):
        u = (K @ (xs[-1] - xd)).item()
        us.append(u)
        xs.append(A @ xs[-1] + B.flatten() * u)
    return np.array(xs), np.array(us)

x_cl, u_cl = simulate_cl(A, B, K_place, x0, xd)
print("闭环首几步 x:", x_cl[:5])
```

---

## 2c) 开环 vs 闭环仿真对比（无噪声）
```python
t = np.arange(N+1)
plt.figure(figsize=(10,4))
plt.subplot(1,2,1)
plt.plot(t, x_open[:,0], label="开环高度")
plt.plot(t, x_cl[:N+1,0], label="闭环高度")
plt.axhline(xd[0], ls='--', c='k', label='期望高度')
plt.legend(); plt.grid(); plt.title("高度对比")

plt.subplot(1,2,2)
plt.plot(np.arange(N), u_open, label="开环u")
plt.plot(np.arange(N), u_cl[:N], label="闭环u")
plt.legend(); plt.grid(); plt.title("控制输入对比")
plt.tight_layout()
plt.show()
```

---

## 2d) 有噪声（速度方程加 \(0.02\,w_k\)）鲁棒性对比
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
plt.plot(t, x_open_n[:,0], label="开环高度(噪声)")
plt.plot(t, x_cl_n[:,0],   label="闭环高度(噪声)")
plt.axhline(xd[0], ls='--', c='k')
plt.legend(); plt.grid(); plt.title("噪声下高度对比")
plt.show()
```

---

## 3) LQR（无限时域）三组 \(Q,R\)
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
    axes[0].plot(t, x_lqr[:,0], label=f"{name} 高度")
    axes[1].plot(np.arange(N), u_lqr, label=f"{name} u")
axes[0].axhline(2, ls='--', c='k')
axes[0].set_title("LQR 高度"); axes[0].grid(); axes[0].legend()
axes[1].set_title("LQR 控制输入"); axes[1].grid(); axes[1].legend()
plt.tight_layout(); plt.show()
```

---

## 快速运行顺序
1. 运行“公共代码”。  
2. 依次执行 1)、2a)、2b)、2c)、2d)、3)。  
3. 生成的图即可直接用于报告，打印的 K 与终态用于填答数值。

