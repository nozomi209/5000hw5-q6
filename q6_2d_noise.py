"""
Q6-2d 有噪声的鲁棒性对比：开环最小能量 vs 闭环极点配置
绘图改用 plotly，避免 matplotlib 与 numpy 二进制不兼容。
运行后生成 q6_2d_noise_height.html。
"""

import numpy as np
import plotly.graph_objs as go

from q6_2a_openloop import min_energy_sequence
from q6_2b_place_ackermann import ackermann_gain

A = np.array([[1.0, 1.0],
              [0.0, 1.0]])
B = np.array([[0.5],
              [1.0]])
x0 = np.array([0.0, 0.0])
xf = np.array([2.0, 0.0])  # 开环目标
xd = np.array([3.0, 0.0])  # 闭环悬停
N = 50


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
        x_next = A @ x + B.flatten() * u + np.array([0.0, 0.02]) * w
        xs.append(x_next)
    return np.array(xs), np.array(us)


if __name__ == "__main__":
    u_seq = min_energy_sequence(A, B, xf, N).flatten()
    x_open_n, _ = simulate_noise(A, B, u_seq=u_seq, steps=N, seed=42)

    K = ackermann_gain(A, B, [0.5, 0.5])
    x_cl_n, _ = simulate_noise(A, B, K=K, x_ref=xd, steps=N, seed=42)

    t = np.arange(N + 1)
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=t, y=x_open_n[:, 0], mode="lines", name="开环高度(噪声)"))
    fig.add_trace(go.Scatter(x=t, y=x_cl_n[:, 0], mode="lines", name="闭环高度(噪声)"))
    fig.add_hline(y=xd[0], line=dict(dash="dash", color="black"), annotation_text="期望高度")
    fig.update_layout(title="噪声下高度对比", xaxis_title="k", yaxis_title="z")
    fig.write_html("q6_2d_noise_height.html", include_plotlyjs="cdn")
    print("已生成 q6_2d_noise_height.html，可用浏览器打开查看。")

