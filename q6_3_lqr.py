"""
Q6-3 LQR 三组 (Q,R) 仿真与控制增益
绘图改用 plotly，避免 matplotlib 与 numpy 二进制不兼容。
生成 q6_3_lqr_height.html 与 q6_3_lqr_input.html。
"""

import numpy as np
import plotly.graph_objs as go
from scipy.linalg import solve_discrete_are

from q6_2b_place_ackermann import simulate_cl

A = np.array([[1.0, 1.0],
              [0.0, 1.0]])
B = np.array([[0.5],
              [1.0]])
x0 = np.array([0.0, 0.0])
xd = np.array([2.0, 0.0])
N = 50


def dlqr(A, B, Q, R):
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A)
    return K


if __name__ == "__main__":
    lqr_cases = [
        ("Q=I, R=1", np.eye(2), 1.0),
        ("Q=100I, R=1", 100 * np.eye(2), 1.0),
        ("Q=I, R=0.1", np.eye(2), 0.1),
    ]

    results = []
    for name, Q, R in lqr_cases:
        K = dlqr(A, B, Q, R)
        x_lqr, u_lqr = simulate_cl(A, B, K, x0, xd, steps=N)
        results.append((name, x_lqr, u_lqr, K))
        print(f"{name} -> K = {K}")

    t = np.arange(N + 1)

    # 高度曲线
    fig_h = go.Figure()
    for name, x_lqr, _, _ in results:
        fig_h.add_trace(go.Scatter(x=t, y=x_lqr[:, 0], mode="lines", name=f"{name} 高度"))
    fig_h.add_hline(y=xd[0], line=dict(dash="dash", color="black"), annotation_text="期望高度")
    fig_h.update_layout(title="LQR 高度", xaxis_title="k", yaxis_title="z")

    # 控制输入
    fig_u = go.Figure()
    for name, _, u_lqr, _ in results:
        fig_u.add_trace(go.Scatter(x=np.arange(N), y=u_lqr, mode="lines", name=f"{name} u"))
    fig_u.update_layout(title="LQR 控制输入", xaxis_title="k", yaxis_title="u")

    fig_h.write_html("q6_3_lqr_height.html", include_plotlyjs="cdn")
    fig_u.write_html("q6_3_lqr_input.html", include_plotlyjs="cdn")
    print("已生成 q6_3_lqr_height.html 与 q6_3_lqr_input.html，可用浏览器查看。")

