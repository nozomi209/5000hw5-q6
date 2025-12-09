"""
Q6-2c 开环(最小能量) vs 闭环(极点配置) 仿真对比（无噪声）
绘图改用 plotly，避免 matplotlib 与 numpy 二进制不兼容。
运行后生成 q6_2c_compare.html，可浏览器查看。
"""

import numpy as np
import plotly.graph_objs as go

from q6_2a_openloop import min_energy_sequence, simulate_open
from q6_2b_place_ackermann import ackermann_gain, simulate_cl

A = np.array([[1.0, 1.0],
              [0.0, 1.0]])
B = np.array([[0.5],
              [1.0]])
x0 = np.array([0.0, 0.0])
xf = np.array([2.0, 0.0])  # 2a 目标
xd = np.array([3.0, 0.0])  # 2b 悬停
N = 50

if __name__ == "__main__":
    # 开环最小能量
    u_seq = min_energy_sequence(A, B, xf, N)
    x_open, u_open = simulate_open(A, B, u_seq, x0)

    # 闭环极点配置
    K = ackermann_gain(A, B, [0.5, 0.5])
    x_cl, u_cl = simulate_cl(A, B, K, x0, xd, steps=N)

    t = np.arange(N + 1)

    # 高度对比
    fig1 = go.Figure()
    fig1.add_trace(go.Scatter(x=t, y=x_open[:, 0], mode="lines", name="开环高度"))
    fig1.add_trace(go.Scatter(x=t, y=x_cl[:N + 1, 0], mode="lines", name="闭环高度"))
    fig1.add_hline(y=xd[0], line=dict(dash="dash", color="black"), annotation_text="期望高度")
    fig1.update_layout(title="高度对比", xaxis_title="k", yaxis_title="z")

    # 控制输入对比
    fig2 = go.Figure()
    fig2.add_trace(go.Scatter(x=np.arange(N), y=u_open, mode="lines", name="开环 u"))
    fig2.add_trace(go.Scatter(x=np.arange(N), y=u_cl[:N], mode="lines", name="闭环 u"))
    fig2.update_layout(title="控制输入对比", xaxis_title="k", yaxis_title="u")

    # 写 HTML
    fig1.write_html("q6_2c_compare_height.html", include_plotlyjs="cdn")
    fig2.write_html("q6_2c_compare_input.html", include_plotlyjs="cdn")

    print("已生成 q6_2c_compare_height.html 与 q6_2c_compare_input.html，可用浏览器打开查看。")

