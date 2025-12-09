"""
Q6-2c Open-Loop (Minimum Energy) vs Closed-Loop (Pole Placement) Simulation Comparison (No Noise)
Uses plotly for plotting to avoid matplotlib and numpy binary incompatibility.
Generates q6_2c_compare.html after running, viewable in browser.
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
xf = np.array([2.0, 0.0])  # 2a target
xd = np.array([3.0, 0.0])  # 2b hover
N = 50

if __name__ == "__main__":
    # Open-loop minimum energy
    u_seq = min_energy_sequence(A, B, xf, N)
    x_open, u_open = simulate_open(A, B, u_seq, x0)

    # Closed-loop pole placement
    K = ackermann_gain(A, B, [0.5, 0.5])
    x_cl, u_cl = simulate_cl(A, B, K, x0, xd, steps=N)

    t = np.arange(N + 1)

    # Height comparison
    fig1 = go.Figure()
    fig1.add_trace(go.Scatter(x=t, y=x_open[:, 0], mode="lines", name="Open-loop Height"))
    fig1.add_trace(go.Scatter(x=t, y=x_cl[:N + 1, 0], mode="lines", name="Closed-loop Height"))
    fig1.add_hline(y=xd[0], line=dict(dash="dash", color="black"), annotation_text="Desired Height")
    fig1.update_layout(title="Height Comparison", xaxis_title="k", yaxis_title="z")

    # Control input comparison
    fig2 = go.Figure()
    fig2.add_trace(go.Scatter(x=np.arange(N), y=u_open, mode="lines", name="Open-loop u"))
    fig2.add_trace(go.Scatter(x=np.arange(N), y=u_cl[:N], mode="lines", name="Closed-loop u"))
    fig2.update_layout(title="Control Input Comparison", xaxis_title="k", yaxis_title="u")

    # Write HTML
    fig1.write_html("q6_2c_compare_height.html", include_plotlyjs="cdn")
    fig2.write_html("q6_2c_compare_input.html", include_plotlyjs="cdn")

    print("Generated q6_2c_compare_height.html and q6_2c_compare_input.html, open in browser to view.")

