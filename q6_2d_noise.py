"""
Q6-2d Robustness Comparison with Noise: Open-Loop Minimum Energy vs Closed-Loop Pole Placement
Uses plotly for plotting to avoid matplotlib and numpy binary incompatibility.
Generates q6_2d_noise_height.html after running.
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
xf = np.array([2.0, 0.0])  # Open-loop target
xd = np.array([3.0, 0.0])  # Closed-loop hover
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
    fig.add_trace(go.Scatter(x=t, y=x_open_n[:, 0], mode="lines", name="Open-loop Height (Noise)"))
    fig.add_trace(go.Scatter(x=t, y=x_cl_n[:, 0], mode="lines", name="Closed-loop Height (Noise)"))
    fig.add_hline(y=xd[0], line=dict(dash="dash", color="black"), annotation_text="Desired Height")
    fig.update_layout(title="Height Comparison Under Noise", xaxis_title="k", yaxis_title="z")
    fig.write_html("q6_2d_noise_height.html", include_plotlyjs="cdn")
    print("Generated q6_2d_noise_height.html, open in browser to view.")

