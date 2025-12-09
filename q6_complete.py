import numpy as np
import plotly.graph_objs as go
from scipy.linalg import solve_discrete_are
from plotly.subplots import make_subplots
import plotly.offline as pyo

# ==================== Common Parameters ====================
A = np.array([[1.0, 1.0],
              [0.0, 1.0]])
B = np.array([[0.5],
              [1.0]])
x0 = np.array([0.0, 0.0])
N = 50
xf = np.array([2.0, 0.0])  # 2a target
xd = np.array([3.0, 0.0])  # 2b/2c/2d hover
xd_lqr = np.array([2.0, 0.0])  # 3 LQR target

# ==================== Helper Functions ====================
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

def ackermann_gain(A, B, desired_poles):
    ctrb = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(A.shape[0])])
    if np.linalg.matrix_rank(ctrb) < A.shape[0]:
        raise ValueError("System is not controllable, cannot place poles")
    poly = np.poly(desired_poles)
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

def dlqr(A, B, Q, R):
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A)
    return K

# ==================== Execute All Computations ====================
print("=" * 60)
print("HW5 Problem 6 Complete Solution")
print("=" * 60)

# 1. Discretization matrices
print("\n[1. Discretization Matrices]")
print("A =")
print(A)
print("B =")
print(B)

# 2a. Controllability and minimum energy open-loop
print("\n[2a. Controllability and Minimum Energy Open-Loop Control]")
rank_ctrb, ctrb = controllability(A, B)
u_seq = min_energy_sequence(A, B, xf, N)
x_open, u_open = simulate_open(A, B, u_seq, x0)
print(f"Controllability rank = {rank_ctrb}")
print(f"Final state x50 = {x_open[-1]}")
print(f"First 3 controls u0,u1,u2 = {u_open[:3]}")

# 2b. Pole placement
print("\n[2b. Closed-Loop Pole Placement (0.5, 0.5)]")
K_place = ackermann_gain(A, B, [0.5, 0.5])
x_cl, u_cl = simulate_cl(A, B, K_place, x0, xd, steps=N)
print(f"K (Ackermann) = {K_place}")
print(f"First 5 states:\n{x_cl[:5]}")

# 2c. Open-loop vs closed-loop comparison (no noise)
print("\n[2c. Open-Loop vs Closed-Loop Comparison (No Noise)]")
print("Computation completed, see plots below")

# 2d. Comparison with noise
print("\n[2d. Robustness Comparison with Noise]")
x_open_n, _ = simulate_noise(A, B, u_seq=u_open, steps=N, seed=42)
x_cl_n, _ = simulate_noise(A, B, K=K_place, x_ref=xd, steps=N, seed=42)
print("Computation completed, see plots below")

# 3. LQR
print("\n[3. LQR Control (Three Q,R Cases)]")
lqr_cases = [
    ("Q=I, R=1", np.eye(2), 1.0),
    ("Q=100I, R=1", 100 * np.eye(2), 1.0),
    ("Q=I, R=0.1", np.eye(2), 0.1),
]
lqr_results = []
for name, Q, R in lqr_cases:
    K = dlqr(A, B, Q, R)
    x_lqr, u_lqr = simulate_cl(A, B, K, x0, xd_lqr, steps=N)
    lqr_results.append((name, x_lqr, u_lqr, K))
    print(f"{name} -> K = {K}")

print("\n" + "=" * 60)
print("All computations completed, generating HTML report...")
print("=" * 60)

# ==================== Generate HTML Report ====================
t = np.arange(N + 1)

# Create subplot layout
fig = make_subplots(
    rows=4, cols=2,
    subplot_titles=(
        "2c: Height Comparison (No Noise)",
        "2c: Velocity Comparison (No Noise)",
        "2c: Control Input Comparison (No Noise)",
        "2d: Height Comparison (With Noise)",
        "3: LQR Height Comparison",
        "3: LQR Velocity Comparison",
        "3: LQR Control Input Comparison",
        "3: LQR States Summary"
    ),
    vertical_spacing=0.10,
    horizontal_spacing=0.1
)

# 2c: Height comparison
fig.add_trace(
    go.Scatter(x=t, y=x_open[:, 0], mode="lines", name="Open-loop Height", line=dict(color="blue"), showlegend=True),
    row=1, col=1
)
fig.add_trace(
    go.Scatter(x=t, y=x_cl[:N + 1, 0], mode="lines", name="Closed-loop Height", line=dict(color="red"), showlegend=True),
    row=1, col=1
)
fig.add_hline(y=xd[0], line=dict(dash="dash", color="black"), annotation_text="Desired Height", row=1, col=1)

# 2c: Velocity comparison
fig.add_trace(
    go.Scatter(x=t, y=x_open[:, 1], mode="lines", name="Open-loop Velocity", line=dict(color="blue"), showlegend=False),
    row=1, col=2
)
fig.add_trace(
    go.Scatter(x=t, y=x_cl[:N + 1, 1], mode="lines", name="Closed-loop Velocity", line=dict(color="red"), showlegend=False),
    row=1, col=2
)
fig.add_hline(y=0, line=dict(dash="dash", color="black"), annotation_text="Desired Velocity", row=1, col=2)

# 2c: Control input comparison
fig.add_trace(
    go.Scatter(x=np.arange(N), y=u_open, mode="lines", name="Open-loop u", line=dict(color="blue"), showlegend=False),
    row=2, col=1
)
fig.add_trace(
    go.Scatter(x=np.arange(N), y=u_cl[:N], mode="lines", name="Closed-loop u", line=dict(color="red"), showlegend=False),
    row=2, col=1
)

# 2d: Height comparison with noise
fig.add_trace(
    go.Scatter(x=t, y=x_open_n[:, 0], mode="lines", name="Open-loop Height (Noise)", line=dict(color="blue", dash="dot"), showlegend=False),
    row=2, col=2
)
fig.add_trace(
    go.Scatter(x=t, y=x_cl_n[:, 0], mode="lines", name="Closed-loop Height (Noise)", line=dict(color="red", dash="dot"), showlegend=False),
    row=2, col=2
)
fig.add_hline(y=xd[0], line=dict(dash="dash", color="black"), annotation_text="Desired Height", row=2, col=2)

# 3: LQR Height
for name, x_lqr, _, _ in lqr_results:
    fig.add_trace(
        go.Scatter(x=t, y=x_lqr[:, 0], mode="lines", name=f"{name} Height", showlegend=False),
        row=3, col=1
    )
fig.add_hline(y=xd_lqr[0], line=dict(dash="dash", color="black"), annotation_text="Desired Height", row=3, col=1)

# 3: LQR Velocity
for name, x_lqr, _, _ in lqr_results:
    fig.add_trace(
        go.Scatter(x=t, y=x_lqr[:, 1], mode="lines", name=f"{name} Velocity", showlegend=False),
        row=3, col=2
    )

# 3: LQR Control input
for name, _, u_lqr, _ in lqr_results:
    fig.add_trace(
        go.Scatter(x=np.arange(N), y=u_lqr, mode="lines", name=f"{name} u", showlegend=False),
        row=4, col=1
    )

# 3: LQR States summary (both height and velocity together)
for name, x_lqr, _, _ in lqr_results:
    fig.add_trace(
        go.Scatter(x=t, y=x_lqr[:, 0], mode="lines", name=f"{name} Height", showlegend=False),
        row=4, col=2
    )
    fig.add_trace(
        go.Scatter(x=t, y=x_lqr[:, 1], mode="lines", name=f"{name} Velocity", showlegend=False),
        row=4, col=2
    )

# Update layout
fig.update_layout(
    height=1600,
    title_text="HW5 Problem 6 Complete Solution - All Results Summary",
    title_x=0.5,
    showlegend=True
)

# Update x-axis and y-axis labels
for i in range(1, 5):
    for j in range(1, 3):
        fig.update_xaxes(title_text="k", row=i, col=j)
        if i == 1:
            if j == 1:
                fig.update_yaxes(title_text="z (Height)", row=i, col=j)
            else:
                fig.update_yaxes(title_text="ż (Velocity)", row=i, col=j)
        elif i == 2:
            if j == 1:
                fig.update_yaxes(title_text="u (Control Input)", row=i, col=j)
            else:
                fig.update_yaxes(title_text="z (Height)", row=i, col=j)
        elif i == 3:
            if j == 1:
                fig.update_yaxes(title_text="z (Height)", row=i, col=j)
            else:
                fig.update_yaxes(title_text="ż (Velocity)", row=i, col=j)
        else:  # i == 4
            if j == 1:
                fig.update_yaxes(title_text="u (Control Input)", row=i, col=j)
            else:
                fig.update_yaxes(title_text="States (z, ż)", row=i, col=j)

# Generate HTML content
plotly_html = fig.to_html(include_plotlyjs='cdn', div_id='plotly-div')

html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>HW5 Problem 6 Complete Solution</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }}
        .container {{
            max-width: 1400px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }}
        h1 {{
            color: #2c3e50;
            text-align: center;
            border-bottom: 3px solid #3498db;
            padding-bottom: 10px;
        }}
        h2 {{
            color: #34495e;
            margin-top: 30px;
            border-left: 4px solid #3498db;
            padding-left: 10px;
        }}
        .result-box {{
            background-color: #ecf0f1;
            padding: 15px;
            margin: 10px 0;
            border-radius: 5px;
            font-family: 'Courier New', monospace;
            white-space: pre-wrap;
        }}
        .result-box strong {{
            color: #e74c3c;
        }}
        .plot-container {{
            margin: 20px 0;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>HW5 Problem 6 Complete Solution</h1>
        
        <h2>1. Discretization Matrices</h2>
        <div class="result-box">
<strong>A =</strong>
{A}

<strong>B =</strong>
{B}
        </div>
        
        <h2>2a. Controllability and Minimum Energy Open-Loop Control</h2>
        <div class="result-box">
<strong>Controllability Rank:</strong> {rank_ctrb}
<strong>Final State x50:</strong> [{x_open[-1][0]:.6f}, {x_open[-1][1]:.6e}]
<strong>First 3 Controls:</strong> u0={u_open[0]:.6f}, u1={u_open[1]:.6f}, u2={u_open[2]:.6f}
        </div>
        
        <h2>2b. Closed-Loop Pole Placement (0.5, 0.5)</h2>
        <div class="result-box">
<strong>Feedback Gain K:</strong> [{K_place[0][0]:.6f}, {K_place[0][1]:.6f}]
<strong>First 5 States:</strong>
{x_cl[:5]}
        </div>
        
        <h2>2c. Open-Loop vs Closed-Loop Comparison (No Noise)</h2>
        <div class="result-box">
See plots below
        </div>
        
        <h2>2d. Robustness Comparison with Noise</h2>
        <div class="result-box">
See plots below (using same random seed seed=42)
        </div>
        
        <h2>3. LQR Control (Three Q,R Cases)</h2>
        <div class="result-box">
{chr(10).join([f'<strong>{name}:</strong> K = [{K[0][0]:.6f}, {K[0][1]:.6f}]' for name, _, _, K in lqr_results])}
        </div>
        
        <h2>All Plots Summary</h2>
        <div class="plot-container">
            {plotly_html}
        </div>
    </div>
</body>
</html>
"""

# Write HTML file
with open("q6_complete_report.html", "w", encoding="utf-8") as f:
    f.write(html_content)

print("\n✅ Complete report generated: q6_complete_report.html")
print("   Open in browser to view all results and plots!")

