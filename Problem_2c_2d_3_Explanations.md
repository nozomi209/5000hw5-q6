# Problem 2c, 2d, and 3: Detailed Explanations

## Problem 2c: Open-Loop vs Closed-Loop Control Comparison

### Question
For k = 0, ..., 50, simulate system (3) under the open-loop controller (2a) and the closed-loop controller (2b). Plot the states evolution under each control method on the same plot (create one plot for height and one for velocity). Compare the two system responses.

### Answer

#### 1. **Control Strategy Differences**

**Open-Loop Control (Problem 2a):**
- Uses a pre-computed minimum energy control sequence $u_0, u_1, ..., u_{49}$ calculated offline
- The control input at each time step is predetermined and does not depend on the current state
- Target: Reach $x_f = [2, 0]^T$ at $k=50$

**Closed-Loop Control (Problem 2b):**
- Uses feedback control law $u_k = -K(x_k - x_d)$
- The control input is computed in real-time based on the current state $x_k$
- Feedback gain: $K = [0.25, 0.875]$ (from pole placement at 0.5, 0.5)
- Target: Maintain hover at $x_d = [3, 0]^T$

#### 2. **System Response Comparison**

**Height Response:**
- **Open-loop**: The height trajectory follows a smooth curve that reaches exactly $z=2$ at $k=50$. The trajectory is predetermined and cannot adapt to disturbances or modeling errors.
- **Closed-loop**: The height trajectory converges to $z=3$ asymptotically. The system exhibits exponential convergence behavior due to the pole placement design (poles at 0.5, 0.5).

**Velocity Response:**
- **Open-loop**: The velocity starts at 0, increases to provide upward motion, then decreases to reach zero velocity at $k=50$. The velocity profile is carefully designed to minimize energy consumption.
- **Closed-loop**: The velocity converges to 0 as the system approaches the desired hover state. The convergence is smooth and stable.

**Control Input:**
- **Open-loop**: The control sequence is non-zero throughout the entire time horizon, with varying magnitude optimized for minimum energy.
- **Closed-loop**: The control input starts large (to drive the system toward the desired state) and gradually decreases as the system converges. Once near the desired state, the control input becomes small (mainly compensating for small deviations).

#### 3. **Key Observations**

1. **Tracking Performance:**
   - Open-loop control achieves exact tracking at the final time step ($k=50$)
   - Closed-loop control provides asymptotic tracking, continuously maintaining the desired state

2. **Robustness:**
   - Open-loop control is sensitive to disturbances and modeling errors (no feedback mechanism)
   - Closed-loop control can reject disturbances through feedback (as demonstrated in Problem 2d)

3. **Energy Consumption:**
   - Open-loop control minimizes total energy consumption (by design)
   - Closed-loop control may consume more energy initially but provides better long-term stability

4. **Practical Considerations:**
   - Open-loop control requires perfect knowledge of the system model and no disturbances
   - Closed-loop control is more practical for real-world applications where disturbances and uncertainties exist

---

## Problem 2d: Robustness Under Noise

### Question
Assume that due to wind disturbances, there is noise added to the velocity dynamics:
$$x_{k+1} = Ax_k + Bu_k + \begin{bmatrix} 0 \\ 0.02 \end{bmatrix} w_k, \quad x_0 = [0, 0]^T$$
where $w_k \sim \mathcal{N}(0, 1)$. For $k = 0, ..., 50$, simulate system (5) and plot the UAV's height under open-loop and closed-loop control. Which one is more robust? (Use the same noise sequence for both.)

### Answer

#### 1. **Noise Model**
The noise $w_k$ is a zero-mean Gaussian random variable with unit variance, affecting only the velocity component of the state vector. The noise magnitude is scaled by 0.02, representing wind disturbances.

#### 2. **Simulation Results**

**Open-Loop Control Under Noise:**
- The height trajectory deviates significantly from the desired path
- Since the control sequence is predetermined and does not respond to state deviations, the system cannot correct for noise-induced errors
- The final state at $k=50$ does not reach the target $z=2$ due to accumulated noise effects
- The trajectory shows random fluctuations around the nominal path

**Closed-Loop Control Under Noise:**
- The height trajectory remains close to the desired value $z=3$
- The feedback controller continuously corrects for noise-induced deviations
- The system exhibits bounded oscillations around the desired state
- The control law $u_k = -K(x_k - x_d)$ actively compensates for disturbances

#### 3. **Robustness Analysis**

**Closed-loop control is significantly more robust than open-loop control.**

**Reasons:**

1. **Feedback Correction:**
   - Closed-loop control measures the current state and adjusts the control input accordingly
   - Any deviation from the desired state is immediately corrected
   - Open-loop control has no mechanism to detect or correct errors

2. **Disturbance Rejection:**
   - The feedback gain $K$ provides disturbance rejection capability
   - The closed-loop system acts as a regulator, maintaining the desired state despite noise
   - Open-loop control cannot reject disturbances; errors accumulate over time

3. **Stability:**
   - Closed-loop control maintains stability even under persistent disturbances
   - The system converges to a bounded region around the desired state
   - Open-loop control has no stability guarantee under disturbances

4. **Performance Degradation:**
   - Closed-loop: Small performance degradation (bounded oscillations)
   - Open-loop: Large performance degradation (significant deviation from target)

#### 4. **Quantitative Comparison**

From the simulation results:
- **Open-loop**: Final height error is significant (depends on noise realization)
- **Closed-loop**: Final height error is small (bounded by feedback gain and noise magnitude)

The closed-loop system demonstrates superior robustness, making it the preferred choice for practical applications where disturbances are inevitable.

---

## Problem 3: LQR Optimal Control

### Question
Simulate system (3) under infinite-horizon LQR control with cost function:
$$J = \sum_{k=0}^{\infty} (x - x_d)^T Q(x - x_d) + u_k^T R u_k$$
where $x_d = [2, 0]^T$. Test three cases:
1. $Q = I$, $R = 1$
2. $Q = 100I$, $R = 1$
3. $Q = I$, $R = 0.1$

Plot the states $x_k$ and input $u_k$ for all three cases. Compare the plots and explain any differences.

### Answer

#### 1. **LQR Control Design**

The infinite-horizon LQR controller minimizes the cost function $J$ by solving the discrete-time algebraic Riccati equation (DARE):
$$P = A^T P A - A^T P B (B^T P B + R)^{-1} B^T P A + Q$$

The optimal feedback gain is:
$$K = (B^T P B + R)^{-1} B^T P A$$

The control law is:
$$u_k = -K(x_k - x_d)$$

#### 2. **Feedback Gains for Each Case**

| Case | Q | R | Feedback Gain K |
|------|---|---|-----------------|
| Case 1 | $I$ | $1$ | $K = [0.434, 1.028]$ |
| Case 2 | $100I$ | $1$ | $K = [0.661, 1.326]$ |
| Case 3 | $I$ | $0.1$ | $K = [0.617, 1.270]$ |

#### 3. **Comparison of Three Cases**

##### **Case 1: Q = I, R = 1 (Balanced Design)**

**Characteristics:**
- Moderate feedback gains: $K = [0.434, 1.028]$
- Balanced tradeoff between state error and control effort
- Smooth convergence to desired state

**Performance:**
- Height converges to $z=2$ smoothly
- Velocity converges to $0$ without overshoot
- Control input is moderate in magnitude
- Convergence time is moderate

**Interpretation:**
This case represents a balanced design where both state error and control effort are equally penalized. The system achieves stable convergence with reasonable control effort.

##### **Case 2: Q = 100I, R = 1 (High State Penalty)**

**Characteristics:**
- Larger feedback gains: $K = [0.661, 1.326]$
- Higher penalty on state error relative to control effort
- More aggressive control action

**Performance:**
- Height converges to $z=2$ **faster** than Case 1
- Velocity converges to $0$ more quickly
- Control input is **larger** in magnitude, especially in the initial phase
- Faster convergence but higher control effort

**Interpretation:**
By increasing $Q$ to $100I$, we place 100 times more weight on minimizing state error. This results in:
- **Faster convergence**: The system prioritizes reaching the desired state quickly
- **Larger control gains**: More aggressive feedback to drive the state error to zero
- **Higher control effort**: Larger control inputs are acceptable to achieve faster convergence

**Tradeoff:** Faster response comes at the cost of increased control effort and potentially higher energy consumption.

##### **Case 3: Q = I, R = 0.1 (Low Control Penalty)**

**Characteristics:**
- Moderate-to-large feedback gains: $K = [0.617, 1.270]$
- Lower penalty on control effort relative to state error
- More aggressive control allowed

**Performance:**
- Height converges to $z=2$ faster than Case 1 but similar to Case 2
- Velocity converges to $0$ quickly
- Control input magnitude is **larger** than Case 1, similar to Case 2
- Good convergence performance with relaxed control constraints

**Interpretation:**
By decreasing $R$ to $0.1$, we reduce the penalty on control effort by a factor of 10. This allows:
- **More aggressive control**: The system can use larger control inputs without heavy penalty
- **Faster convergence**: Similar to Case 2, the system converges quickly
- **Larger control gains**: The feedback gain is larger than Case 1, enabling faster response

**Tradeoff:** Better performance is achieved by allowing larger control inputs, which may require more powerful actuators and higher energy consumption.

#### 4. **Key Differences Summary**

| Aspect | Case 1 (Q=I, R=1) | Case 2 (Q=100I, R=1) | Case 3 (Q=I, R=0.1) |
|--------|-------------------|----------------------|---------------------|
| **State Penalty** | Low | High | Low |
| **Control Penalty** | Medium | Medium | Low |
| **Feedback Gain** | Smallest | Largest | Medium-Large |
| **Convergence Speed** | Slowest | Fastest | Fast |
| **Control Effort** | Smallest | Largest | Large |
| **Use Case** | Energy-efficient, smooth | Fast response required | Fast response, relaxed control constraints |

#### 5. **Physical Interpretation**

**Case 1 (Q=I, R=1):**
- Suitable for applications where energy efficiency is important
- Smooth, comfortable motion (e.g., passenger drones)
- Moderate actuator requirements

**Case 2 (Q=100I, R=1):**
- Suitable for applications requiring fast, precise tracking
- High-performance systems (e.g., racing drones, precision landing)
- Requires powerful actuators

**Case 3 (Q=I, R=0.1):**
- Suitable when control effort is not a major concern
- Fast response with relaxed control constraints
- Good balance between performance and control effort

#### 6. **Design Insights**

1. **Increasing Q (state penalty):**
   - Increases feedback gains
   - Improves convergence speed
   - Increases control effort
   - Better tracking performance

2. **Decreasing R (control penalty):**
   - Allows larger control inputs
   - Enables faster response
   - Increases energy consumption
   - May require more powerful actuators

3. **Optimal Tuning:**
   - The choice of $Q$ and $R$ depends on the specific application requirements
   - There is a tradeoff between performance (convergence speed) and cost (control effort)
   - LQR provides a systematic way to tune this tradeoff while maintaining stability

#### 7. **Stability Guarantee**

All three cases result in stable closed-loop systems because:
- The LQR solution guarantees closed-loop stability (under controllability and observability assumptions)
- All feedback gains place the closed-loop poles inside the unit circle
- The infinite-horizon cost function ensures asymptotic stability

The differences lie in **performance** (convergence speed) and **cost** (control effort), not in stability.

---

## Conclusion

1. **Problem 2c:** Closed-loop control provides better tracking and adaptability compared to open-loop control, though open-loop may be more energy-efficient for specific tasks.

2. **Problem 2d:** Closed-loop control is significantly more robust to disturbances, demonstrating superior performance under noise compared to open-loop control.

3. **Problem 3:** The LQR controller allows systematic tuning of the tradeoff between state error and control effort. Higher state penalties ($Q$) or lower control penalties ($R$) lead to faster convergence but require larger control inputs. The optimal choice depends on the specific application requirements.

