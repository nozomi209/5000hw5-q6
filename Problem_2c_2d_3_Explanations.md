# Problem 2c, 2d, and 3: Detailed Explanations / 第2c、2d和3题详细解释

## Problem 2c: Open-Loop vs Closed-Loop Control Comparison / 第2c题：开环与闭环控制对比

### Question / 问题
For k = 0, ..., 50, simulate system (3) under the open-loop controller (2a) and the closed-loop controller (2b). Plot the states evolution under each control method on the same plot (create one plot for height and one for velocity). Compare the two system responses.

对于 k = 0, ..., 50，在开环控制器(2a)和闭环控制器(2b)下仿真系统(3)。在同一图上绘制每种控制方法下的状态演化（创建高度图和速度图各一个）。比较两种系统响应。

### Answer / 答案

#### 1. **Control Strategy Differences / 控制策略差异**

**Open-Loop Control (Problem 2a) / 开环控制（第2a题）：**
- Uses a pre-computed minimum energy control sequence $u_0, u_1, ..., u_{49}$ calculated offline
- 使用离线计算的预计算最小能量控制序列 $u_0, u_1, ..., u_{49}$
- The control input at each time step is predetermined and does not depend on the current state
- 每个时间步的控制输入是预先确定的，不依赖于当前状态
- Target: Reach $x_f = [2, 0]^T$ at $k=50$
- 目标：在 $k=50$ 时达到 $x_f = [2, 0]^T$

**Closed-Loop Control (Problem 2b) / 闭环控制（第2b题）：**
- Uses feedback control law $u_k = -K(x_k - x_d)$
- 使用反馈控制律 $u_k = -K(x_k - x_d)$
- The control input is computed in real-time based on the current state $x_k$
- 控制输入基于当前状态 $x_k$ 实时计算
- Feedback gain: $K = [0.25, 0.875]$ (from pole placement at 0.5, 0.5)
- 反馈增益：$K = [0.25, 0.875]$（通过极点配置在0.5, 0.5处得到）
- Target: Maintain hover at $x_d = [3, 0]^T$
- 目标：维持在 $x_d = [3, 0]^T$ 处悬停

#### 2. **System Response Comparison / 系统响应对比**

**Height Response / 高度响应：**
- **Open-loop / 开环**: The height trajectory follows a smooth curve that reaches exactly $z=2$ at $k=50$. The trajectory is predetermined and cannot adapt to disturbances or modeling errors.
- **开环**: 高度轨迹遵循一条平滑曲线，在 $k=50$ 时精确达到 $z=2$。轨迹是预先确定的，无法适应扰动或建模误差。
- **Closed-loop / 闭环**: The height trajectory converges to $z=3$ asymptotically. The system exhibits exponential convergence behavior due to the pole placement design (poles at 0.5, 0.5).
- **闭环**: 高度轨迹渐近收敛到 $z=3$。由于极点配置设计（极点在0.5, 0.5），系统表现出指数收敛行为。

**Velocity Response / 速度响应：**
- **Open-loop / 开环**: The velocity starts at 0, increases to provide upward motion, then decreases to reach zero velocity at $k=50$. The velocity profile is carefully designed to minimize energy consumption.
- **开环**: 速度从0开始，增加以提供向上运动，然后减小以在 $k=50$ 时达到零速度。速度曲线经过精心设计以最小化能耗。
- **Closed-loop / 闭环**: The velocity converges to 0 as the system approaches the desired hover state. The convergence is smooth and stable.
- **闭环**: 当系统接近期望悬停状态时，速度收敛到0。收敛过程平滑且稳定。

**Control Input / 控制输入：**
- **Open-loop / 开环**: The control sequence is non-zero throughout the entire time horizon, with varying magnitude optimized for minimum energy.
- **开环**: 控制序列在整个时间范围内非零，幅度变化以优化最小能量。
- **Closed-loop / 闭环**: The control input starts large (to drive the system toward the desired state) and gradually decreases as the system converges. Once near the desired state, the control input becomes small (mainly compensating for small deviations).
- **闭环**: 控制输入开始时较大（以驱动系统朝向期望状态），随着系统收敛而逐渐减小。一旦接近期望状态，控制输入变得很小（主要补偿小偏差）。

#### 3. **Key Observations / 关键观察**

1. **Tracking Performance / 跟踪性能：**
   - Open-loop control achieves exact tracking at the final time step ($k=50$)
   - 开环控制在最终时间步（$k=50$）实现精确跟踪
   - Closed-loop control provides asymptotic tracking, continuously maintaining the desired state
   - 闭环控制提供渐近跟踪，持续维持期望状态

2. **Robustness / 鲁棒性：**
   - Open-loop control is sensitive to disturbances and modeling errors (no feedback mechanism)
   - 开环控制对扰动和建模误差敏感（无反馈机制）
   - Closed-loop control can reject disturbances through feedback (as demonstrated in Problem 2d)
   - 闭环控制可以通过反馈抑制扰动（如第2d题所示）

3. **Energy Consumption / 能耗：**
   - Open-loop control minimizes total energy consumption (by design)
   - 开环控制最小化总能耗（通过设计）
   - Closed-loop control may consume more energy initially but provides better long-term stability
   - 闭环控制初始可能消耗更多能量，但提供更好的长期稳定性

4. **Practical Considerations / 实际考虑：**
   - Open-loop control requires perfect knowledge of the system model and no disturbances
   - 开环控制需要完美的系统模型知识且无扰动
   - Closed-loop control is more practical for real-world applications where disturbances and uncertainties exist
   - 闭环控制对于存在扰动和不确定性的实际应用更加实用

---

## Problem 2d: Robustness Under Noise / 第2d题：噪声下的鲁棒性

### Question / 问题
Assume that due to wind disturbances, there is noise added to the velocity dynamics:
假设由于风扰动，速度动力学中添加了噪声：
$$x_{k+1} = Ax_k + Bu_k + \begin{bmatrix} 0 \\ 0.02 \end{bmatrix} w_k, \quad x_0 = [0, 0]^T$$
where $w_k \sim \mathcal{N}(0, 1)$. For $k = 0, ..., 50$, simulate system (5) and plot the UAV's height under open-loop and closed-loop control. Which one is more robust? (Use the same noise sequence for both.)
其中 $w_k \sim \mathcal{N}(0, 1)$。对于 $k = 0, ..., 50$，仿真系统(5)并绘制开环和闭环控制下的无人机高度。哪个更鲁棒？（对两者使用相同的噪声序列。）

### Answer / 答案

#### 1. **Noise Model / 噪声模型**
The noise $w_k$ is a zero-mean Gaussian random variable with unit variance, affecting only the velocity component of the state vector. The noise magnitude is scaled by 0.02, representing wind disturbances.
噪声 $w_k$ 是零均值、单位方差的高斯随机变量，仅影响状态向量的速度分量。噪声幅度按0.02缩放，表示风扰动。

#### 2. **Simulation Results / 仿真结果**

**Open-Loop Control Under Noise / 噪声下的开环控制：**
- The height trajectory deviates significantly from the desired path
- 高度轨迹显著偏离期望路径
- Since the control sequence is predetermined and does not respond to state deviations, the system cannot correct for noise-induced errors
- 由于控制序列是预先确定的且不响应状态偏差，系统无法纠正噪声引起的误差
- The final state at $k=50$ does not reach the target $z=2$ due to accumulated noise effects
- 由于累积的噪声效应，$k=50$ 时的最终状态未达到目标 $z=2$
- The trajectory shows random fluctuations around the nominal path
- 轨迹在标称路径周围显示随机波动

**Closed-Loop Control Under Noise / 噪声下的闭环控制：**
- The height trajectory remains close to the desired value $z=3$
- 高度轨迹保持接近期望值 $z=3$
- The feedback controller continuously corrects for noise-induced deviations
- 反馈控制器持续纠正噪声引起的偏差
- The system exhibits bounded oscillations around the desired state
- 系统在期望状态周围表现出有界振荡
- The control law $u_k = -K(x_k - x_d)$ actively compensates for disturbances
- 控制律 $u_k = -K(x_k - x_d)$ 主动补偿扰动

#### 3. **Robustness Analysis / 鲁棒性分析**

**Closed-loop control is significantly more robust than open-loop control.**
**闭环控制明显比开环控制更鲁棒。**

**Reasons / 原因：**

1. **Feedback Correction / 反馈校正：**
   - Closed-loop control measures the current state and adjusts the control input accordingly
   - 闭环控制测量当前状态并相应调整控制输入
   - Any deviation from the desired state is immediately corrected
   - 任何偏离期望状态的情况都会立即被纠正
   - Open-loop control has no mechanism to detect or correct errors
   - 开环控制没有检测或纠正误差的机制

2. **Disturbance Rejection / 扰动抑制：**
   - The feedback gain $K$ provides disturbance rejection capability
   - 反馈增益 $K$ 提供扰动抑制能力
   - The closed-loop system acts as a regulator, maintaining the desired state despite noise
   - 闭环系统充当调节器，尽管存在噪声仍维持期望状态
   - Open-loop control cannot reject disturbances; errors accumulate over time
   - 开环控制无法抑制扰动；误差随时间累积

3. **Stability / 稳定性：**
   - Closed-loop control maintains stability even under persistent disturbances
   - 即使在持续扰动下，闭环控制仍保持稳定性
   - The system converges to a bounded region around the desired state
   - 系统收敛到期望状态周围的有界区域
   - Open-loop control has no stability guarantee under disturbances
   - 开环控制在扰动下没有稳定性保证

4. **Performance Degradation / 性能退化：**
   - Closed-loop: Small performance degradation (bounded oscillations)
   - 闭环：小的性能退化（有界振荡）
   - Open-loop: Large performance degradation (significant deviation from target)
   - 开环：大的性能退化（显著偏离目标）

#### 4. **Quantitative Comparison / 定量对比**

From the simulation results:
从仿真结果来看：
- **Open-loop / 开环**: Final height error is significant (depends on noise realization)
- **开环**: 最终高度误差显著（取决于噪声实现）
- **Closed-loop / 闭环**: Final height error is small (bounded by feedback gain and noise magnitude)
- **闭环**: 最终高度误差小（受反馈增益和噪声幅度限制）

The closed-loop system demonstrates superior robustness, making it the preferred choice for practical applications where disturbances are inevitable.
闭环系统表现出卓越的鲁棒性，使其成为扰动不可避免的实际应用的首选。

---

## Problem 3: LQR Optimal Control / 第3题：LQR最优控制

### Question / 问题
Simulate system (3) under infinite-horizon LQR control with cost function:
在无限时域LQR控制下仿真系统(3)，成本函数为：
$$J = \sum_{k=0}^{\infty} (x - x_d)^T Q(x - x_d) + u_k^T R u_k$$
where $x_d = [2, 0]^T$. Test three cases:
其中 $x_d = [2, 0]^T$。测试三种情况：
1. $Q = I$, $R = 1$
2. $Q = 100I$, $R = 1$
3. $Q = I$, $R = 0.1$

Plot the states $x_k$ and input $u_k$ for all three cases. Compare the plots and explain any differences.
绘制所有三种情况下的状态 $x_k$ 和输入 $u_k$。比较图表并解释任何差异。

### Answer / 答案

#### 1. **LQR Control Design / LQR控制设计**

The infinite-horizon LQR controller minimizes the cost function $J$ by solving the discrete-time algebraic Riccati equation (DARE):
无限时域LQR控制器通过求解离散时间代数Riccati方程（DARE）来最小化成本函数 $J$：
$$P = A^T P A - A^T P B (B^T P B + R)^{-1} B^T P A + Q$$

The optimal feedback gain is:
最优反馈增益为：
$$K = (B^T P B + R)^{-1} B^T P A$$

The control law is:
控制律为：
$$u_k = -K(x_k - x_d)$$

#### 2. **Feedback Gains for Each Case / 每种情况的反馈增益**

| Case / 情况 | Q | R | Feedback Gain K / 反馈增益 K |
|------|---|---|-----------------|
| Case 1 / 情况1 | $I$ | $1$ | $K = [0.434, 1.028]$ |
| Case 2 / 情况2 | $100I$ | $1$ | $K = [0.661, 1.326]$ |
| Case 3 / 情况3 | $I$ | $0.1$ | $K = [0.617, 1.270]$ |

#### 3. **Comparison of Three Cases / 三种情况对比**

##### **Case 1: Q = I, R = 1 (Balanced Design / 平衡设计)**

**Characteristics / 特征：**
- Moderate feedback gains: $K = [0.434, 1.028]$
- 中等反馈增益：$K = [0.434, 1.028]$
- Balanced tradeoff between state error and control effort
- 状态误差和控制努力之间的平衡权衡
- Smooth convergence to desired state
- 平滑收敛到期望状态

**Performance / 性能：**
- Height converges to $z=2$ smoothly
- 高度平滑收敛到 $z=2$
- Velocity converges to $0$ without overshoot
- 速度收敛到 $0$，无超调
- Control input is moderate in magnitude
- 控制输入幅度适中
- Convergence time is moderate
- 收敛时间适中

**Interpretation / 解释：**
This case represents a balanced design where both state error and control effort are equally penalized. The system achieves stable convergence with reasonable control effort.
这种情况代表平衡设计，状态误差和控制努力受到同等惩罚。系统以合理的控制努力实现稳定收敛。

##### **Case 2: Q = 100I, R = 1 (High State Penalty / 高状态惩罚)**

**Characteristics / 特征：**
- Larger feedback gains: $K = [0.661, 1.326]$
- 更大的反馈增益：$K = [0.661, 1.326]$
- Higher penalty on state error relative to control effort
- 相对于控制努力，对状态误差的惩罚更高
- More aggressive control action
- 更激进的控制动作

**Performance / 性能：**
- Height converges to $z=2$ **faster** than Case 1
- 高度收敛到 $z=2$ **更快**于情况1
- Velocity converges to $0$ more quickly
- 速度更快收敛到 $0$
- Control input is **larger** in magnitude, especially in the initial phase
- 控制输入幅度**更大**，特别是在初始阶段
- Faster convergence but higher control effort
- 收敛更快但控制努力更高

**Interpretation / 解释：**
By increasing $Q$ to $100I$, we place 100 times more weight on minimizing state error. This results in:
通过将 $Q$ 增加到 $100I$，我们在最小化状态误差上放置了100倍的权重。这导致：
- **Faster convergence / 更快收敛**: The system prioritizes reaching the desired state quickly
- **更快收敛**: 系统优先快速到达期望状态
- **Larger control gains / 更大的控制增益**: More aggressive feedback to drive the state error to zero
- **更大的控制增益**: 更激进的反馈以将状态误差驱动到零
- **Higher control effort / 更高的控制努力**: Larger control inputs are acceptable to achieve faster convergence
- **更高的控制努力**: 更大的控制输入是可接受的，以实现更快收敛

**Tradeoff / 权衡:** Faster response comes at the cost of increased control effort and potentially higher energy consumption.
**权衡:** 更快的响应以增加控制努力和可能更高的能耗为代价。

##### **Case 3: Q = I, R = 0.1 (Low Control Penalty / 低控制惩罚)**

**Characteristics / 特征：**
- Moderate-to-large feedback gains: $K = [0.617, 1.270]$
- 中等到大的反馈增益：$K = [0.617, 1.270]$
- Lower penalty on control effort relative to state error
- 相对于状态误差，对控制努力的惩罚更低
- More aggressive control allowed
- 允许更激进的控制

**Performance / 性能：**
- Height converges to $z=2$ faster than Case 1 but similar to Case 2
- 高度收敛到 $z=2$ 比情况1快，但与情况2相似
- Velocity converges to $0$ quickly
- 速度快速收敛到 $0$
- Control input magnitude is **larger** than Case 1, similar to Case 2
- 控制输入幅度**大于**情况1，与情况2相似
- Good convergence performance with relaxed control constraints
- 在放宽控制约束下具有良好的收敛性能

**Interpretation / 解释：**
By decreasing $R$ to $0.1$, we reduce the penalty on control effort by a factor of 10. This allows:
通过将 $R$ 降低到 $0.1$，我们将控制努力的惩罚降低了10倍。这允许：
- **More aggressive control / 更激进的控制**: The system can use larger control inputs without heavy penalty
- **更激进的控制**: 系统可以使用更大的控制输入而不会受到严重惩罚
- **Faster convergence / 更快收敛**: Similar to Case 2, the system converges quickly
- **更快收敛**: 与情况2相似，系统快速收敛
- **Larger control gains / 更大的控制增益**: The feedback gain is larger than Case 1, enabling faster response
- **更大的控制增益**: 反馈增益大于情况1，实现更快响应

**Tradeoff / 权衡:** Better performance is achieved by allowing larger control inputs, which may require more powerful actuators and higher energy consumption.
**权衡:** 通过允许更大的控制输入实现更好的性能，这可能需要更强大的执行器和更高的能耗。

#### 4. **Key Differences Summary / 关键差异总结**

| Aspect / 方面 | Case 1 (Q=I, R=1) | Case 2 (Q=100I, R=1) | Case 3 (Q=I, R=0.1) |
|--------|-------------------|----------------------|---------------------|
| **State Penalty / 状态惩罚** | Low / 低 | High / 高 | Low / 低 |
| **Control Penalty / 控制惩罚** | Medium / 中等 | Medium / 中等 | Low / 低 |
| **Feedback Gain / 反馈增益** | Smallest / 最小 | Largest / 最大 | Medium-Large / 中-大 |
| **Convergence Speed / 收敛速度** | Slowest / 最慢 | Fastest / 最快 | Fast / 快 |
| **Control Effort / 控制努力** | Smallest / 最小 | Largest / 最大 | Large / 大 |
| **Use Case / 使用场景** | Energy-efficient, smooth / 节能、平滑 | Fast response required / 需要快速响应 | Fast response, relaxed control constraints / 快速响应，放宽控制约束 |

#### 5. **Physical Interpretation / 物理解释**

**Case 1 (Q=I, R=1) / 情况1：**
- Suitable for applications where energy efficiency is important
- 适用于能源效率重要的应用
- Smooth, comfortable motion (e.g., passenger drones)
- 平滑、舒适的运动（例如，载客无人机）
- Moderate actuator requirements
- 中等执行器要求

**Case 2 (Q=100I, R=1) / 情况2：**
- Suitable for applications requiring fast, precise tracking
- 适用于需要快速、精确跟踪的应用
- High-performance systems (e.g., racing drones, precision landing)
- 高性能系统（例如，竞速无人机、精确着陆）
- Requires powerful actuators
- 需要强大的执行器

**Case 3 (Q=I, R=0.1) / 情况3：**
- Suitable when control effort is not a major concern
- 适用于控制努力不是主要关注点的情况
- Fast response with relaxed control constraints
- 在放宽控制约束下快速响应
- Good balance between performance and control effort
- 性能和控制努力之间的良好平衡

#### 6. **Design Insights / 设计洞察**

1. **Increasing Q (state penalty) / 增加Q（状态惩罚）：**
   - Increases feedback gains
   - 增加反馈增益
   - Improves convergence speed
   - 提高收敛速度
   - Increases control effort
   - 增加控制努力
   - Better tracking performance
   - 更好的跟踪性能

2. **Decreasing R (control penalty) / 降低R（控制惩罚）：**
   - Allows larger control inputs
   - 允许更大的控制输入
   - Enables faster response
   - 实现更快响应
   - Increases energy consumption
   - 增加能耗
   - May require more powerful actuators
   - 可能需要更强大的执行器

3. **Optimal Tuning / 最优调参：**
   - The choice of $Q$ and $R$ depends on the specific application requirements
   - $Q$ 和 $R$ 的选择取决于特定的应用需求
   - There is a tradeoff between performance (convergence speed) and cost (control effort)
   - 性能（收敛速度）和成本（控制努力）之间存在权衡
   - LQR provides a systematic way to tune this tradeoff while maintaining stability
   - LQR提供了一种在保持稳定性的同时调整此权衡的系统方法

#### 7. **Stability Guarantee / 稳定性保证**

All three cases result in stable closed-loop systems because:
所有三种情况都导致稳定的闭环系统，因为：
- The LQR solution guarantees closed-loop stability (under controllability and observability assumptions)
- LQR解保证闭环稳定性（在可控性和可观测性假设下）
- All feedback gains place the closed-loop poles inside the unit circle
- 所有反馈增益都将闭环极点放置在单位圆内
- The infinite-horizon cost function ensures asymptotic stability
- 无限时域成本函数确保渐近稳定性

The differences lie in **performance** (convergence speed) and **cost** (control effort), not in stability.
差异在于**性能**（收敛速度）和**成本**（控制努力），而不在于稳定性。

---

## Conclusion / 结论

1. **Problem 2c / 第2c题:** Closed-loop control provides better tracking and adaptability compared to open-loop control, though open-loop may be more energy-efficient for specific tasks.
   **第2c题:** 闭环控制相比开环控制提供更好的跟踪和适应性，尽管开环对于特定任务可能更节能。

2. **Problem 2d / 第2d题:** Closed-loop control is significantly more robust to disturbances, demonstrating superior performance under noise compared to open-loop control.
   **第2d题:** 闭环控制对扰动明显更鲁棒，在噪声下表现出优于开环控制的性能。

3. **Problem 3 / 第3题:** The LQR controller allows systematic tuning of the tradeoff between state error and control effort. Higher state penalties ($Q$) or lower control penalties ($R$) lead to faster convergence but require larger control inputs. The optimal choice depends on the specific application requirements.
   **第3题:** LQR控制器允许系统调整状态误差和控制努力之间的权衡。更高的状态惩罚（$Q$）或更低的控制惩罚（$R$）导致更快收敛但需要更大的控制输入。最优选择取决于特定的应用需求。
