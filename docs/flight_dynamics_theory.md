# Flight Dynamics, C* Control Law Derivation, and AFDX Network Calculus

## 1. Longitudinal Equations of Motion

The small-perturbation longitudinal equations of motion for a symmetric rigid aircraft referenced to stability axes are formulated in state-space representation:

$$\begin{bmatrix} \dot{u} \\ \dot{w} \\ \dot{q} \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} X_u & X_w & 0 & -g_0 \cos \theta_0 \\ Z_u & Z_w & u_0 & -g_0 \sin \theta_0 \\ M_u & M_w + M_{\dot{w}} Z_w & M_q + M_{\dot{w}} u_0 & 0 \\ 0 & 0 & 1 & 0 \end{bmatrix} \begin{bmatrix} u \\ w \\ q \\ \theta \end{bmatrix} + \begin{bmatrix} X_{\delta e} \\ Z_{\delta e} \\ M_{\delta e} + M_{\dot{w}} Z_{\delta e} \\ 0 \end{bmatrix} \delta_e$$

where:
- $u, w$ are longitudinal and vertical perturbation velocities in body coordinates.
- $q = \dot{\theta}$ is the body pitch rate in radians per second.
- $\theta$ is the pitch attitude angle.
- $X_i, Z_i, M_i$ are normalized dimensional aerodynamic stability derivatives.

Under the short-period approximation, forward velocity perturbations are decoupled ($\dot{u} \approx 0$), reducing the state vector to angle of attack perturbation $\alpha = w / u_0$ and pitch rate $q$:

$$\begin{bmatrix} \dot{\alpha} \\ \dot{q} \end{bmatrix} = \begin{bmatrix} \frac{Z_\alpha}{u_0} & 1 \\ M_\alpha & M_q \end{bmatrix} \begin{bmatrix} \alpha \\ q \end{bmatrix} + \begin{bmatrix} \frac{Z_{\delta e}}{u_0} \\ M_{\delta e} \end{bmatrix} \delta_e$$

---

## 2. Derivation of the C* Flight Control Law

Modern civil transport aircraft operate across a wide flight regime, from low-speed flare ($V_{\text{CAS}} \approx 130\,\text{knots}$) during landing to high-speed cruise ($M \approx 0.82$, $V_{\text{CAS}} \approx 320\,\text{knots}$) at high altitude. 

Pilot handling qualities (Cooper-Harper Level 1 rating) require:
1. Crisp, immediate rotational response ($q$ demand) at low airspeed to ensure accurate flare trajectory and pitch touchdown precision.
2. Direct load factor response ($N_z$ command) at high airspeed to maintain constant flight path angle $\gamma$ in cruise and reject atmospheric turbulence without pilot intervention.

To unify these conflicting requirements into a single continuous handling quality parameter, the **C* criteria** blends normal acceleration increment $\Delta N_z$ and pitch rate $q$:

$$C^*_{\text{feedback}} = (N_z - 1.0) + \frac{V_{\text{co}}}{g_0} q$$

where:
- $N_z = -\frac{a_z}{g_0}$ is normal load factor in $g$ units ($1.0g$ in steady level flight).
- $g_0 = 9.80665\,\text{m/s}^2$ is standard gravitational acceleration.
- $V_{\text{co}}$ is the crossover airspeed, chosen such that pitch rate and normal acceleration contribute equally to $C^*$ at $V = V_{\text{co}}$:

$$V_{\text{co}} \approx 210\,\text{knots} \approx 108\,\text{m/s}$$

### Low-Speed vs High-Speed Asymptotic Behavior

The kinematic relationship between normal acceleration and pitch rate in steady flight is:

$$\Delta N_z = \frac{V_{\text{TAS}}}{g_0} \dot{\gamma} = \frac{V_{\text{TAS}}}{g_0} (q - \dot{\alpha})$$

Substituting this into the $C^*$ formulation yields:

$$C^* = \frac{V_{\text{TAS}}}{g_0} (q - \dot{\alpha}) + \frac{V_{\text{co}}}{g_0} q = \frac{V_{\text{TAS}} + V_{\text{co}}}{g_0} q - \frac{V_{\text{TAS}}}{g_0} \dot{\alpha}$$

- **At low speed ($V_{\text{TAS}} \ll V_{\text{co}}$)**:
  $$\frac{V_{\text{co}}}{g_0} q \gg \Delta N_z \implies C^* \approx \frac{V_{\text{co}}}{g_0} q$$
  The controller functions purely as a pitch-rate command system ($q$-demand).
- **At high speed ($V_{\text{TAS}} \gg V_{\text{co}}$)**:
  $$\Delta N_z \gg \frac{V_{\text{co}}}{g_0} q \implies C^* \approx \Delta N_z$$
  The controller functions purely as an acceleration command system ($g$-demand).

---

## 3. Compressible Saint-Venant Aerodynamics

Air data sensors measure impact pressure $q_c = P_t - P_s$. For subsonic compressible flow ($\text{Mach} < 1.0$), the isentropic relationship between total pressure $P_t$, static pressure $P_s$, and Mach number $M$ is defined by the Saint-Venant compressible flow equation:

$$\frac{P_t}{P_s} = \left( 1 + \frac{\gamma - 1}{2} M^2 \right)^{\frac{\gamma}{\gamma - 1}}$$

With ratio of specific heats $\gamma = 1.4$, the exponent evaluates to $\frac{1.4}{0.4} = \frac{7}{2} = 3.5$:

$$\frac{P_t}{P_s} = \left( 1 + 0.2 M^2 \right)^{3.5}$$

Solving explicitly for Mach number $M$:

$$M = \sqrt{5 \left[ \left(\frac{P_t}{P_s}\right)^{\frac{2}{7}} - 1 \right]}$$

Calibrated Airspeed ($V_{\text{CAS}}$) is defined as the speed which, at standard sea level conditions ($P_0 = 101325\,\text{Pa}$, $\rho_0 = 1.225\,\text{kg/m}^3$, $a_0 = 340.294\,\text{m/s}$), produces the identical impact pressure $q_c$:

$$V_{\text{CAS}} = a_0 \sqrt{5 \left[ \left(\frac{q_c}{P_0} + 1\right)^{\frac{2}{7}} - 1 \right]}$$

---

## 4. ARINC 664 Network Calculus & Determinism Proof

Deterministic execution over switched Ethernet requires formal bounds on packet queuing delay and buffer backlogs using Network Calculus.

### Traffic Arrival Curve

An AFDX Virtual Link $i$ is constrained by a token bucket arrival curve $\alpha_i(t)$:

$$\alpha_i(t) = L_{\max, i} + \rho_i t$$

where:
- $L_{\max, i}$ is the maximum frame length in bytes ($64 \le L_{\max} \le 1518$).
- $\rho_i = \frac{L_{\max, i}}{\text{BAG}_i}$ is the allocated peak transmission rate.
- $\text{BAG}_i \in \{1, 2, 4, 8, 16, 32, 64, 128\}\,\text{ms}$ is the Bandwidth Allocation Gap.

### Switch Service Curve

An AFDX switch with output port rate $C = 100\,\text{Mbps}$ provides a rate-latency service curve $\beta(t)$:

$$\beta(t) = C (t - T)^+$$

where latency $T$ accounts for internal crossbar transit and serialization delay of one maximum-sized frame:

$$T = \frac{L_{\max, \text{port}}}{C} + t_{\text{switch\_tech}}$$

### End-to-End Latency and Jitter Bounds

For a Virtual Link traversing $H$ switch hops, the maximum end-to-end packet latency $D_{\max}$ is bounded by the horizontal deviation between aggregate arrival curve $\sum \alpha_i$ and convolution of service curves $\bigotimes \beta_j$:

$$D_{\max} \le \frac{\sum_{i \in \text{VL}} L_{\max, i}}{C} + H \cdot T$$

Maximum accumulated transmission jitter $J$ satisfies the strict ARINC 664 safety envelope:

$$J \le 40\,\mu\text{s} + \frac{\sum L_{\max}}{C} \le 500\,\mu\text{s}$$

guaranteeing bounded packet inter-arrival times at all flight control surface actuators.
