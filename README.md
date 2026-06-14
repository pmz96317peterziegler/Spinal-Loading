# Spinal & Hip Loading Over Lifting Positions

A 2D quasi-static biomechanical model that estimates the **erector-spinae (spinal) muscle force** and **hip joint reaction force** required to hold a load across a continuum of lifting postures. The body is modeled as a six-link planar chain; **inverse kinematics** generates the postures and a **static-equilibrium** analysis solves the internal loads at each one. The motivation is occupational: quantifying *why* "lift with your legs" reduces lower-back load.

![Spinal force versus knee angle](results/spinal_force_vs_knee_angle.png)

> Spinal (erector-spinae) force falls from ~7000 N in a stooped, straight-leg posture to ~2000 N in a deep squat. Which is around a 3.5× reduction in low-back muscle load from posture alone.

---

## Overview

When someone lifts or holds an object, the lower-back muscles and the hip joint carry large internal loads that depend strongly on posture. A stooped lift loads the spine very differently from a squat. This project quantifies that relationship.

The body is represented as six rigid links: **foot, shank (leg), thigh, trunk–head–neck (THN), upper arm, and forearm-plus-hand**. These are connected by revolute joints, with the foot fixed to the ground. The model reaches a fixed target hand position through a family of postures, and at each posture solves the statics for:

- the **compressive** and **shear** components of force along the spine, and
- the net **spinal muscle force** and **hip joint reaction force**.

Results are reported against **knee angle**, a single scalar that captures how squatted vs. upright the posture is.

## Model

### Segments & anthropometry

Segment lengths are scaled from the subject's height $H$ using standard height-fraction landmarks (Winter, *Biomechanics and Motor Control of Human Movement*); segment masses are fractions of total body mass $M$. Because the model is a single sagittal-plane chain, the **paired** limbs (feet, shanks, thighs, arms) carry **double mass** to represent both sides, while the trunk–head–neck is counted once.

| Segment | Length | Mass |
|---|---|---|
| Foot | $0.152\,H \cdot 0.8$ | $2 \times 0.0145\,M$ |
| Shank (leg) | $(0.285-0.039)\,H$ | $2 \times 0.0465\,M$ |
| Thigh | $(0.48-0.285)\,H$ | $2 \times 0.10\,M$ |
| Trunk–head–neck | $(0.818-0.48)\,H$ | $0.578\,M$ |
| Upper arm | $0.186\,H$ | $2 \times 0.028\,M$ |
| Forearm + hand | $(0.145+0.108)\,H$ | $2 \times (0.016+0.006)\,M$ |

**Subject / load:** $H = 1.6\ \text{m}$, $M = 60\ \text{kg}$, object mass $m_o = 5\ \text{kg}$ (≈ 11 lb, a moderate hand-held load), $g = 9.81\ \text{m/s}^2$.

### Kinematics (homogeneous transforms)

Each joint contributes a rotation $R(\theta_i)$ and each segment a translation $L_i$ along its length, written as $4\times4$ homogeneous transformation matrices (a planar simplification rather than the full Denavit–Hartenberg parameter set):

$$
R(\theta_i)=
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i & 0 & 0\\
\sin\theta_i & \cos\theta_i & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix},
\qquad
L_i=
\begin{bmatrix}
1 & 0 & 0 & \ell_i\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Chaining them from the fixed foot up the body gives the global position of every joint and segment center of mass — e.g. for the lower limb:

$$
T_{\text{knee}} = R_{\text{ground}}\,L_{\text{foot}}\,R_{\text{ankle}}\,L_{\text{leg}},
\qquad
T_{\text{hip}} = T_{\text{knee}}\,R_{\text{knee}}\,L_{\text{thigh}}
$$

The whole-body center of mass (with the held object) is the mass-weighted average of the segment COMs:

$$
x_{cg} = \frac{\sum_i m_i\,x_{cg,i} + m_o\,x_{cg,o}}{M + m_o}
$$

### Inverse kinematics: `compute_IK_solutions.m`

The six-link chain reaching a 2D hand target is **kinematically redundant** — many joint configurations satisfy the same target. The solver builds a forward-kinematics model of the fingertip and uses MATLAB's `fsolve` to drive it to the target $(x,y) = (0.2,\,0.1)\ \text{m}$.

To sample a realistic *family* of solutions rather than one arbitrary pose, it seeds the solver with **20 initial guesses linearly interpolated between two reference postures**. These include a deep squat (bent knee, bent elbow) and a near-upright stance (locked knee, straight arm):

$$
\theta_{\text{guess}}(t) = (1-t)\,\theta_{\text{squat}} + t\,\theta_{\text{upright}},
\qquad t = 0,\dots,1
$$

producing 20 postures that smoothly span the squat → stoop range.

### Statics: `final_analysis.m`

At each posture, an **imaginary cut at the hip** exposes two internal loads: the **hip joint reaction force** $\vec F_J=(F_{Jx},F_{Jy})$ and the **erector-spinae muscle force** $F_m$ acting along the muscle's line of action. The muscle is modeled with a distal attachment 75 % up the trunk and a proximal attachment offset 5 % of the THN length from the spine at the hip.

Force and moment balance on the lower body (foot + shank + thigh, with the ground reaction $F_N = (M+m_o)g$ acting at $x_{cg}$) give three equations in three unknowns:

$$
\sum F_x = 0:\quad F_{Jx} + F_m\,\hat u_x = 0
$$

$$
\sum F_y = 0:\quad F_{Jy} + F_m\,\hat u_y + (m_{\text{foot}}+m_{\text{leg}}+m_{\text{thigh}})g - (M+m_o)g = 0
$$

$$
\sum M_O = 0:\quad (\vec r_J \times \vec F_J)_z + F_m\,(\vec r_m \times \hat u)_z + \sum_i (\vec r_i \times \vec W_i)_z + (\vec r_{F_N}\times \vec F_N)_z = 0
$$

assembled as a $3\times3$ linear system $A\,\vec x = \vec B$ and solved for $\vec x = (F_{Jx},\,F_{Jy},\,F_m)$.

The hip force is then projected onto the spine axis $\hat u_{\text{spine}}$ to separate **compression** (along the spine) from **shear** (across it):

$$
\hat u_{\text{spine}} = \frac{\vec p_{\text{shoulder}} - \vec p_{\text{hip}}}{\lVert \vec p_{\text{shoulder}} - \vec p_{\text{hip}} \rVert},
\qquad
\vec F_{\text{stab}} = (\vec F_J \cdot \hat u_{\text{spine}})\,\hat u_{\text{spine}},
\qquad
\vec F_{\text{shear}} = \vec F_J - \vec F_{\text{stab}}
$$

A **stability test** checks whether the whole-body COM lies over the base of support (the foot), flagging each posture as *stable*, *falling forward*, or *falling backward*.

## Results

| Spinal force | Shear force | Stability (compression) force |
|---|---|---|
| ![Spinal force](results/spinal_force_vs_knee_angle.png) | ![Shear force](results/shear_force.png) | ![Stability force](results/stability_force.png) |

**Spinal muscle force.** Compression dominates and scales with the trunk's moment arm about the hip. In the stooped, straight-leg postures the spine carries **> 6000 N**; in a deep squat it drops to **~2500 N** — the quantitative explanation for "lift with your legs." (The peak actually occurs slightly off the fully-locked posture, near a 40° knee angle, before falling as the knees bend further.)

**Shear force** stays small (≈ 6–14 N) across all postures, confirming the hip load is almost purely axial along the spine.

**Stability force** mirrors the spinal-compression trend, trending toward zero as the knees bend and the load is brought closer to the body.

**Postures.** Stick-figure plots show each modeled posture with every segment center of mass marked (red), the whole-body COM (black), and the erector-spinae line of action (red).

| Position 1 (deep squat) | Position 20 (upright) |
|---|---|
| ![Position 1](results/position_1.png) | ![Position 20](results/position_20.png) |

## Repository structure

```
.
├── src/
│   ├── final_analysis.m         # entry point: statics, spine/shear decomposition, plots
│   └── compute_IK_solutions.m   # inverse kinematics (forward model + fsolve, 20 postures)
├── results/                     # rendered figures
└── README.md
```

## How to run

Requirements: **MATLAB** with the **Optimization Toolbox** (the inverse kinematics uses `fsolve`).

```matlab
% with src/ on the MATLAB path
final_analysis   % runs IK + statics, prints a stability verdict per posture,
                 % and produces the posture and summary-force plots
```

`final_analysis.m` calls `compute_IK_solutions.m`, so both files must be on the path. The target hand position (`x_target`, `y_target`), body height, body mass, and object mass are set at the top of `final_analysis.m`.

## Modeling assumptions & limitations

- **Planar (2D), symmetric.** Motion is confined to the sagittal plane; left and right limbs are lumped into single links with doubled mass. No out-of-plane rotation or lateral sway.
- **Quasi-static.** Each posture is analyzed in static equilibrium; the inertial effects of an actual moving lift are not modeled and would raise peak loads.
- **Single equivalent spinal muscle.** The erector spinae is represented by one line of action with an assumed attachment geometry (distal ≈ 75 % up the trunk, proximal offset ≈ 5 % of THN length from the spine). These attachment assumptions are the model's largest source of uncertainty and dominate the spinal-force estimate. Antagonist co-contraction and passive tissue forces are neglected.
- **Population anthropometry** scaled from height and total mass rather than subject-specific measurement.
- **Redundant IK.** Because many postures reach the same target, the specific solutions returned depend on the two reference postures used to seed the solver.

## Possible extensions

- Replace the single-muscle assumption with a multi-muscle model.
- Add inertial terms to extend from quasi-static to fully dynamic lifting.
- Sweep object mass and target location to map spinal load against lift parameters (an injury-risk surface).
- Subject-specific anthropometry and measured muscle attachment points.

## Applications

- Designing safer occupational lifting protocols to reduce low-back injury and chronic pain.
- Kinematic models for rehabilitation and range-of-motion assessment.
- Specifying assistive/exoskeleton devices that offload spinal compression during heavy labor.

## Authors

Group project: Nick Linkowski, Ben Brown, Peter Ziegler, Aidan Jones, Brady Stein, Bara Mbaye.
