# AGENTS.md

This document outlines the agent-based architecture of the **PHCurveLibrary** for Pythagorean-Hodograph (PH) curves with G² interpolation. It describes each component (“agent”) responsible for distinct tasks in curve creation, validation, path planning, and execution, along with the underlying mathematics and key references.

## Data Structures

Before defining agents, **PHCurveLibrary** includes two core data structures:

### HermiteControlPoint3D

* **Type:** `struct`
* **Purpose:** Encapsulates Hermite boundary data for a curve segment:

  * `Position` (Vector3): 3D coordinates of the endpoint.
  * `Tangent` (Vector3): Directional derivative at the endpoint.
  * `Curvature` (float): Signed curvature magnitude at the endpoint.
  * `PrincipalNormal` (Vector3): Direction of curvature in 3D space.
* **Use in agents:** Passed as input to `PHCurveFactory.CreateQuintic` to generate PH segments with G² continuity.

### PHCurve3D

* **Type:** `struct`
* **Purpose:** Represents a PH curve segment by storing its five hodograph coefficients (A, B, C, D, E).
* **Capabilities:** Evaluates position, derivatives, speed, tangent, normal, curvature, exact arc length, and offset points.
* **Use in agents:** Returned by `PHCurveFactory` and consumed by `PHCurveSegment` agent for evaluations and by `PathPlanner` for constructing paths.

## PHCurveSegment the agent-based architecture for Pythagorean-Hodograph (PH) curves with G² interpolation. Mathematical foundations and references are explicitly provided.

## PHCurveSegment

* **Responsibility:** Represents a single PH curve segment.
* **Mathematics:** A quintic PH segment is defined by its hodograph:
  $r'(t) = A + B t + C t^2 + D t^3 + E t^4,$
  and its position via integration:
  $r(t) = \int_0^t r'(u)\,du = A t + \frac{B t^2}{2} + \frac{C t^3}{3} + \frac{D t^4}{4} + \frac{E t^5}{5}.$
  Curvature is given by
  $\kappa(t) = \frac{\|r'(t) \times r''(t)\|}{\|r'(t)\|^3}.$
* **Tasks:**

  * Store coefficients $A, B, C, D, E$.
  * Evaluate position, first and second derivatives, unit tangent $T(t)=r'(t)/\|r'(t)\|$, curvature, principal normal, exact arc length, and offset points.
* **Interfaces:**

  * `Position(t)`, `Derivative(t)`, `SecondDerivative(t)`, `Speed(t)`, `TangentUnit(t)`, `PrincipalNormal(t)`, `OffsetPoint(t,d)`.
* **References:**

  * Farouki & Sakkalis (1990): Foundations of PH Quintics, *CAGD* 7(4), 285–294.
  * Meek & Walton (2007): G² Quintic Spiral Segments, *CAGD* 24(5), 267–285.

## PHCurveFactory

* **Responsibility:** Generates PH curve segments from Hermite boundary data.
* **Mathematics:** Solves the system for unknown vectors $B, C, D, E$:

  $$\begin{cases}
    A + B + C + D + E = T_1,\\
    A + \tfrac12B + \tfrac13C + \tfrac14D + \tfrac15E = P_1 - P_0,\\
    A\cdot C + \tfrac12 B\cdot B = 0,\\
    B\cdot D + C\cdot C + A\cdot E = 0,\\
    2C + 3D + 4E = \kappa_1\|T_1\|^2 N_1,
  \end{cases} \]
  which enforces Hermite and PH constraints as well as end curvature.
  $$
* **Tasks:**

  * `CreateQuintic(p0, p1)`: Solves this system to produce a `PHCurve3D`.
  * `ValidateG2(a, b)`: Checks G² continuity (position, tangent direction, and normal alignment at the join).
* **Interfaces:**

  * Factory methods for other degrees (e.g., `CreateCubic`, `CreateSeptic`).
* **References:**

  * Jaklić et al. (2015): G² Quintic Hermite Interpolation, *Numerical Mathematics: Theory, Methods and Applications* 8(3), 219–236.
  * Farouki & Dong (2012): PHquintic Library, Technical Report.

## PathPlanner

* **Responsibility:** Concatenates multiple PH curve segments into a continuous path.
* **Mathematics:** Ensures G² conditions at each junction: given segments $S_i$, require
  $T_i(1) = T_{i+1}(0), \quad N_i(1) = N_{i+1}(0).$
* **Tasks:**

  * `AddSegment(p_{i-1}, p_i)`: Creates and appends a segment via `PHCurveFactory`.
  * `BuildPath()`: Returns an ordered list of `PHCurve3D` segments.
  * `ValidatePathG2()`: Calls `ValidateG2` for each adjacent pair.
* **References:**

  * Albrecht & Farouki (1996): Homotopy Methods for PH Splines, *CAGD* 13(3), 291–305.

## AlternativeCurveAgent

* **Responsibility:** Provides alternative curve types for G² transitions.
* **Mathematics:** Clothoid (Euler spiral) curvature function
  $\kappa(s) = a s + b,$
  parameterized by Fresnel integrals.
* **Tasks:**

  * `CreateClothoid(p0, p1, κ0, κ1)`: Computes spiral parameters $a, b$.
  * `CreateRationalCubic(p0, p1, weights)`: Constructs rational Bézier curves.
* **References:**

  * Walton & Meek (1996): Clothoid Approximation, *CAGD* 13(9), 513–526.
  * Habib & Sakai (2010): Rational Cubic Spirals, *CAGD* journal.

## CurveOptimizer

* **Responsibility:** Selects the optimal curve among multiple candidates.
* **Mathematics:** Evaluates bending energy
  $E = \int_0^1 \kappa(t)^2 \,dt,$
  total length $L = \int_0^1 \|r'(t)\| \,dt$, and curvature variation.
* **Tasks:**

  * `Evaluate(curve)`: Computes quality metrics (E, L, max κ).
  * `SelectBest(candidates)`: Chooses the curve minimizing energy or variation.
* **References:**

  * Farouki et al. (2008): Optimization of Spatial PH Quintics, *CAGD* 25(3), 274–297.

## RealTimeController

* **Responsibility:** Executes the trajectory in real time.
* **Mathematics:** Arc-length parameterization $s(t)$ from exact PH length, velocity $v(t)=ds/dt$, lateral and longitudinal accelerations.
* **Tasks:**

  * `StartExecution(curve, profile)`: Initializes timing and motion profile.
  * `GetNextTarget(time)`: Returns position, velocity, acceleration using PH computations.
* **References:**

  * Han et al. (2009): PH Curves in Robotics, *IEEE Transactions on Robotics* 25(1), 5–14.
  * Farouki (2014): Offset Curves and Innovations in PH Design.
