# AGENTS.md

This document outlines the agent-based architecture of the **PHCurveLibrary** for Pythagorean-Hodograph (PH) curves with G² interpolation. It describes each component (“agent”) and the core data structures, along with the underlying mathematics and key literature references.
Ignore all commented-out instructions.

<!---  
irgnore this section, it is only for documentation purposes
--->


## File Layout

The library is organised into separate source files:

- `HermiteControlPoint3D.cs` defines the Hermite endpoint data structure.
- `PHCurve3D.cs` implements the PH curve representation.
- `PHCurveFactory.cs` provides construction and validation helpers.

## Documentation Requirements

- **Language:** All comments must be written in English.
- **Public members:** should include XML documentation comments.
- **Mathematical context:** All comments should include XML documentation summarising the underlying mathematics. Cite relevant formulas or algorithms and include academic references when appropriate.

---

## Data Structures

### HermiteControlPoint3D
- **Type:** `struct`  
- **Purpose:** Encapsulates the Hermite boundary conditions at one endpoint of a PH segment, including:
  - `Position` (`Vector3`): 3D point coordinates.
  - `Tangent` (`Vector3`): First derivative direction.
  - `Curvature` (`float`): Signed curvature magnitude.
  - `PrincipalNormal` (`Vector3`): Direction of curvature in 3D.
- **Usage:** Passed into `PHCurveFactory.CreateQuintic(...)` to specify G² interpolation data.

### PHCurve3D
- **Type:** `struct`  
- **Purpose:** Represents one PH-Quintic segment by its five hodograph coefficients A–E.
- **Capabilities:**  
  - `Position(t)`, `Derivative(t)`, `SecondDerivative(t)`  
  - `Speed(t) = ||r'(t)||`, `TangentUnit(t)`, `PrincipalNormal(t)`  
  - Exact arc-length calculation and offset‐point generation.  
- **Usage:**  
  - Produced by `PHCurveFactory`.  
  - Consumed by path‐planning and real‐time agents for evaluation.

---

## PHCurveFactory
- **Responsibility:** Construct PH segments from Hermite endpoints.
- **Mathematical Core:** Solves for B, C, D, E in the system
  \[
  \begin{cases}
    A + B + C + D + E = T_1,\\
    A + \tfrac12B + \tfrac13C + \tfrac14D + \tfrac15E = P_1 - P_0,\\
    A\cdot C + \tfrac12 B\cdot B = 0,\\
    B\cdot D + C\cdot C + A\cdot E = 0,\\
    2C + 3D + 4E = \kappa_1\,\|T_1\|^2\,N_1,
  \end{cases}
  \]
  enforcing Hermite‐position, tangent, PH‐orthogonality, and end‐curvature constraints.
- **Tasks:**
  - `CreateQuintic(p0, p1)` → returns `PHCurve3D`.
  - `ValidateG2(a, b)` → checks position, tangent, normal continuity.
- **References:**
  - Jaklić et al. (2015), _G² Quintic Hermite Interpolation_, NM TMA 8(3):219–236. [Preprint](https://osebje.famnit.upr.si/~vito.vitrih/papers/G2PHDeg5_NM_TMA_revision.pdf) [Local PDF](References/Jaklic_et_al_2015_G2_Quintic_PH_Interpolation.pdf)
  - Farouki & Dong (2012), _PHquintic Library_, Technical Report. [PDF](https://escholarship.org/content/qt1jk437p5/qt1jk437p5_noSplash_4531c0e73cf4cf42e7af65473e741413.pdf) [Local PDF](References/Farouki_Dong_2012_PHquintic_Library.pdf)

---

## PathPlanner
- **Responsibility:** Assemble a multi‐segment PH path.
- **Mathematical Core:** Ensures at each join \(T_i(1)=T_{i+1}(0)\) and \(N_i(1)=N_{i+1}(0)\).
- **Tasks:**
  - `AddSegment(p_{i-1}, p_i)` → uses `PHCurveFactory`.
  - `BuildPath()` → returns `List<PHCurve3D>`.
  - `ValidatePathG2()` → iterates `ValidateG2`.
- **References:**
  - Albrecht & Farouki (1996), _Homotopy Methods for PH Splines_, CAGD 13(3):291–305. [DOI](https://doi.org/10.1007/BF02124754)

---

## AlternativeCurveAgent
- **Responsibility:** Offer non-PH G² transitions.
- **Mathematical Core:**  
  - **Clothoid (Euler Spiral):** \(\kappa(s)=a s + b\), parameterized by Fresnel integrals.  
  - **Rational Cubic:** Weighted Bézier form for end‐curvature control.
- **Tasks:**
  - `CreateClothoid(p0, p1, κ0, κ1)`
  - `CreateRationalCubic(p0, p1, weights)`
- **References:**
  - Walton & Meek (1996), _Clothoid Approximation_, CAGD 13(9):513–526. [PDF](https://www.researchgate.net/profile/Dereck_Meek/publication/222850474_G2_curve_design_with_a_pair_of_Pythagorean_Hodograph_quintic_spiral_segments/links/0fcfd50bb539c74254000000/G2-curve-design-with-a-pair-of-Pythagorean-Hodograph-quintic-spiral-segments.pdf)
  - Habib & Sakai (2010), _Rational Cubic Spirals_, CAGD. [DOI](https://doi.org/10.1016/j.cad.2010.07.006)

---

## CurveOptimizer
- **Responsibility:** Select the best curve among multiple candidates.
- **Mathematical Core:**  
  - Bending energy \(E=\int_0^1\kappa(t)^2\,dt\).  
  - Path length \(L=\int_0^1\|r'(t)\|\,dt\).  
  - Curvature variation metrics.
- **Tasks:**
  - `Evaluate(curve)` → computes quality metrics.
  - `SelectBest(candidates)` → picks minimal‐energy solution.
- **References:**
  - Farouki et al. (2008), _Optimization of Spatial PH Quintics_, CAGD 25(3):274–297. [DOI](https://doi.org/10.1016/j.cagd.2007.09.007) [Local PDF](References/Farouki_et_al_2008_Identification_of_Spatial_PH_Quintic_Hermite_Interpolants.pdf)

---
<!---
irgnore this section, it is only for documentation purposes

## RealTimeController
- **Responsibility:** Execute PH trajectory in real time.
- **Mathematical Core:**  
  - Exact arc‐length parameterization \(s(t)\).  
  - Velocity \(v(t)=ds/dt\).  
  - Acceleration profiles from PH derivatives.
- **Tasks:**
  - `StartExecution(curve, profile)`
  - `GetNextTarget(time)` → (position, velocity, acceleration).
- **References:**
  - Han et al. (2009), _PH Curves in Robotics_, IEEE Trans. Robotics 25(1):5–14.
  - Farouki (2014), _Offset Curves & Innovations in PH Design_.

---
--->

## Additional References
For a summary of these papers, see [SUMMARY.md](References/SUMMARY.md).

- **Farouki & Dong (2012), _PHquintic Library_**
  [PDF (UC Davis eScholarship)](https://escholarship.org/content/qt1jk437p5/qt1jk437p5_noSplash_4531c0e73cf4cf42e7af65473e741413.pdf)
  [Local PDF](References/Farouki_Dong_2012_PHquintic_Library.pdf)

- **Jaklič et al. (2015), _G² Quintic PH-Interpolation_**
  [Preprint (University of Primorska)](https://osebje.famnit.upr.si/~vito.vitrih/papers/G2PHDeg5_NM_TMA_revision.pdf)
  [Local PDF](References/Jaklic_et_al_2015_G2_Quintic_PH_Interpolation.pdf)

- **Meek & Walton (2007), _G² PH-Quintic Spirale_**
  [PDF (ResearchGate)](https://www.researchgate.net/profile/Dereck_Meek/publication/222850474_G2_curve_design_with_a_pair_of_Pythagorean_Hodograph_quintic_spiral_segments/links/0fcfd50bb539c74254000000/G2-curve-design-with-a-pair-of-Pythagorean-Hodograph-quintic-spiral-segments.pdf)

- **Kozak (2014), _Spatial Rational PH Cubic_**
  [PDF (arXiv)](https://arxiv.org/pdf/1401.1234.pdf)
  [Local PDF](References/Kozak_2014_Spatial_Rational_PH_Cubic.pdf)

- **Farouki et al. (2008), _Spatial PH Quintic Optimisation_**
  [PDF (arXiv)](https://www.academia.edu/16097877/Identification_of_spatial_PH_quintic_Hermite_interpolants_with_near-optimal_shape_measures)
  [Local PDF](References/Farouki_et_al_2008_Identification_of_Spatial_PH_Quintic_Hermite_Interpolants.pdf)

- **Walton & Meek (1996), _Clothoid Approximation_**
  [PDF (ResearchGate)](https://www.researchgate.net/profile/Dereck_Meek/publication/222850474_G2_curve_design_with_a_pair_of_Pythagorean_Hodograph_quintic_spiral_segments/links/0fcfd50bb539c74254000000/G2-curve-design-with-a-pair-of-Pythagorean-Hodograph-quintic-spiral-segments.pdf)

- **Farouki & Saguin-Sprynski (2014), C2 interpolation of spatial data subject to arc-length constraints**
  [PDF (ResearchGate)](https://www.researchgate.net/profile/Nathalie-Saguin-Sprynski/publication/259161443_C2_interpolation_of_spatial_data_subject_to_arc-length_constraints_using_Pythagorean-hodograph_quintic_splines/links/5f0c38e3299bf1074452d3aa/C2-interpolation-of-spatial-data-subject-to-arc-length-constraints-using-Pythagorean-hodograph-quintic-splines.pdf?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InB1YmxpY2F0aW9uIiwicGFnZSI6InB1YmxpY2F0aW9uIiwicHJldmlvdXNQYWdlIjoiX2RpcmVjdCJ9fQ)
  [Local PDF](References/Farouki_Saguin_Sprynski_2014_C2_Interpolation.pdf)

- **Farouki et al. (2008), _Identification of Spatial PH Quintic Hermite Interpolants with Near-Optimal Shape Measures_**
  [Download PDF (ResearchGate / Academia.edu)](https://www.academia.edu/16097877/Identification_of_spatial_PH_quintic_Hermite_interpolants_with_near-optimal_shape_measures)
  [Local PDF](References/Farouki_et_al_2008_Identification_of_Spatial_PH_Quintic_Hermite_Interpolants.pdf)
- **Schröcker & Šír (2023), _Optimal interpolation with spatial rational PH curves_**
  [PDF (arXiv)](https://arxiv.org/pdf/2302.04632.pdf)
  [Local PDF](References/Schroecker_Sir_2023_Optimal_Interpolation_with_Spatial_Rational_PH_Curves.pdf)
- **Arrizabalaga & Ryll (2022), _Spatial motion planning with Pythagorean Hodograph curves_**
  [PDF (arXiv)](https://arxiv.org/pdf/2209.01673.pdf)
  [Local PDF](References/Arrizabalaga_Ryll_2022_Spatial_Motion_Planning_with_PH_Curves.pdf)
- **Arrizabalaga et al. (2024), _PHODCOS: Pythagorean Hodograph-based Differentiable Coordinate System_**
  [PDF (arXiv)](https://arxiv.org/pdf/2410.07750.pdf)
  [Local PDF](References/Arrizabalaga_et_al_2024_PHODCOS.pdf)
