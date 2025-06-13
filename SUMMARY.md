# Summary of Reference Papers

This document summarises the main results of the PDF files located in the `References` folder. It also explains how these concepts relate to the current implementation of **PH-Curve** and outlines potential extensions and new features.

## 1. Farouki & Saguin-Sprynski (2014)
**C2 interpolation of spatial data subject to arc-length constraints using Pythagorean–hodograph quintic splines** describes a method to reconstruct spatial curves when only relative distances between measurement frames are known. The authors derive two PH quintic spline formulations that enforce prescribed segment arc lengths. The problem reduces to minimising a quadratic energy subject to quadratic constraints on the PH coefficients. The key mathematical insight is that PH curves have polynomial speed \(\sigma(t)\), allowing the arc-length conditions to be expressed algebraically. The spline segments are constructed in quaternion form and the remaining degrees of freedom are fixed via shape optimisation.

## 2. Farouki et al. (2008)
**Identification of spatial PH quintic Hermite interpolants with near-optimal shape measures** examines how to choose the free parameters arising in PH quintic Hermite interpolation. After analysing when the ordinary cubic interpolant is already a PH curve, the authors show that the arc length depends on one parameter only. They propose three practical selection criteria: (i) minimising a bivariate distance function to a PH cubic, (ii) fixing the parameter associated with extremal arc length and then minimising a univariate function, and (iii) a heuristic two-step approach. The study relies on quaternion algebra and demonstrates that general helical PH quintics always exist and correspond to arc-length extrema.

## 3. Jaklič et al. (2015)
**Interpolation by G² quintic Pythagorean-hodograph curves** focuses on constructing G² continuous splines that interpolate two points with given tangents and curvature vectors in \(\mathbb{R}^d\). The problem is reduced to solving two polynomial equations in the unknown tangent lengths. The authors present an asymptotic analysis for smooth data and solve the system by homotopy continuation. Several solutions may exist, so the continuation method is used to trace the one with the best approximation order.

## 4. Kozak (2014)
The paper titled **Global well-posedness of strong solutions to the 3D primitive equations with horizontal eddy diffusivity** deals with partial differential equations in fluid dynamics. Although unrelated to PH curves, it establishes existence of strong solutions for the primitive equations with horizontal diffusion. This document is included in the repository but does not contribute directly to the PH-curve implementation.

## 5. Farouki & Dong (2012)
**PHquintic: A library of basic functions for the construction and analysis of planar quintic Pythagorean–hodograph curves** presents an extensive software library based on the complex representation of planar PH quintics. The library supports exact arc-length computation, offset curves, bending energy evaluation and Hermite interpolation. It highlights that PH quintics are the lowest order curves suitable for free-form design due to their ability to inflect and interpolate arbitrary first-order Hermite data.

## Relation to the Implementation
The `PHCurveLibrary` project mirrors many concepts from the literature:
- `PHCurve3D` represents a quintic PH segment defined by its hodograph coefficients A–E and offers methods for position, derivatives, speed and arc length. It also computes Frenet-frame vectors.
- `HermiteControlPoint3D` encapsulates position, tangent, curvature and principal normal data as described in the interpolation papers.
- `PHCurveFactory` solves the G² Hermite interpolation problem following Jaklič et al. (2015). It uses a linear algebra system to determine the remaining coefficients and provides validation of G² continuity.
- `PathPlanner` assembles multiple segments ensuring tangents and normals match at the joins, aligning with the spline constructions in the references.
The accompanying unit tests verify the mathematical formulas and reference results from Farouki & Dong (2012).

## Possible Extensions
The references suggest several avenues for enhancement:
1. **Arc-length Constrained Splines** – Implement the optimisation strategy from Farouki & Saguin-Sprynski (2014) to fit measurement data with specified segment lengths.
2. **Automatic Parameter Selection** – Integrate the selection criteria from Farouki et al. (2008) to choose free parameters for improved fairness without manual tuning.
3. **Homotopy-Based Solvers** – Adopt the homotopy continuation approach of Jaklič et al. (2015) for robust solution tracking when multiple interpolants exist.
4. **Planar Library Integration** – Provide wrappers for the planar PHquintic functions of Farouki & Dong (2012) to support 2D applications and offset curves with exact formulas.
5. **Rational PH Curves** – Explore rational PH curves and rotation-minimising frames as discussed in the broader PH curve literature.

## Potential New Features
Based on these ideas, the following features could extend the current project:
- **C² PH Spline Fitting** with optional arc-length constraints for sensor-based shape reconstruction.
- **Curve Optimisation Module** that evaluates bending energy, arc length and curvature variation to select the best interpolant.
- **Homotopy Continuation Toolkit** for tracing solutions across parameter variations, aiding in complex interpolation tasks.
- **Real-Time Trajectory Execution** using exact arc-length parameterisation and motion profiles suitable for robotics.
- **Support for Clothoids or Rational Cubics** as alternative transitions when PH solutions are unsuitable.

These extensions would strengthen the library and broaden its applicability to robotics, CAD and animation.

