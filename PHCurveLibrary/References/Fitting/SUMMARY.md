# Bezier Curve Fitting with Pythagorean Hodograph (PH) Curves

## Introduction
Curve fitting using Bezier and spline curves is a core problem in computer graphics, CAD/CAM, and robotics. Classical polynomial or spline methods often lack closed-form solutions for important geometric quantities such as arc length and curvature. **Pythagorean Hodograph (PH) curves**, a special class of polynomial curves, address these shortcomings by offering closed-form expressions for arc length and rational offsets.

This document provides a comprehensive overview of the mathematical foundation, key algorithms, and publications on PH curve fitting, focusing primarily on applications in computer graphics while also covering CAD, CNC, and robotics. We explain the mathematics in detail, compare algorithms, and evaluate performance and usability. Free PDF download links are provided where possible.

## Mathematical Foundation of PH Curves

### Classical Bezier Curves
A planar Bezier curve of degree \( n \) is typically expressed as:
\[
P(t) = \sum_{i=0}^{n} B_i^n(t) P_i,
\]
where \( B_i^n(t) \) are Bernstein polynomials and \( P_i \) are control points.

### Definition of PH Curves
A polynomial curve \( P(t) = (x(t), y(t)) \) is a **Pythagorean Hodograph** curve if:
\[
x'(t)^2 + y'(t)^2 = \sigma(t)^2,
\]
where \( \sigma(t) \) is a polynomial. This condition allows:
- Closed-form arc length: \( s(t) = \int_0^t \sigma(u) \, du \)
- Rational offset curves
- Easy speed profiling and interpolation

The key idea: represent \( x'(t) + i y'(t) = [f(t) + i g(t)]^2 \) with real polynomials \( f(t), g(t) \). Then,
\[
x'(t)^2 + y'(t)^2 = (f(t)^2 + g(t)^2)^2.
\]

### Practical Benefits
- Exact arc-length parametrization
- Rational offset curves (important for tool-path planning)
- Continuous curvature (\( G^2 \)) interpolation
- Analytical inverse time-parameterization (\( s(t) \rightarrow t(s) \))

These features make PH curves ideal for CNC interpolation, motion control, and path smoothing in graphics and robotics.

## Key Algorithms and Publications

### 1. Local Hermite Interpolation (Meek & Walton, Farouki et al.)
- Construct piecewise PH segments (often quintics)
- Hermite data: endpoints, tangent directions, and curvature
- Solve nonlinear systems segment-wise using Newton's method
- Suitable for interactive and local editing

### 2. Global Least-Squares Approximation (Farouki, Saitou, Tsai 1998)
- Minimize squared error between given data and PH curve
- Use Newton-Raphson or simulated annealing
- Adaptive segmentation: subdivide if error exceeds threshold
- Footpoint refinement: reassign parameter values iteratively
- Add penalty terms (e.g., bending energy, rotation number)
- PDF link: [Free download available](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.38.7640&rep=rep1&type=pdf) [Local PDF](Farouki_Saitou_Tsai_1998_Least_Squares_Approximation_PH_Curvs.pdf)

### 3. Evolution-Based Curve Fitting (Aigner, Jüttler et al. 2007)
- Dynamic evolution: \( P(t, \tau) \) evolves over fictitious time \( \tau \)
- Inspired by active contour models
- Gauss-Newton-like convergence
- Robust to parametrization issues
- PDF link: [Free download available](https://www.ag.jku.at/ftp/pub/Preprints/JKU-2007-02.pdf) [Local PDF](Aigner_Sir_Jüttler_2007_Least_Squares_Fitting.pdf)

### 4. Homotopy Continuation (Albrecht & Farouki 1996)
- Connect simple initial curve to desired target
- Ensure \( C^2 \) continuity across PH segments
- Solve complex interpolation problems globally
- PDF link: [Local PDF](Fitting/Farouki_Albrecht_1996_C2_PH%20Curve_homotopy_methode.pdf)

### 5. Theoretical and Recent Advances
- **Weierstrass-type theorem** for PH curves (Choi & Moon 2008): they can approximate any smooth path
 - **Rational PH curves** and Minkowski-space PH curves (Kosinka & Lavička 2014) [Local PDF](Kosinka_Lavicka_2014_Survey_of_Recent_Advances.pdf)
- **Applications in robotics**: PH curves support curvature and heading constraints
 - **High-speed CNC**: produce vibration-free motion and uniform material removal ([Local PDF](Farouki_Computer_numerical_control_algorithms.pdf))

## Performance Evaluation

| Method                    | Type       | Accuracy     | Speed           | Pros                                               | Cons                                          |
|--------------------------|------------|--------------|------------------|----------------------------------------------------|-----------------------------------------------|
| Hermite Interpolation    | Local      | Medium       | Fast             | Easy to implement, interactive                    | No global error control                       |
| Least-Squares (Farouki)  | Global     | High         | Moderate         | High precision, adaptive, analytical arc length   | Nonlinear system, possible local minima       |
| Evolution-based (Jüttler)| Global     | High         | Slow (iterative) | Robust to initialization, flexible                | Computationally heavy                         |
| Homotopy (Albrecht)      | Global     | High         | Variable         | Solves hard problems via continuation             | Complex to implement                          |

## Application Scenarios

- **Computer Graphics**: Fair curve design, offset control, arc-length reparametrization
- **CNC and CAD/CAM**: Real-time interpolators, vibration-free high-speed machining
- **Robotics and Path Planning**: Curvature-limited, time-synchronized paths

## Conclusion
PH curves offer a compelling combination of geometric flexibility and analytical properties. Their ability to support closed-form arc length, curvature continuity, and rational offsets makes them uniquely suited for applications requiring high spatial and temporal precision.

While more complex than traditional splines, PH curve fitting methods have matured into practical tools for engineering and graphics. Algorithms range from fast local interpolators to robust global optimizers. When implemented carefully, they can dramatically improve the quality and efficiency of path-based systems.

## Recommended Reading and Downloads
 - Farouki et al. 1998: [Least-Squares Approximation with PH Curves (PDF)](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.38.7640&rep=rep1&type=pdf) [Local PDF](Farouki_Saitou_Tsai_1998_Least_Squares_Approximation_PH_Curvs.pdf)
 - Aigner et al. 2007: [Evolution-Based PH Fitting (PDF)](https://www.ag.jku.at/ftp/pub/Preprints/JKU-2007-02.pdf) [Local PDF](Aigner_Sir_Jüttler_2007_Least_Squares_Fitting.pdf)
 - Kosinka & Lavička 2014: *PH Curves Survey* (open access) [Local PDF](Kosinka_Lavicka_2014_Survey_of_Recent_Advances.pdf)
 - Pastva 1998: *Bezier Curve Fitting* [Local PDF](Pastva_1998_Bezier_Curve_Fitting.pdf)
 - Farouki: *Computer Numerical Control Algorithms* [Local PDF](Farouki_Computer_numerical_control_algorithms.pdf)
- Farouki 2008: *Pythagorean-Hodograph Curves: Algebra and Geometry Inseparable* (book)
- Albrecht & Farouki 1996: *C² PH Splines via Homotopy* [Local PDF](Fitting/Farouki_Albrecht_1996_C2_PH%20Curve_homotopy_methode.pdf)

Let me know if you'd like code implementations or tutorials for any of these methods.

