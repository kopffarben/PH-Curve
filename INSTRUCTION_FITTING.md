# PH Curve Fitting Instructions

This document outlines how to approximate a sequence of measured points with Pythagorean Hodograph (PH) curves. Each fitting strategy is represented by a dedicated agent. Additionally, every variant offers an incremental method for real-time curve generation. Tests and benchmarks cover all implementations and compare fixed against incremental approaches.

## Common Requirements

- Input is a `List<PointData>` containing `Vector3 Position`, `Vector3 UpVector` and `float Time`. The list is sorted by `Time` and may contain noise.
- Output is a `List<PHCurve3D>` where every segment stores its absolute start and end time.
- `positionTolerance` and `orientationTolerance` bound the allowed deviation from the measured path.
- Up vectors should ideally coincide with the normal but position and tangent continuity are more important.
- After each new segment is created, perform a G² optimisation with the previous segment.
- Implement in `PathPlanner` the methods
  `CurveFitting(List<PointData> points, float positionTolerance, float orientationTolerance, FittingMethod method)` and
  `CurveFittingIncremental(List<PointData> buffer, float positionTolerance, float orientationTolerance, FittingMethod method)`.
  Both select the algorithm via the `method` parameter and return the generated `PHCurve3D` segments.

Each fitting agent is responsible for one algorithm and provides both fixed and incremental variants.

## 1. Local Hermite Interpolation

- **Reference:** [Jaklic et al. 2015](PHCurveLibrary/References/Jaklic_et_al_2015_G2_Quintic_PH_Interpolation.pdf)
- **Idea:** Build a quintic PH curve between each pair of points using their positions, tangents and curvature estimated from neighbours.
- **Advantages:**
  - Very fast because only small linear systems are solved.
  - G² continuity is achieved directly through Hermite data.
- **Disadvantages:**
  - No global error control; noisy data yields many short segments.
- **Complexity:** constant per segment, overall linear in the number of segments.
- **Agent Prompt:**
  1. Implement `FitLocalHermite(List<PointData> points, float positionTolerance, float orientationTolerance)`.
  2. Compute Hermite data from successive points and call `PHCurveFactory.CreateQuintic`.
  3. Apply a short smoothing step if the up vector varies strongly.
  4. Check the error; increase the segment count if tolerances are exceeded.
- **Incremental Variant:**
  - `FitLocalHermiteIncremental(List<PointData> buffer, ...)` reads new points each frame, creates segments until the tolerance would be violated, then clears the buffer and continues.

## 2. Global Least-Squares Approximation

- **Reference:** [Farouki, Saitou, Tsai 1998](PHCurveLibrary/References/Fitting/Farouki_Saitou_Tsai_1998_Least_Squares_Approximation_PH_Curvs.pdf)
- **Idea:** Determine PH parameters by minimising the quadratic error over all points of a segment. Use adaptive subdivision if the error remains too high.
- **Advantages:**
  - Very accurate approximation of the entire point sequence.
  - Allows controlled segment counts.
- **Disadvantages:**
  - Requires nonlinear optimisation and may converge to local minima.
  - High computational effort depending on the iteration count.
- **Complexity:** roughly `O(k n)` to `O(k n^2)` with `k` iterations and `n` points.
- **Agent Prompt:**
  1. Implement `FitLeastSquares(List<PointData> points, float positionTolerance, float orientationTolerance)`.
  2. Optimise the hodograph coefficients using Newton–Raphson and treat up-vector deviation as a soft penalty term.
  3. Subdivide the interval if the error exceeds the tolerance.
- **Incremental Variant:**
  - `FitLeastSquaresIncremental(...)` performs optimisation only on the current buffer and creates one or more segments once the tolerance is met.

## 3. Evolution Based Fitting

- **Reference:** [Aigner, Jüttler et al. 2007](PHCurveLibrary/References/Fitting/Aigner_Sir_Jüttler_2007_Least_Squares_Fitting.pdf)
- **Idea:** Start from an initial curve and evolve it toward the data by minimising an energy `E = αE_pos + βE_orient + γE_smooth`.
- **Advantages:**
  - Robust against outliers and inaccurate parameterisation.
  - Flexible combination with additional smoothing terms.
- **Disadvantages:**
  - Long run time due to many evolution steps.
  - Parameter choice (step size, weights) is non-trivial.
- **Complexity:** typically `O(k n)` or higher with `k` evolution steps and `n` points.
- **Agent Prompt:**
  1. `FitEvolution(List<PointData> points, float positionTolerance, float orientationTolerance)` creates an initial PH curve and evolves it until all tolerances are satisfied or a maximum number of steps is reached.
- **Incremental Variant:**
  - `FitEvolutionIncremental(...)` performs one evolution step per frame on the current buffer and materialises segments when the tolerance is satisfied.

## 4. Homotopy Continuation

- **Reference:** [Albrecht & Farouki 1996](PHCurveLibrary/References/Fitting/Farouki_Albrecht_1996_C2_PH%20Curve_homotopy_methode.pdf)
- **Idea:** Start from a simple curve and deform it via a homotopy parameter towards the final solution while keeping G² conditions.
- **Advantages:**
  - Avoids abrupt changes and can follow multiple valid solutions.
  - Suitable for highly nonlinear constraints.
- **Disadvantages:**
  - Complex implementation and sensitive step-size control.
  - Computational cost varies with the homotopy path.
- **Complexity:** usually `O(k n)` to `O(k n^2)` depending on the number of continuation steps.
- **Agent Prompt:**
  1. `FitHomotopy(List<PointData> points, float positionTolerance, float orientationTolerance)` initialises with a line or circular arc.
  2. Follow the solution path using a predictor–corrector method until all points lie within the tolerances.
  3. Refine the time parametrisation as needed.
- **Incremental Variant:**
  - `FitHomotopyIncremental(...)` reuses the previous homotopy path as the starting point for the next segment and generates new segments in real time.

## 5. Heuristic Segmentation with Post Smoothing

- **Reference:** [Kosinka & Lavicka 2014](PHCurveLibrary/References/Fitting/Kosinka_Lavicka_2014_Survey_of_Recent_Advances.pdf)
- **Idea:** Roughly segment the point list using distance, tangent change or timestamps. Apply one of the above methods within each segment and smooth the transitions afterwards.
- **Advantages:**
  - Lower optimisation cost because only partial tracks are fitted.
  - Allows a tunable balance between run time and accuracy.
- **Disadvantages:**
  - Result quality strongly depends on the segmentation heuristic.
  - G² continuity must be validated and possibly corrected.
- **Complexity:** ideally linear to slightly superlinear in the number of points, depending on the heuristic.
- **Agent Prompt:**
  1. `FitHeuristic(List<PointData> points, ...)` analyses the input, determines segment boundaries and invokes one of the methods 1–4.
  2. Transition areas are checked with `PHCurveFactory.ValidateG2` and refined if needed.
- **Incremental Variant:**
  - `FitHeuristicIncremental(...)` continuously monitors the incoming point buffer, marks a section as finished and smooths the joins to existing segments.

## Algorithm Comparison

The five variants differ in computational effort and quality:

| Method                  | Advantages                                | Disadvantages                          | Complexity           |
|-------------------------|-------------------------------------------|----------------------------------------|----------------------|
| Local Hermite          | Fast, easy implementation                 | Many segments for noisy data          | Linear in segments   |
| Least Squares          | Accurate, controllable segment count      | Nonlinear optimisation, local minima  | `O(k n)` to `O(k n^2)` |
| Evolution              | Robust and flexible                       | Long run time, difficult parameters   | mostly `O(k n)` or higher |
| Homotopy               | Follows continuous solutions              | Complex, variable effort              | `O(k n)` to `O(k n^2)` |
| Heuristic Segmentation | Reduced optimisation overhead             | Quality depends on heuristic          | near linear          |

## Test Guidelines

Every variant requires extensive unit tests in the `PHCurveLibrary.Test` project:

1. **Long Sequences:** Generate synthetic point sets exceeding one hundred entries and verify that the number of produced segments remains minimal.
2. **Up-Vector Consistency:** At sample points along each segment, check that the angle between computed normal and up vector does not exceed a threshold.
3. **Tolerance Variants:** Run the algorithms with several position and orientation tolerances (e.g. 0.1, 0.01, 0.001) and confirm that the errors scale accordingly.
4. **G² Continuity:** Use `PHCurveFactory.ValidateG2` to ensure that successive segments remain within the tolerance.
5. **Time (bounds):** Confirm that absolute start and end times of the segments exactly match the input points and increase monotonically.
6. **Time (intermediate):** At multiple times within a segment, compare position and up vector against the original points and verify the deviation stays below `positionTolerance` and `orientationTolerance`.

Console output should explain which mathematical property is validated, and all tests must be documented in English as per [AGENTS.md](AGENTS.md).

## Benchmarks

Simple benchmarks should be added to the test project using [BenchmarkDotNet](https://benchmarkdotnet.org/):

- Measure execution times for `FitLocalHermite`, `FitLeastSquares`, `FitEvolution`, `FitHomotopy` and their incremental counterparts on identical point sequences (at least one thousand points).
- Record the number of produced segments and the maximal fitting error.
- Summarise the results in a table to highlight performance and quality differences.

## Integrating Absolute Time

Extend `PHCurve3D` with `float StartTime` and `float EndTime`. All constructors and factory methods must supply these values, allowing every curve to be replayed in real time and aligned with the sensor frame.

## Summary

These instructions provide the basis for implementing various PH curve fitting methods including real-time variants. Each method is represented by its own agent and selected via the `method` parameter in the `PathPlanner` entry points.
