# Fitting Algorithms

This document summarises the curve fitting components of **PHCurveLibrary**. Each fitter implements one algorithm for approximating a sequence of measured points with a G² continuous quintic Pythagorean hodograph (PH) curve.

The implementations follow the publications listed in `PHCurveLibrary/References` and correspond to the design described in `INSTRUCTION_FITTING.md`.

## Local Hermite Interpolation

*Reference:* Jaklić et al., "G² Quintic Hermite Interpolation" (2015).

The local Hermite fitter constructs a PH segment between each consecutive pair of sample points. Tangent directions are estimated from neighbouring points while curvature is set to zero. The algorithm searches for the longest segment whose positional and orientation errors remain below the tolerances. No global optimisation is performed, so noisy data leads to many short segments.

````csharp
// Example usage
var path = PathPlanner.CurveFitting(points, 0.01f, 0.01f, FittingMethod.LocalHermite);
````

## Least–Squares Approximation

*Reference:* Farouki, Saitou & Tsai, "Least-Squares Approximation of Spatial Pythagorean-Hodograph Curves" (1998).

This fitter estimates tangent lengths and endpoint curvatures from the point cloud and constructs a PH segment that minimises the squared deviation. The approach is more accurate than the purely local method but still solves only small linear systems. Iterative refinement is limited to curvature estimation.

````csharp
var segments = PathPlanner.CurveFitting(points, 0.01f, 0.01f, FittingMethod.LeastSquares);
````

## Evolution-Based Fitting

*Reference:* Aigner, Šir & Jüttler, "Least-Squares Fitting of PH Curves by Evolution" (2007).

Starting from a local Hermite interpolation the curve is evolved toward the data by minimising an energy containing positional and orientation terms. Each iteration performs a Gauss–Newton step on the hodograph coefficients. The implementation limits the number of steps and applies a small step size to maintain stability.

## Homotopy Continuation

*Reference:* Albrecht & Farouki, "Homotopy Methods for Pythagorean-Hodograph Splines" (1996).

The homotopy fitter gradually deforms a simple initial segment into the target PH solution. A predictor–corrector strategy increases the homotopy parameter until the deviation from the data falls below the specified tolerances. If the continuation fails the algorithm falls back to local Hermite interpolation.

## Heuristic Segmentation

*Reference:* Kosinka & Lavička, "Survey of Recent Advances in PH Curves" (2014).

Here the input sequence is split whenever positional or time gaps exceed a heuristic threshold. Each fragment is then fitted with the local Hermite approach. Adjacent segments are optimised for G² continuity.

## Testing with Sampled Point Sequences

To verify the fitters, unit tests in `FittingSamplesTests.cs` create point lists from known PH curves using `PHCurveFactory.CreateQuintic`. Five distinct sequences are generated:

1. Straight line segment.
2. Quarter circle in the *xy*-plane.
3. Short helix segment.
4. An S-shaped transition with opposing curvatures.
5. A reversed circular arc.

Each sequence is sampled at ten points. All fitters are executed and the reconstructed segments are checked against the reference points. The tests confirm that every algorithm returns at least one segment and that the positional error stays below `0.1` units.

