// PathPlanner.cs
//
// References:
// Albrecht & Farouki (1996): Homotopy Methods for PH Splines
//
using System;
using System.Collections.Generic;
using System.Numerics;
using PHCurveLibrary.Fitting;

namespace PHCurveLibrary
{
    /// <summary>
    /// Builds a multi-segment path composed of <see cref="PHCurve3D"/> segments.
    /// Each segment is created from successive Hermite control points using
    /// <see cref="PHCurveFactory.CreateQuintic(PHCurveLibrary.HermiteControlPoint3D, PHCurveLibrary.HermiteControlPoint3D, float, float)"/>.
    /// </summary>
    public class PathPlanner
    {
        private readonly List<PHCurve3D> segments = new();

        /// <summary>
        /// Add a new curve segment defined by two Hermite control points.
        /// </summary>
        /// <param name="start">Start Hermite data.</param>
        /// <param name="end">End Hermite data.</param>
        public void AddSegment(HermiteControlPoint3D start, HermiteControlPoint3D end)
        {
            PHCurve3D curve = PHCurveFactory.CreateQuintic(start, end, 0f, 1f);
            segments.Add(curve);
        }

        /// <summary>
        /// Build the complete path consisting of all added segments.
        /// </summary>
        /// <returns>List of PH curve segments.</returns>
        public List<PHCurve3D> BuildPath()
        {
            return new List<PHCurve3D>(segments);
        }

        /// <summary>
        /// Validate <c>G²</c> continuity between successive segments of the path.
        /// </summary>
        /// <param name="tolerance">Comparison tolerance.</param>
        /// <returns><c>true</c> if all joins satisfy <c>G²</c> continuity.</returns>
        public bool ValidatePathG2(float tolerance = 1e-4f)
        {
            for (int i = 0; i < segments.Count - 1; i++)
            {
                if (!PHCurveFactory.ValidateG2(segments[i], segments[i + 1], tolerance))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Fit PH curves to a list of measured points using the specified algorithm.
        /// </summary>
        /// <param name="points">Ordered sample points with timestamps.</param>
        /// <param name="positionTolerance">Allowed positional deviation.</param>
        /// <param name="orientationTolerance">Allowed orientation deviation.</param>
        /// <param name="method">Fitting algorithm to use.</param>
        /// <returns>Generated segments with absolute start and end times.</returns>
        public static List<PHCurve3D> CurveFitting(
            List<PointData> points,
            float positionTolerance,
            float orientationTolerance,
            FittingMethod method)
        {
            return method switch
            {
                FittingMethod.LocalHermite => LocalHermiteFitter.Fit(points, positionTolerance, orientationTolerance),
                FittingMethod.LeastSquares => LeastSquaresFitter.Fit(points, positionTolerance, orientationTolerance),
                FittingMethod.Evolution => EvolutionFitter.Fit(points, positionTolerance, orientationTolerance),
                FittingMethod.Homotopy => HomotopyFitter.Fit(points, positionTolerance, orientationTolerance),
                FittingMethod.Heuristic => HeuristicFitter.Fit(points, positionTolerance, orientationTolerance),
                _ => throw new ArgumentOutOfRangeException(nameof(method))
            };
        }

        /// <summary>
        /// Incremental variant that generates segments from a buffer of new points.
        /// </summary>
        /// <param name="buffer">Buffer containing newly acquired points.</param>
        /// <param name="positionTolerance">Allowed positional deviation.</param>
        /// <param name="orientationTolerance">Allowed orientation deviation.</param>
        /// <param name="method">Fitting algorithm to use.</param>
        /// <returns>Newly created segments covering the buffer.</returns>
        public static List<PHCurve3D> CurveFittingIncremental(
            List<PointData> buffer,
            float positionTolerance,
            float orientationTolerance,
            FittingMethod method)
        {
            return method switch
            {
                FittingMethod.LocalHermite => LocalHermiteFitter.FitIncremental(buffer, positionTolerance, orientationTolerance),
                FittingMethod.LeastSquares => LeastSquaresFitter.FitIncremental(buffer, positionTolerance, orientationTolerance),
                FittingMethod.Evolution => EvolutionFitter.FitIncremental(buffer, positionTolerance, orientationTolerance),
                FittingMethod.Homotopy => HomotopyFitter.FitIncremental(buffer, positionTolerance, orientationTolerance),
                FittingMethod.Heuristic => HeuristicFitter.FitIncremental(buffer, positionTolerance, orientationTolerance),
                _ => throw new ArgumentOutOfRangeException(nameof(method))
            };
        }

    }
}
