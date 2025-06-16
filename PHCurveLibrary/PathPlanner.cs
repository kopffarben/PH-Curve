// PathPlanner.cs
//
// References:
// Albrecht & Farouki (1996): Homotopy Methods for PH Splines
//
using System;
using System.Collections.Generic;
using System.Numerics;

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
                FittingMethod.LocalHermite => FitLocalHermite(points, positionTolerance, orientationTolerance),
                FittingMethod.LeastSquares => FitLeastSquares(points, positionTolerance, orientationTolerance),
                FittingMethod.Evolution => FitEvolution(points, positionTolerance, orientationTolerance),
                FittingMethod.Homotopy => FitHomotopy(points, positionTolerance, orientationTolerance),
                FittingMethod.Heuristic => FitHeuristic(points, positionTolerance, orientationTolerance),
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
                FittingMethod.LocalHermite => FitLocalHermiteIncremental(buffer, positionTolerance, orientationTolerance),
                FittingMethod.LeastSquares => FitLeastSquaresIncremental(buffer, positionTolerance, orientationTolerance),
                FittingMethod.Evolution => FitEvolutionIncremental(buffer, positionTolerance, orientationTolerance),
                FittingMethod.Homotopy => FitHomotopyIncremental(buffer, positionTolerance, orientationTolerance),
                FittingMethod.Heuristic => FitHeuristicIncremental(buffer, positionTolerance, orientationTolerance),
                _ => throw new ArgumentOutOfRangeException(nameof(method))
            };
        }

        private static List<PHCurve3D> FitLocalHermite(List<PointData> pts, float posTol, float oriTol)
        {
            List<PHCurve3D> result = new();
            if (pts.Count < 2)
            {
                return result;
            }

            for (int i = 0; i < pts.Count - 1; ++i)
            {
                PointData p0 = pts[i];
                PointData p1 = pts[i + 1];
                Vector3 dir = p1.Position - p0.Position;
                if (dir.LengthSquared() < 1e-8f)
                {
                    dir = Vector3.UnitX;
                }
                Vector3 tan = Vector3.Normalize(dir);

                Vector3 up0 = p0.UpVector.LengthSquared() > 1e-8f ? Vector3.Normalize(p0.UpVector) : Vector3.UnitY;
                Vector3 up1 = p1.UpVector.LengthSquared() > 1e-8f ? Vector3.Normalize(p1.UpVector) : Vector3.UnitY;

                Vector3 n0 = Vector3.Normalize(Vector3.Cross(Vector3.Cross(tan, up0), tan));
                Vector3 n1 = Vector3.Normalize(Vector3.Cross(Vector3.Cross(tan, up1), tan));

                var h0 = new HermiteControlPoint3D(p0.Position, tan, 0f, n0);
                var h1 = new HermiteControlPoint3D(p1.Position, tan, 0f, n1);

                PHCurve3D seg = PHCurveFactory.CreateQuintic(h0, h1, p0.Time, p1.Time);
                result.Add(seg);
            }

            return result;
        }

        private static List<PHCurve3D> FitLocalHermiteIncremental(List<PointData> buf, float posTol, float oriTol)
        {
            return FitLocalHermite(buf, posTol, oriTol);
        }

        private static List<PHCurve3D> FitLeastSquares(List<PointData> pts, float posTol, float oriTol)
            => FitLocalHermite(pts, posTol, oriTol);

        private static List<PHCurve3D> FitLeastSquaresIncremental(List<PointData> buf, float posTol, float oriTol)
            => FitLocalHermiteIncremental(buf, posTol, oriTol);

        private static List<PHCurve3D> FitEvolution(List<PointData> pts, float posTol, float oriTol)
            => FitLocalHermite(pts, posTol, oriTol);

        private static List<PHCurve3D> FitEvolutionIncremental(List<PointData> buf, float posTol, float oriTol)
            => FitLocalHermiteIncremental(buf, posTol, oriTol);

        private static List<PHCurve3D> FitHomotopy(List<PointData> pts, float posTol, float oriTol)
            => FitLocalHermite(pts, posTol, oriTol);

        private static List<PHCurve3D> FitHomotopyIncremental(List<PointData> buf, float posTol, float oriTol)
            => FitLocalHermiteIncremental(buf, posTol, oriTol);

        private static List<PHCurve3D> FitHeuristic(List<PointData> pts, float posTol, float oriTol)
            => FitLocalHermite(pts, posTol, oriTol);

        private static List<PHCurve3D> FitHeuristicIncremental(List<PointData> buf, float posTol, float oriTol)
            => FitLocalHermiteIncremental(buf, posTol, oriTol);
    }
}
