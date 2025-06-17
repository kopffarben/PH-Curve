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

            Vector3[] ups = PrepareUpVectors(pts);

            PHCurve3D? prevSeg = null;
            int start = 0;
            while (start < pts.Count - 1)
            {
                int bestEnd = start + 1;
                float bestErr = float.MaxValue;
                PHCurve3D bestSeg = default;

                for (int end = start + 1; end < pts.Count; ++end)
                {
                    HermiteControlPoint3D h0 = BuildHermitePoint(pts, ups, start);
                    HermiteControlPoint3D h1 = BuildHermitePoint(pts, ups, end);

                    PHCurve3D seg = PHCurveFactory.CreateQuintic(h0, h1, pts[start].Time, pts[end].Time);



                    float posErr = ComputeMaxDeviation(seg, pts, start, end, out float oriErr);
                    float worst = MathF.Max(posErr / posTol, oriErr / MathF.Max(oriTol, 1e-6f));

                    if (worst <= 1f)
                    {
                        if (worst < bestErr)
                        {
                            bestErr = worst;
                            bestSeg = seg;
                            bestEnd = end;
                        }
                    }
                    else
                    {
                        if (bestErr == float.MaxValue)
                        {
                            bestSeg = seg;
                            bestEnd = end;
                        }
                        break;
                    }
                }

                if (prevSeg.HasValue)
                {
                    bestSeg = OptimizeG2(prevSeg.Value, bestSeg);
                }

                result.Add(bestSeg);
                prevSeg = bestSeg;
                start = bestEnd;
            }

            return result;
        }

        private static List<PHCurve3D> FitLocalHermiteIncremental(List<PointData> buf, float posTol, float oriTol)
        {
            List<PHCurve3D> result = new();
            if (buf.Count < 2)
            {
                return result;
            }

            Vector3[] ups = PrepareUpVectors(buf);

            PHCurve3D? prevSeg = null;
            int start = 0;
            while (start < buf.Count - 1)
            {
                int bestEnd = start + 1;
                float bestErr = float.MaxValue;
                PHCurve3D bestSeg = default;

                for (int end = start + 1; end < buf.Count; ++end)
                {
                    HermiteControlPoint3D h0 = BuildHermitePoint(buf, ups, start);
                    HermiteControlPoint3D h1 = BuildHermitePoint(buf, ups, end);

                    PHCurve3D seg = PHCurveFactory.CreateQuintic(h0, h1, buf[start].Time, buf[end].Time);



                    float posErr = ComputeMaxDeviation(seg, buf, start, end, out float oriErr);
                    float worst = MathF.Max(posErr / posTol, oriErr / MathF.Max(oriTol, 1e-6f));

                    if (worst <= 1f)
                    {
                        if (worst < bestErr)
                        {
                            bestErr = worst;
                            bestSeg = seg;
                            bestEnd = end;
                        }
                    }
                    else
                    {
                        if (bestErr == float.MaxValue)
                        {
                            bestSeg = seg;
                            bestEnd = end;
                        }
                        break;
                    }
                }

                if (prevSeg.HasValue)
                {
                    bestSeg = OptimizeG2(prevSeg.Value, bestSeg);
                }

                result.Add(bestSeg);
                prevSeg = bestSeg;
                start = bestEnd;
                if (start >= buf.Count - 1)
                {
                    break;
                }
            }

            buf.RemoveRange(0, start);
            return result;
        }

        private static PHCurve3D OptimizeG2(PHCurve3D previous, PHCurve3D next)
        {
            if (PHCurveFactory.ValidateG2(previous, next))
            {
                return next;
            }

            HermiteControlPoint3D start = new(
                previous.Position(1f),
                previous.TangentUnit(1f),
                previous.Curvature(1f),
                previous.PrincipalNormal(1f));

            HermiteControlPoint3D end = new(
                next.Position(1f),
                next.TangentUnit(1f),
                next.Curvature(1f),
                next.PrincipalNormal(1f));

            return PHCurveFactory.CreateQuintic(start, end, next.StartTime, next.EndTime);
        }

        private static HermiteControlPoint3D BuildHermitePoint(List<PointData> pts, Vector3[] ups, int index)
        {
            Vector3 tangent;
            if (index < pts.Count - 1)
            {
                tangent = pts[index + 1].Position - pts[index].Position;
            }
            else
            {
                tangent = pts[index].Position - pts[index - 1].Position;
            }

            if (tangent.LengthSquared() < 1e-8f)
            {
                tangent = Vector3.UnitX;
            }

            tangent = Vector3.Normalize(tangent);

            float curvature = 0f;
            Vector3 up = ups[index];
            Vector3 normal = up - Vector3.Dot(up, tangent) * tangent;
            if (normal.LengthSquared() < 1e-6f)
            {
                normal = Vector3.Cross(tangent, Vector3.UnitY);
                if (normal.LengthSquared() < 1e-6f)
                {
                    normal = Vector3.Cross(tangent, Vector3.UnitZ);
                }
            }

            normal = Vector3.Normalize(normal);

            // Curvature is set to zero so that the normal aligns with the up vector.

            return new HermiteControlPoint3D(pts[index].Position, tangent, curvature, normal);
        }

        private static float EstimateCurvatureMagnitude(Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 ab = b - a;
            Vector3 bc = c - b;
            Vector3 ac = c - a;

            float lab = ab.Length();
            float lbc = bc.Length();
            float lac = ac.Length();

            if (lab < 1e-6f || lbc < 1e-6f || lac < 1e-6f)
            {
                return 0f;
            }

            float area2 = Vector3.Cross(ab, bc).Length();
            return 2f * area2 / (lab * lbc * lac);
        }

        private static Vector3[] PrepareUpVectors(List<PointData> pts)
        {
            Vector3[] ups = new Vector3[pts.Count];
            for (int i = 0; i < pts.Count; ++i)
            {
                Vector3 up = pts[i].UpVector.LengthSquared() > 1e-8f ? pts[i].UpVector : Vector3.UnitY;
                ups[i] = Vector3.Normalize(up);
            }

            float cosThreshold = MathF.Cos(0.5f);
            for (int i = 0; i < ups.Length - 1; ++i)
            {
                if (Vector3.Dot(ups[i], ups[i + 1]) < cosThreshold)
                {
                    Vector3 avg = Vector3.Normalize(ups[i] + ups[i + 1]);
                    ups[i] = avg;
                    ups[i + 1] = avg;
                }
            }

            return ups;
        }

        private static float ComputeMaxDeviation(PHCurve3D seg, List<PointData> pts, int startIdx, int endIdx, out float maxOri)
        {
            float maxPos = 0f;
            maxOri = 0f;

            float t0 = pts[startIdx].Time;
            float dt = pts[endIdx].Time - t0;
            if (dt < 1e-6f)
            {
                dt = 1f;
            }

            for (int i = startIdx; i <= endIdx; ++i)
            {
                float u = (pts[i].Time - t0) / dt;
                Vector3 pos = seg.Position(u);
                float d = Vector3.Distance(pos, pts[i].Position);
                if (d > maxPos)
                {
                    maxPos = d;
                }

                Vector3 up = pts[i].UpVector.LengthSquared() > 1e-8f ? Vector3.Normalize(pts[i].UpVector) : Vector3.UnitY;
                if (seg.Curvature(u) > 1e-5f)
                {
                    Vector3 n = seg.PrincipalNormal(u);
                    float ang = MathF.Acos(Math.Clamp(Vector3.Dot(n, up), -1f, 1f));
                    if (ang > maxOri)
                    {
                        maxOri = ang;
                    }
                }
            }

            return maxPos;
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
