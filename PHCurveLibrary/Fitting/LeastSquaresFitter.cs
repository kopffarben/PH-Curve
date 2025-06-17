using System;
using System.Collections.Generic;
using System.Numerics;

namespace PHCurveLibrary.Fitting
{
    /// <summary>
    /// Implements a simple least-squares approximation by
    /// estimating tangent lengths and endpoint curvatures
    /// from the sample points. The resulting Hermite data
    /// is passed to <see cref="PHCurveFactory.CreateQuintic"/>.
    /// </summary>
    public static class LeastSquaresFitter
    {
        /// <summary>
        /// Fit points using a basic least-squares approach.
        /// </summary>
        /// <param name="points">Ordered sample points.</param>
        /// <param name="positionTolerance">Allowed positional deviation.</param>
        /// <param name="orientationTolerance">Allowed orientation deviation.</param>
        /// <returns>Generated segments.</returns>
        public static List<PHCurve3D> Fit(
            List<PointData> points,
            float positionTolerance,
            float orientationTolerance)
        {
            return FitInternal(points, positionTolerance, orientationTolerance, false);
        }

        /// <summary>
        /// Incremental fitting on a growing buffer of points.
        /// </summary>
        /// <param name="buffer">Buffer of new sample points.</param>
        /// <param name="positionTolerance">Allowed positional deviation.</param>
        /// <param name="orientationTolerance">Allowed orientation deviation.</param>
        /// <returns>Generated segments covering the buffer.</returns>
        public static List<PHCurve3D> FitIncremental(
            List<PointData> buffer,
            float positionTolerance,
            float orientationTolerance)
        {
            return FitInternal(buffer, positionTolerance, orientationTolerance, true);
        }

        private static List<PHCurve3D> FitInternal(
            List<PointData> pts,
            float posTol,
            float oriTol,
            bool incremental)
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
                    PHCurve3D seg = BuildSegment(pts, ups, start, end);

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
                if (incremental && start >= pts.Count - 1)
                {
                    break;
                }
            }

            if (incremental)
            {
                pts.RemoveRange(0, start);
            }

            return result;
        }

        private static PHCurve3D BuildSegment(
            List<PointData> pts,
            Vector3[] ups,
            int start,
            int end)
        {
            Vector3 dir0 = Vector3.Normalize(TangentDirection(pts, start, true));
            Vector3 dir1 = Vector3.Normalize(TangentDirection(pts, end, false));

            float len0 = Vector3.Distance(pts[start + 1].Position, pts[start].Position);
            float len1 = Vector3.Distance(pts[end].Position, pts[end - 1].Position);
            if (len0 < 1e-3f) len0 = 1f;
            if (len1 < 1e-3f) len1 = 1f;

            float curv0 = EstimateCurvature(pts, ups, start);
            float curv1 = EstimateCurvature(pts, ups, end);

            Vector3 n0 = ComputeNormal(dir0, ups[start]);
            Vector3 n1 = ComputeNormal(dir1, ups[end]);

            HermiteControlPoint3D h0 = new(pts[start].Position, dir0 * len0, curv0, n0);
            HermiteControlPoint3D h1 = new(pts[end].Position, dir1 * len1, curv1, n1);

            return PHCurveFactory.CreateQuintic(h0, h1, pts[start].Time, pts[end].Time);
        }

        private static float EstimateCurvature(List<PointData> pts, Vector3[] ups, int index)
        {
            int prev = Math.Max(index - 1, 0);
            int next = Math.Min(index + 1, pts.Count - 1);
            if (prev == index || next == index)
            {
                return 0f;
            }

            Vector3 a = pts[prev].Position;
            Vector3 b = pts[index].Position;
            Vector3 c = pts[next].Position;

            Vector3 ab = b - a;
            Vector3 bc = c - b;
            float lenAb = ab.Length();
            float lenBc = bc.Length();
            if (lenAb < 1e-6f || lenBc < 1e-6f)
            {
                return 0f;
            }

            Vector3 cross = Vector3.Cross(ab, bc);
            float area2 = cross.Length();
            float chord = Vector3.Distance(c, a);
            if (chord < 1e-6f)
            {
                return 0f;
            }

            float curvature = 2f * area2 / (lenAb * lenBc * chord);
            float sign = MathF.Sign(Vector3.Dot(cross, ups[index]));
            return curvature * sign;
        }

        private static Vector3 TangentDirection(List<PointData> pts, int index, bool forward)
        {
            if (forward)
            {
                if (index < pts.Count - 1)
                {
                    return pts[index + 1].Position - pts[index].Position;
                }
                else
                {
                    return pts[index].Position - pts[index - 1].Position;
                }
            }
            else
            {
                if (index > 0)
                {
                    return pts[index].Position - pts[index - 1].Position;
                }
                else
                {
                    return pts[1].Position - pts[0].Position;
                }
            }
        }

        private static Vector3 ComputeNormal(Vector3 tangent, Vector3 up)
        {
            Vector3 n = up - Vector3.Dot(up, tangent) * tangent;
            if (n.LengthSquared() < 1e-6f)
            {
                n = Vector3.Cross(tangent, Vector3.UnitY);
                if (n.LengthSquared() < 1e-6f)
                {
                    n = Vector3.Cross(tangent, Vector3.UnitZ);
                }
            }

            return Vector3.Normalize(n);
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

        private static float ComputeMaxDeviation(
            PHCurve3D seg,
            List<PointData> pts,
            int startIdx,
            int endIdx,
            out float maxOri)
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
    }
}
