using System;
using System.Collections.Generic;
using System.Numerics;

namespace PHCurveLibrary.Fitting
{
    /// <summary>
    /// Implements local Hermite interpolation for curve fitting.
    /// </summary>
    public static class LocalHermiteFitter
    {
        /// <summary>
        /// Fit PH curves to a list of points using local Hermite interpolation.
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
        /// Incremental variant reading new points from a buffer.
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

                Vector3 origin = pts[start].Position;
                for (int end = start + 1; end < pts.Count; ++end)
                {
                    float segDuration = pts[end].Time - pts[start].Time;
                    HermiteControlPoint3D h0 = BuildHermitePoint(pts, ups, start, segDuration, origin);
                    HermiteControlPoint3D h1 = BuildHermitePoint(pts, ups, end, segDuration, origin);

                    PHCurve3D seg = PHCurveFactory.CreateQuintic(
                        h0,
                        h1,
                        pts[start].Time,
                        pts[end].Time);

                    float posErr = ComputeMaxDeviation(seg, pts, start, end, out float oriErr, origin);
                    float worst = MathF.Max(posErr / posTol, oriErr / MathF.Max(oriTol, 1e-6f));

                    if (worst <= 1f)
                    {
                        if (worst <= bestErr)
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

        private static PHCurve3D OptimizeG2(PHCurve3D previous, PHCurve3D next)
        {
            if (PHCurveFactory.ValidateG2(previous, next))
            {
                return next;
            }

            HermiteControlPoint3D start = new(
                Vector3.Zero,
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

        private static HermiteControlPoint3D BuildHermitePoint(
            List<PointData> pts,
            Vector3[] ups,
            int index,
            float scale,
            Vector3 origin)
        {
            Vector3 tangent;
            float dt;

            if (index < pts.Count - 1)
            {
                tangent = pts[index + 1].Position - pts[index].Position;
                dt = pts[index + 1].Time - pts[index].Time;
            }
            else
            {
                tangent = pts[index].Position - pts[index - 1].Position;
                dt = pts[index].Time - pts[index - 1].Time;
            }

            if (tangent.LengthSquared() < 1e-8f)
            {
                tangent = Vector3.UnitX;
                dt = 1f;
            }

            if (dt > 1e-6f)
            {
                tangent = (tangent / dt) * MathF.Abs(scale);
            }
            else
            {
                tangent = Vector3.Normalize(tangent) * MathF.Abs(scale);
            }

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

            return new HermiteControlPoint3D(pts[index].Position - origin, tangent, curvature, normal);
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
            out float maxOri,
            Vector3 origin)
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
                Vector3 pos = seg.Position(u) + origin;
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
    }
}
