using System;
using System.Collections.Generic;
using System.Numerics;

namespace PHCurveLibrary.Fitting
{
    /// <summary>
    /// Implements heuristic segmentation with optional post smoothing.
    /// The method analyses the incoming points and splits the sequence
    /// whenever position or time gaps exceed a threshold. Each segment is
    /// then fitted using <see cref="LocalHermiteFitter"/> and consecutive
    /// segments are adjusted for <c>G²</c> continuity.
    /// </summary>
    public static class HeuristicFitter
    {
        /// <summary>
        /// Fit points using heuristic segmentation.
        /// </summary>
        public static List<PHCurve3D> Fit(
            List<PointData> points,
            float positionTolerance,
            float orientationTolerance)
        {
            return FitInternal(points, positionTolerance, orientationTolerance, false);
        }

        /// <summary>
        /// Incremental heuristic fitting.
        /// </summary>
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

            float distThr = posTol * 10f;
            // Use the average sampling period as a proxy for a time tolerance.
            float avgStep = (pts[^1].Time - pts[0].Time) / MathF.Max(1, pts.Count - 1);
            float timeThr = avgStep * 10f;
            float angleThr = oriTol * 5f;

            int start = 0;
            PHCurve3D? prevSeg = null;
            while (start < pts.Count - 1)
            {
                int end = start + 1;
                while (end < pts.Count - 1)
                {
                    float gap = Vector3.Distance(pts[end].Position, pts[end + 1].Position);
                    float tGap = pts[end + 1].Time - pts[end].Time;
                    Vector3 t0 = Vector3.Normalize(pts[end + 1].Position - pts[end].Position);
                    Vector3 t1 = Vector3.Normalize(pts[end].Position - pts[end - 1].Position);
                    float ang = MathF.Acos(Math.Clamp(Vector3.Dot(t0, t1), -1f, 1f));
                    if (gap > distThr || tGap > timeThr || ang > angleThr)
                    {
                        break;
                    }

                    end++;
                }

                float segDuration = pts[end].Time - pts[start].Time;
                Vector3 origin = pts[start].Position;
                HermiteControlPoint3D h0 = BuildHermitePoint(pts, ups, start, segDuration, origin);
                HermiteControlPoint3D h1 = BuildHermitePoint(pts, ups, end, segDuration, origin);
                PHCurve3D seg = PHCurveFactory.CreateQuintic(h0, h1, pts[start].Time, pts[end].Time);

                if (prevSeg.HasValue)
                {
                    seg = OptimizeG2(prevSeg.Value, seg);
                }

                result.Add(seg);
                prevSeg = seg;
                start = end;

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
    }
}
