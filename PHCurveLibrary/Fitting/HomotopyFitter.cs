using System;
using System.Collections.Generic;
using System.Numerics;

namespace PHCurveLibrary.Fitting
{
    /// <summary>
    /// Fits measured points by deforming an initial curve towards the final
    /// <c>G²</c> solution using a simple homotopy continuation strategy.
    /// The implementation follows the ideas of Albrecht &amp; Farouki
    /// (1996) where a predictor–corrector traces a continuous family of
    /// PH curves.
    /// </summary>
    public static class HomotopyFitter
    {
        /// <summary>
        /// Fit PH curves to a list of points using homotopy continuation.
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
            List<PHCurve3D> segs = FitInternal(points, positionTolerance, orientationTolerance, false);
            if (segs.Count == 0)
            {
                // Fallback to local Hermite interpolation if homotopy fails.
                segs = LocalHermiteFitter.Fit(points, positionTolerance, orientationTolerance);
            }

            return segs;
        }

        /// <summary>
        /// Incremental variant that reuses the current homotopy state.
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
            List<PHCurve3D> segs = FitInternal(buffer, positionTolerance, orientationTolerance, true);
            if (segs.Count == 0)
            {
                segs = LocalHermiteFitter.FitIncremental(buffer, positionTolerance, orientationTolerance);
            }

            return segs;
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
                int end = start + 1;
                PHCurve3D? seg = null;
                while (end < pts.Count && seg == null)
                {
                    seg = TryHomotopySegment(pts, ups, start, end, posTol, oriTol);
                    if (seg == null)
                    {
                        end++;
                    }
                }

                if (seg == null)
                {
                    break;
                }

                PHCurve3D useSeg = seg.Value;
                if (prevSeg.HasValue)
                {
                    useSeg = OptimizeG2(prevSeg.Value, useSeg);
                }

                result.Add(useSeg);
                prevSeg = useSeg;
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

        private static PHCurve3D? TryHomotopySegment(
            List<PointData> pts,
            Vector3[] ups,
            int start,
            int end,
            float posTol,
            float oriTol)
        {
            float segDuration = pts[end].Time - pts[start].Time;
            Vector3 origin = pts[start].Position;
            HermiteControlPoint3D finalStart = BuildHermitePoint(pts, ups, start, segDuration, origin);
            HermiteControlPoint3D finalEnd = BuildHermitePoint(pts, ups, end, segDuration, origin);

            Vector3 baseTangent = Vector3.Normalize(pts[end].Position - pts[start].Position);
            HermiteControlPoint3D baseStart = new(
                pts[start].Position,
                baseTangent,
                0f,
                Vector3.Normalize(ups[start] - Vector3.Dot(ups[start], baseTangent) * baseTangent));
            HermiteControlPoint3D baseEnd = new(
                pts[end].Position,
                baseTangent,
                0f,
                Vector3.Normalize(ups[end] - Vector3.Dot(ups[end], baseTangent) * baseTangent));

            float t0 = pts[start].Time;
            float t1 = pts[end].Time;

            float lambda = 0f;
            float step = 0.25f;
            PHCurve3D seg = PHCurveFactory.CreateQuintic(baseStart, baseEnd, t0, t1);

            for (int iter = 0; iter < 30 && lambda < 1f; iter++)
            {
                float target = MathF.Min(1f, lambda + step);
                HermiteControlPoint3D s = Interpolate(baseStart, finalStart, target);
                HermiteControlPoint3D e = Interpolate(baseEnd, finalEnd, target);
                PHCurve3D candidate = PHCurveFactory.CreateQuintic(s, e, t0, t1);
                float posErr = ComputeMaxDeviation(candidate, pts, start, end, out float oriErr, origin);

                if (posErr <= posTol && oriErr <= oriTol)
                {
                    seg = candidate;
                    lambda = target;
                    step = MathF.Min(step * 1.5f, 0.5f);
                }
                else
                {
                    float hi = target;
                    float lo = lambda;
                    for (int j = 0; j < 4 && hi - lo > 1e-3f; j++)
                    {
                        float mid = 0.5f * (lo + hi);
                        HermiteControlPoint3D ms = Interpolate(baseStart, finalStart, mid);
                        HermiteControlPoint3D me = Interpolate(baseEnd, finalEnd, mid);
                        PHCurve3D test = PHCurveFactory.CreateQuintic(ms, me, t0, t1);
                        float pErr = ComputeMaxDeviation(test, pts, start, end, out float oErr, origin);
                        if (pErr <= posTol && oErr <= oriTol)
                        {
                            seg = test;
                            lo = mid;
                        }
                        else
                        {
                            hi = mid;
                        }
                    }

                    lambda = lo;
                    step *= 0.5f;

                    if (step < 0.01f)
                    {
                        break;
                    }
                }
            }

            float finalPos = ComputeMaxDeviation(seg, pts, start, end, out float finalOri, origin);
            if (finalPos <= posTol && finalOri <= oriTol)
            {
                return seg;
            }

            return null;
        }

        private static HermiteControlPoint3D Interpolate(
            in HermiteControlPoint3D a,
            in HermiteControlPoint3D b,
            float t)
        {
            Vector3 tangent = Vector3.Normalize(Vector3.Lerp(a.Tangent, b.Tangent, t));
            float curvature = a.Curvature * (1f - t) + b.Curvature * t;
            Vector3 normal = Vector3.Normalize(Vector3.Lerp(a.PrincipalNormal, b.PrincipalNormal, t));
            return new HermiteControlPoint3D(b.Position, tangent, curvature, normal);
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

            int samples = Math.Max(5, endIdx - startIdx + 1);
            for (int i = 0; i <= samples; ++i)
            {
                float u = i / (float)samples;
                float time = t0 + u * dt;

                // nearest sample point for position comparison
                int idx = startIdx;
                while (idx < endIdx - 1 && pts[idx + 1].Time < time)
                {
                    idx++;
                }
                Vector3 expected = pts[idx].Position;

                Vector3 pos = seg.Position(u) + origin;
                float d = Vector3.Distance(pos, expected);
                if (d > maxPos)
                {
                    maxPos = d;
                }

                Vector3 up = pts[idx].UpVector.LengthSquared() > 1e-8f ? Vector3.Normalize(pts[idx].UpVector) : Vector3.UnitY;
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
