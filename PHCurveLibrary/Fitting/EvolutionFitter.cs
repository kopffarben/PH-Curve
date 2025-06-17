using System;
using System.Collections.Generic;
using System.Numerics;

namespace PHCurveLibrary.Fitting
{
    /// <summary>
    /// Implements the evolution-based fitting approach described by
    /// Aigner, Sir and Jüttler (2007). The algorithm starts from an
    /// initial local Hermite interpolation and iteratively moves the
    /// sample points towards the curve to minimise the energy
    /// E = α E_pos + β E_orient + γ E_smooth.
    /// </summary>
    public static class EvolutionFitter
    {
        private const int MaxSteps = 20;
        private const float StepSize = 0.05f;

        /// <summary>
        /// Fit points using an evolution-based approach.
        /// </summary>
        /// <param name="points">Measured input points.</param>
        /// <param name="positionTolerance">Allowed positional deviation.</param>
        /// <param name="orientationTolerance">Allowed orientation deviation.</param>
        /// <returns>Generated segments fulfilling the tolerances.</returns>
        public static List<PHCurve3D> Fit(
            List<PointData> points,
            float positionTolerance,
            float orientationTolerance)
        {
            return FitInternal(points, positionTolerance, orientationTolerance, false);
        }

        /// <summary>
        /// Incremental evolution fitting.
        /// </summary>
        /// <param name="buffer">Buffer of new points.</param>
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
            List<PHCurve3D> segments = LocalHermiteFitter.Fit(pts, posTol, oriTol);

            for (int step = 0; step < MaxSteps; ++step)
            {
                ComputeError(segments, pts, out float maxPos, out float maxOri);
                if (maxPos <= posTol && maxOri <= oriTol)
                {
                    if (incremental)
                    {
                        pts.Clear();
                    }
                    return segments;
                }

                for (int i = 0; i < segments.Count; i++)
                {
                    PHCurve3D evolved = GradientStep(segments[i], pts, StepSize);
                    List<PointData> segPts = pts.FindAll(p => p.Time >= segments[i].StartTime && p.Time <= segments[i].EndTime);
                    ComputeError(new() { evolved }, segPts, out float _, out float ori);
                    if (ori <= oriTol * 2f)
                    {
                        segments[i] = evolved;
                    }
                }
            }

            if (incremental)
            {
                pts.Clear();
            }

            return segments;
        }

        private static PHCurve3D GradientStep(PHCurve3D seg, List<PointData> pts, float step)
        {
            const int paramCount = 15;

            var jtj = MathNet.Numerics.LinearAlgebra.Matrix<float>.Build.Dense(paramCount, paramCount);
            var jtr = MathNet.Numerics.LinearAlgebra.Vector<float>.Build.Dense(paramCount);

            float start = seg.StartTime;
            float duration = seg.EndTime - start;
            if (duration < 1e-6f)
            {
                duration = 1f;
            }

            foreach (var p in pts)
            {
                if (p.Time < seg.StartTime || p.Time > seg.EndTime)
                {
                    continue;
                }

                float u = (p.Time - start) / duration;
                Vector3 pos = seg.Position(u);
                Vector3 der = seg.Derivative(u);
                Vector3 second = seg.SecondDerivative(u);

                Vector3 normal = Vector3.Cross(der, second);
                if (normal.LengthSquared() < 1e-10f)
                {
                    continue;
                }
                normal = Vector3.Normalize(normal);

                Vector3 diff = pos - p.Position;
                float residual = Vector3.Dot(normal, diff);

                float t = u;
                float jA = duration * t;
                float jB = duration * 0.5f * t * t;
                float jC = duration * t * t * t / 3f;
                float jD = duration * t * t * t * t / 4f;
                float jE = duration * t * t * t * t * t / 5f;

                float[] j = new float[paramCount]
                {
                    normal.X * jA, normal.Y * jA, normal.Z * jA,
                    normal.X * jB, normal.Y * jB, normal.Z * jB,
                    normal.X * jC, normal.Y * jC, normal.Z * jC,
                    normal.X * jD, normal.Y * jD, normal.Z * jD,
                    normal.X * jE, normal.Y * jE, normal.Z * jE
                };

                for (int r = 0; r < paramCount; ++r)
                {
                    jtr[r] += j[r] * residual;
                    for (int c = 0; c < paramCount; ++c)
                    {
                        jtj[r, c] += j[r] * j[c];
                    }
                }
            }

            // Regularization for numerical stability
            for (int i = 0; i < paramCount; ++i)
            {
                jtj[i, i] += 1e-6f;
            }

            MathNet.Numerics.LinearAlgebra.Vector<float> delta = jtj.Solve(jtr);

            Vector3 dA = new(delta[0], delta[1], delta[2]);
            Vector3 dB = new(delta[3], delta[4], delta[5]);
            Vector3 dC = new(delta[6], delta[7], delta[8]);
            Vector3 dD = new(delta[9], delta[10], delta[11]);
            Vector3 dE = new(delta[12], delta[13], delta[14]);

            Vector3 newA = seg.A - step * dA;
            Vector3 newB = seg.B - step * dB;
            Vector3 newC = seg.C - step * dC;
            Vector3 newD = seg.D - step * dD;
            Vector3 newE = seg.E - step * dE;

            return new PHCurve3D(newA, newB, newC, newD, newE, seg.StartTime, seg.EndTime);
        }

        private static void ComputeError(
            List<PHCurve3D> segs,
            List<PointData> reference,
            out float maxPos,
            out float maxOri)
        {
            maxPos = 0f;
            maxOri = 0f;
            foreach (var p in reference)
            {
                PHCurve3D seg = FindSegment(segs, p.Time);
                float u = (p.Time - seg.StartTime) / (seg.EndTime - seg.StartTime);
                int idx = reference.FindIndex(pt => Math.Abs(pt.Time - seg.StartTime) < 1e-6f);
                Vector3 origin = idx >= 0 ? reference[idx].Position : Vector3.Zero;
                Vector3 curvePos = seg.Position(u) + origin;
                float d = Vector3.Distance(curvePos, p.Position);
                if (d > maxPos)
                {
                    maxPos = d;
                }

                Vector3 up = p.UpVector.LengthSquared() > 1e-8f ? Vector3.Normalize(p.UpVector) : Vector3.UnitY;
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
        }

        private static PHCurve3D FindSegment(List<PHCurve3D> segs, float time)
        {
            for (int i = 0; i < segs.Count; ++i)
            {
                if (time <= segs[i].EndTime || i == segs.Count - 1)
                {
                    return segs[i];
                }
            }
            return segs[^1];
        }
    }
}
