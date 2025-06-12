using System;
using System.Linq;
using System.Numerics;

namespace CubicPHCurve
{
    public class CubicPHCurve3DFitter
    {
        /// <summary>
        /// Control point used for fitting including time and normal information.
        /// </summary>
        public struct ControlPointEx
        {
            /// <summary>Spatial position of the point.</summary>
            public Vector3 Position;
            /// <summary>Tangent vector at the point.</summary>
            public Vector3 Tangent;
            /// <summary>Absolute time parameter of the point.</summary>
            public float Time;
            /// <summary>Normal vector at the point.</summary>
            public Vector3 Normal;
            /// <summary>Curvature at the point.</summary>
            public float Curvature;
        }

        public static bool FitSingleSegmentPH3D(
            ControlPointEx[] cps,
            out CubicPHCurve3D fitted,
            out float rmsPosError,
            out float rmsNormError,
            out float T0,
            out float T1)
        {
            if (cps == null || cps.Length < 2)
                throw new ArgumentException("At least two control points are required.");

            float[] absTimes = cps.Select(cp => cp.Time).ToArray();
            float t0 = absTimes.Min();
            float t1 = absTimes.Max();
            T0 = t0;
            T1 = t1;
            float duration = t1 - t0;
            float[] times = absTimes.Select(t => (t - t0) / duration).ToArray();

            int N = cps.Length;
            Vector3[] posS = cps.Select(cp => cp.Position).ToArray();
            Vector3[] nrmS = cps.Select(cp => cp.Normal).ToArray();

            var start = cps.First();
            var end = cps[^1];
            var cpH0 = new CubicPHCurve3D.ControlPoint(start.Position, start.Tangent, start.Normal, start.Curvature);
            var cpH1 = new CubicPHCurve3D.ControlPoint(end.Position, end.Tangent, end.Normal, end.Curvature);
            fitted = CubicPHCurve3D.FromControlPoints(cpH0, cpH1);

            double sumPos2 = 0, sumNorm2 = 0;
            for (int i = 0; i < N; ++i)
            {
                var dp = fitted.Position(times[i]) - posS[i];
                sumPos2 += dp.LengthSquared();

                Vector3 d1 = fitted.Derivative(times[i]);
                Vector3 T = Vector3.Normalize(d1);
                Vector3 d2 = fitted.SecondDerivative(times[i]);
                Vector3 Nf = Vector3.Normalize(d2 - Vector3.Dot(d2, T) * T);
                var dn = Nf - nrmS[i];
                sumNorm2 += dn.LengthSquared();
            }
            rmsPosError = (float)Math.Sqrt(sumPos2 / N);
            rmsNormError = (float)Math.Sqrt(sumNorm2 / N);

            return true;
        }
    }

    public static class PHCurveTimeUtils
    {
        public static Vector3 VelocityAtTime(CubicPHCurve3D curve, float T, float T0, float T1)
        {
            float tParam = (T - T0) / (T1 - T0);
            Vector3 dr_dtp = curve.Derivative(tParam);
            float invDur = 1.0f / (T1 - T0);
            return dr_dtp * invDur;
        }

        public static float SpeedAtTime(CubicPHCurve3D curve, float T, float T0, float T1)
        {
            return VelocityAtTime(curve, T, T0, T1).Length();
        }
    }
}
