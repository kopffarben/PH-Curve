using System;
using System.Linq;
using System.Numerics;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;

namespace CubicPHCurve
{
    public class CubicPHCurve3DFitter
    {
        public struct ControlPointEx
        {
            public Vector3 Position;
            public float Time;
            public Vector3 Normal;
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

            Vector3 p0 = cps.First().Position;
            Vector3 p1 = cps.Last().Position;
            Vector3 u0 = Vector3.Normalize(cps[1].Position - cps[0].Position);
            Vector3 u1 = Vector3.Normalize(cps[^1].Position - cps[^2].Position);

            int N = cps.Length;
            Vector3[] posS = cps.Select(cp => cp.Position).ToArray();
            Vector3[] nrmS = cps.Select(cp => cp.Normal).ToArray();

            double Error(MathNet.Numerics.LinearAlgebra.Vector<double> vars)
            {
                float m0 = (float)vars[0];
                float m1 = (float)vars[1];

                var cpH0 = new CubicPHCurve3D.ControlPoint(p0, u0 * m0);
                var cpH1 = new CubicPHCurve3D.ControlPoint(p1, u1 * m1);
                var seg = CubicPHCurve3D.FromControlPoints(cpH0, cpH1);

                double sum = 0.0;
                for (int i = 0; i < N; ++i)
                {
                    Vector3 dp = seg.Position(times[i]) - posS[i];
                    sum += dp.LengthSquared();

                    Vector3 d1 = seg.Derivative(times[i]);
                    Vector3 T = Vector3.Normalize(d1);
                    Vector3 d2 = seg.SecondDerivative(times[i]);
                    Vector3 Nf = Vector3.Normalize(d2 - Vector3.Dot(d2, T) * T);
                    Vector3 dn = Nf - nrmS[i];
                    sum += dn.LengthSquared();
                }
                return sum;
            }

            double dist = Vector3.Distance(p0, p1);
            var initialGuess = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.DenseOfArray(new[] { dist, dist });
            MathNet.Numerics.LinearAlgebra.Vector<double> sol = FindMinimum.OfFunction(Error, initialGuess, 1e-6, 1000);

            float m0_fit = (float)sol[0];
            float m1_fit = (float)sol[1];

            var finalCP0 = new CubicPHCurve3D.ControlPoint(p0, u0 * m0_fit);
            var finalCP1 = new CubicPHCurve3D.ControlPoint(p1, u1 * m1_fit);
            fitted = CubicPHCurve3D.FromControlPoints(finalCP0, finalCP1);

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
