using System;
using System.Linq;
using System.Numerics;
using MathNet.Numerics.Optimization;

namespace CubicPHCurve
{
    public class CubicPHCurve3DFitter
    {

        public static bool FitSingleSegmentPH3D(
            CubicPHCurve3D.ControlPoint[] cps,
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

            Vector3 startTan = start.Tangent;
            if (startTan == Vector3.Zero)
                startTan = (posS[1] - posS[0]) / (absTimes[1] - absTimes[0]);

            Vector3 endTan = end.Tangent;
            if (endTan == Vector3.Zero)
                endTan = (posS[N - 1] - posS[N - 2]) / (absTimes[N - 1] - absTimes[N - 2]);

            var cpH0 = new CubicPHCurve3D.ControlPoint(start.Position, startTan, start.Normal, start.Curvature);
            var cpH1 = new CubicPHCurve3D.ControlPoint(end.Position, endTan, end.Normal, end.Curvature);
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
}
