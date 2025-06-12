using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Numerics;
using CubicPHCurve;
using MathNet.Numerics.Optimization;

namespace PH_Curve.Test
{
    [TestClass]
    public sealed class CubicPHCurve3DFitterTests
    {
        [TestMethod]
        public void FitSegmentProducesSmallErrors()
        {
            var cps = new[]
            {
                new CubicPHCurve3DFitter.ControlPointEx{Position=new Vector3(0,0,0), Time=0f, Normal=Vector3.UnitZ, Curvature=0f},
                new CubicPHCurve3DFitter.ControlPointEx{Position=new Vector3(1,1,0), Time=0.5f, Normal=Vector3.UnitZ, Curvature=0f},
                new CubicPHCurve3DFitter.ControlPointEx{Position=new Vector3(2,0,0), Time=1f, Normal=Vector3.UnitZ, Curvature=0f}
            };

            bool ok = CubicPHCurve3DFitter.FitSingleSegmentPH3D(cps, out var curve, out var posErr, out var normErr, out var T0, out var T1);
            System.Console.WriteLine($"posErr={posErr} normErr={normErr}");
            Assert.IsTrue(ok);
            Assert.IsTrue(posErr < 0.3f);
            Assert.IsTrue(normErr < 1.5f);
            Assert.AreEqual(0f, T0, 1e-6f);
            Assert.AreEqual(1f, T1, 1e-6f);

            // check velocity utility
            float midT = (T0 + T1) * 0.5f;
            Vector3 vel = PHCurveTimeUtils.VelocityAtTime(curve, midT, T0, T1);
            Vector3 deriv = curve.Derivative(0.5f) / (T1 - T0);
            Assert.IsTrue(Vector3.Distance(vel, deriv) < 1e-4f);
        }

        private static CubicPHCurve3D CreateSampleCurve()
        {
            Vector3 A = new(1f, 0f, 0f);
            Vector3 B = new(0f, 1f, 0f);
            Vector3 C = new(0f, 0f, 1f);
            Vector3 D = Vector3.Zero;
            Vector3 E = Vector3.Zero;
            return new CubicPHCurve3D(A, B, C, D, E);
        }

        private static float Curvature(Vector3 d1, Vector3 d2)
        {
            Vector3 cross = Vector3.Cross(d1, d2);
            float len = d1.Length();
            return cross.Length() / (len * len * len);
        }

        private static CubicPHCurve3DFitter.ControlPointEx[] SampleCurve(int count)
        {
            var curve = CreateSampleCurve();
            var cps = new CubicPHCurve3DFitter.ControlPointEx[count];
            for (int i = 0; i < count; ++i)
            {
                float t = (float)i / (count - 1);
                Vector3 d1 = curve.Derivative(t);
                Vector3 d2 = curve.SecondDerivative(t);
                cps[i] = new CubicPHCurve3DFitter.ControlPointEx
                {
                    Position = curve.Position(t),
                    Time = t,
                    Normal = curve.Normal(t),
                    Curvature = Curvature(d1, d2)
                };
            }
            return cps;
        }

        [TestMethod]
        public void FitSegmentOnSampleCurve()
        {
            int[] counts = { 2, 3, 4, 5 };
            foreach (int count in counts)
            {
                var cps = SampleCurve(count);

                if (count == 2)
                {
                    Assert.ThrowsException<MaximumIterationsException>(() =>
                        CubicPHCurve3DFitter.FitSingleSegmentPH3D(cps, out _, out _, out _, out _, out _));
                    continue;
                }

                bool ok = CubicPHCurve3DFitter.FitSingleSegmentPH3D(cps,
                    out var curve, out var posErr, out var normErr, out var T0, out var T1);

                System.Console.WriteLine($"count={count} posErr={posErr} normErr={normErr}");
                Assert.IsTrue(ok, $"fit failed for count {count}");
                Assert.IsTrue(posErr < 0.1f, $"posErr too large for count {count}");
                Assert.IsTrue(normErr < 1f, $"normErr too large for count {count}");
                Assert.AreEqual(0f, T0, 1e-6f);
                Assert.AreEqual(1f, T1, 1e-6f);
            }
        }
    }
}
