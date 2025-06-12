using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Numerics;
using CubicPHCurve;

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
                new CubicPHCurve3D.ControlPoint
                {
                    Position = new Vector3(0,0,0),
                    Tangent = new Vector3(2,2,0),
                    Normal = Vector3.UnitZ,
                    Curvature = 0f,
                    Time = 0f
                },
                new CubicPHCurve3D.ControlPoint
                {
                    Position = new Vector3(1,1,0),
                    Tangent = new Vector3(0,0,0),
                    Normal = Vector3.UnitZ,
                    Curvature = 0f,
                    Time = 0.5f
                },
                new CubicPHCurve3D.ControlPoint
                {
                    Position = new Vector3(2,0,0),
                    Tangent = new Vector3(2,-2,0),
                    Normal = Vector3.UnitZ,
                    Curvature = 0f,
                    Time = 1f
                }
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
            Vector3 vel = curve.VelocityAtTime(midT, T0, T1);
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


        [TestMethod]
        public void FitSegmentMatchesGivenTangents()
        {
            var reference = new CubicPHCurve3D(new Vector3(1f, 0f, 0f), new Vector3(0f, 1f, 0f), new Vector3(0f, 0f, 1f), Vector3.Zero, Vector3.Zero);

            Vector3 p0 = reference.Position(0f);
            Vector3 pMid = reference.Position(0.5f);
            Vector3 p1 = reference.Position(1f);
            Vector3 t0 = reference.Derivative(0f);
            Vector3 tMid = reference.Derivative(0.5f);
            Vector3 t1 = reference.Derivative(1f);
            Vector3 n0 = reference.Normal(0f);
            Vector3 nMid = reference.Normal(0.5f);
            Vector3 n1 = reference.Normal(1f);
            float k0 = Curvature(t0, reference.SecondDerivative(0f));
            float kMid = Curvature(tMid, reference.SecondDerivative(0.5f));
            float k1 = Curvature(t1, reference.SecondDerivative(1f));

            var cps = new[]
            {
                new CubicPHCurve3D.ControlPoint{Position=p0, Tangent=t0, Normal=n0, Curvature=k0, Time=0f},
                new CubicPHCurve3D.ControlPoint{Position=pMid, Tangent=tMid, Normal=nMid, Curvature=kMid, Time=0.5f},
                new CubicPHCurve3D.ControlPoint{Position=p1, Tangent=t1, Normal=n1, Curvature=k1, Time=1f}
            };

            bool ok = CubicPHCurve3DFitter.FitSingleSegmentPH3D(cps, out var curve, out var posErr, out var normErr, out var T0, out var T1);
            Assert.IsTrue(ok);
            Assert.IsTrue(posErr < 1e-5f);
            Assert.IsTrue(normErr < 1e-5f);
            Assert.AreEqual(0f, T0, 1e-6f);
            Assert.AreEqual(1f, T1, 1e-6f);
            Assert.IsTrue(Vector3.Distance(curve.Derivative(0f), t0) < 1e-6f);
            Assert.IsTrue(Vector3.Distance(curve.Derivative(1f), t1) < 1e-6f);

            float midT = (T0 + T1) * 0.5f;
            Vector3 vel = curve.VelocityAtTime(midT, T0, T1);
            Vector3 deriv = curve.Derivative(0.5f) / (T1 - T0);
            Assert.IsTrue(Vector3.Distance(vel, deriv) < 1e-4f);
        }

        private static CubicPHCurve3D.ControlPoint[] SampleCurve(int count)
        {
            var curve = CreateSampleCurve();
            var cps = new CubicPHCurve3D.ControlPoint[count];
            for (int i = 0; i < count; ++i)
            {
                float t = (float)i / (count - 1);
                Vector3 d1 = curve.Derivative(t);
                Vector3 d2 = curve.SecondDerivative(t);
                cps[i] = new CubicPHCurve3D.ControlPoint
                {
                    Position = curve.Position(t),
                    Tangent = d1,
                    Normal = curve.Normal(t),
                    Curvature = Curvature(d1, d2),
                    Time = t
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
