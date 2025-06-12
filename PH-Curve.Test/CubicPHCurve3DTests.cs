using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Numerics;
using CubicPHCurve;

namespace PH_Curve.Test
{
    [TestClass]
    public sealed class CubicPHCurve3DTests
    {
        private static void AssertVector(Vector3 expected, Vector3 actual, float tolerance, string message)
        {
            Assert.IsTrue(Vector3.Distance(expected, actual) < tolerance, message + $" Expected: {expected} Actual: {actual}");
        }

        private static CubicPHCurve3D CreateSampleCurve()
        {
            // r'(t) = (1,0,0) + (0,1,0)t + (0,0,1)t^2
            Vector3 A = new(1f, 0f, 0f);
            Vector3 B = new(0f, 1f, 0f);
            Vector3 C = new(0f, 0f, 1f);
            Vector3 D = Vector3.Zero;
            Vector3 E = Vector3.Zero;
            return new CubicPHCurve3D(A, B, C, D, E);
        }

        [TestMethod]
        public void DerivativeAndPositionAreConsistent()
        {
            var curve = CreateSampleCurve();
            Vector3 deriv = curve.Derivative(0.5f);
            AssertVector(new Vector3(1f, 0.5f, 0.25f), deriv, 1e-6f, "Derivative");

            Vector3 pos = curve.Position(0.5f);
            AssertVector(new Vector3(0.5f, 0.125f, 0.04166667f), pos, 1e-6f, "Position");
        }

        private static float Curvature(Vector3 d1, Vector3 d2)
        {
            Vector3 cross = Vector3.Cross(d1, d2);
            float len = d1.Length();
            return cross.Length() / (len * len * len);
        }

        [TestMethod]
        public void FromControlPointsReconstructsCurve()
        {
            var original = CreateSampleCurve();
            Vector3 p0 = original.Position(0f);
            Vector3 p1 = original.Position(1f);
            Vector3 t0 = original.Derivative(0f);
            Vector3 t1 = original.Derivative(1f);
            Vector3 n0 = original.Normal(0f);
            Vector3 n1 = original.Normal(1f);
            float k0 = Curvature(t0, original.SecondDerivative(0f));
            float k1 = Curvature(t1, original.SecondDerivative(1f));

            var cp0 = new CubicPHCurve3D.ControlPoint(p0, t0, n0, k0);
            var cp1 = new CubicPHCurve3D.ControlPoint(p1, t1, n1, k1);
            var rebuilt = CubicPHCurve3D.FromControlPoints(cp0, cp1);

            Vector3 rebuilt_p0 = rebuilt.Position(0f);
            Vector3 rebuilt_p1 = rebuilt.Position(1f);
            Vector3 rebuilt_t0 = rebuilt.Derivative(0f);
            Vector3 rebuilt_t1 = rebuilt.Derivative(1f);
            Vector3 rebuilt_n0 = rebuilt.Normal(0f);
            Vector3 rebuilt_n1 = rebuilt.Normal(1f);
            float rebuilt_k0 = Curvature(rebuilt_t0, original.SecondDerivative(0f));
            float rebuilt_k1 = Curvature(rebuilt_t1, original.SecondDerivative(1f));

            System.Console.WriteLine($"original.Position(0f)={p0} rebuilt.Position(0f)={rebuilt_p0}");
            System.Console.WriteLine($"original.Position(1f)={p1} rebuilt.Position(1f)={rebuilt_p1}");
            System.Console.WriteLine($"original.Derivative(0f)={t0} rebuilt.Derivative(0f)={rebuilt_t0}");
            System.Console.WriteLine($"original.Derivative(1f)={t1} rebuilt.Derivative(1f)={rebuilt_t1}");
            System.Console.WriteLine($"original.Normal(0f)={n0} rebuilt.Normal(0f)={rebuilt_n0}");
            System.Console.WriteLine($"original.Normal(1f)={n1} rebuilt.Normal(1f)={rebuilt_n1}");
            System.Console.WriteLine($"original.Curvature(0f)={k0} rebuilt.Curvature(0f)={rebuilt_k0}");
            System.Console.WriteLine($"original.Curvature(1f)={k1} rebuilt.Curvature(1f)={rebuilt_k1}");

            AssertVector(p0, rebuilt_p0, 1e-5f, "Position at 0");
            AssertVector(p1, rebuilt_p1, 1e-5f, "Position at 1");
            AssertVector(t0, rebuilt_t0, 1e-5f, "Derivative at 0");
            AssertVector(t1, rebuilt_t1, 1e-5f, "Derivative at 1");
            AssertVector(n0, rebuilt_n0, 1e-5f, "Normal at 0");
            AssertVector(n1, rebuilt_n1, 1e-5f, "Normal at 1");
            Assert.AreEqual(k0, rebuilt_k0, 1e-5f, "Curvature at 0");
            Assert.AreEqual(k1, rebuilt_k1, 1e-5f, "Curvature at 1");

        }

        [TestMethod]
        public void GeometricQuantitiesAreComputedCorrectly()
        {
            var curve = CreateSampleCurve();
            float t = 0f;

            float speed = curve.Speed(t);
            Assert.AreEqual(1f, speed, 1e-6f, "Speed at t=0");

            Vector3 tangent = curve.Tangent(t);
            AssertVector(Vector3.UnitX, tangent, 1e-6f, "Tangent at t=0");

            Vector3 d2 = curve.SecondDerivative(t);
            AssertVector(Vector3.UnitY, d2, 1e-6f, "Second derivative at t=0");

            Vector3 normal = curve.Normal(t);
            AssertVector(Vector3.UnitY, normal, 1e-6f, "Normal at t=0");

            Vector3 bitan = curve.BiTangent(t);
            AssertVector(Vector3.UnitZ, bitan, 1e-6f, "BiTangent at t=0");

            Vector3 offset = curve.OffsetPoint(t, 1f);
            AssertVector(new Vector3(0f, 1f, 0f), offset, 1e-6f, "Offset point");

            float curvature = curve.Curvature(t);
            Assert.AreEqual(1f, curvature, 1e-6f, "Curvature at t=0");

            Vector3 vel = curve.VelocityAtTime(t, 0f, 1f);
            AssertVector(curve.Derivative(t), vel, 1e-6f, "Velocity at t=0");

            float speedAbs = curve.SpeedAtTime(t, 0f, 1f);
            Assert.AreEqual(speed, speedAbs, 1e-6f, "Speed at absolute time");
        }
    }
}
