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

        [TestMethod]
        public void FromControlPointsReconstructsCurve()
        {
            var original = CreateSampleCurve();
            Vector3 p0 = original.Position(0f);
            Vector3 p1 = original.Position(1f);
            Vector3 t0 = original.Derivative(0f);
            Vector3 t1 = original.Derivative(1f);

            var cp0 = new CubicPHCurve3D.ControlPoint(p0, t0);
            var cp1 = new CubicPHCurve3D.ControlPoint(p1, t1);
            var rebuilt = CubicPHCurve3D.FromControlPoints(cp0, cp1);

            AssertVector(t0, rebuilt.Derivative(0f), 1e-5f, "Derivative at 0");
            AssertVector(t1, rebuilt.Derivative(1f), 1e-5f, "Derivative at 1");
            AssertVector(p1, rebuilt.Position(1f), 1e-5f, "End position");
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
        }
    }
}
