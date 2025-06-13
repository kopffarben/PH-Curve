using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Numerics;
using PHCurveLibrary;

namespace PHCurveLibrary.Tests
{
    /// <summary>
    /// Unit tests for <see cref="PHCurve3D"/> focusing on mathematical correctness.
    /// The chosen coefficients correspond to the parabola r(t) = (t, 0.5 t^2, 0)
    /// with hodograph r'(t) = (1, t, 0).
    /// </summary>
    [TestClass]
    public sealed class PHCurve3DTests
    {
        private static readonly Vector3 A = new(1f, 0f, 0f);
        private static readonly Vector3 B = new(0f, 1f, 0f);
        private static readonly Vector3 C = Vector3.Zero;
        private static readonly Vector3 D = Vector3.Zero;
        private static readonly Vector3 E = Vector3.Zero;

        private static void AssertVector(Vector3 expected, Vector3 actual, float tolerance, string message)
        {
            Assert.IsTrue(Vector3.Distance(expected, actual) < tolerance, message + $" Expected: {expected} Actual: {actual}");
        }

        private static PHCurve3D CreateParabola()
        {
            return new PHCurve3D(A, B, C, D, E);
        }

        /// <summary>
        /// Create a quintic curve with non-zero C, D and E coefficients.  The
        /// derivative is r'(t) = (1 + t³, t + t⁴, t²) resulting in the
        /// position r(t) = (t + t⁴/4,
        ///                    0.5 t² + t⁵/5,
        ///                    t³/3).
        /// This example validates that all analytic formulas still work when
        /// higher-order terms are present (see Farouki & Dong, 2012).
        /// </summary>
        private static PHCurve3D CreateHigherOrderCurve()
        {
            Vector3 a = new(1f, 0f, 0f);
            Vector3 b = new(0f, 1f, 0f);
            Vector3 c = new(0f, 0f, 1f);
            Vector3 d = new(1f, 0f, 0f);
            Vector3 e = new(0f, 1f, 0f);
            return new PHCurve3D(a, b, c, d, e);
        }

        /// <summary>
        /// Position is the time integral of the derivative. For the parabola we
        /// expect r(1) = (1, 0.5, 0).
        /// </summary>
        [TestMethod]
        public void Position_EqualsIntegralOfDerivative()
        {
            PHCurve3D curve = CreateParabola();
            Vector3 pos1 = curve.Position(1f);
            Vector3 expected = new(1f, 0.5f, 0f);
            System.Console.WriteLine($"Position(1)={pos1}");
            AssertVector(expected, pos1, 1e-6f, "Position at t=1");
        }

        /// <summary>
        /// The derivative and second derivative should match the analytic values
        /// r'(t) = (1, t, 0) and r''(t) = (0, 1, 0).
        /// </summary>
        [TestMethod]
        public void Derivative_AndSecondDerivative_MatchAnalytic()
        {
            PHCurve3D curve = CreateParabola();
            float t = 0.3f;
            Vector3 deriv = curve.Derivative(t);
            Vector3 second = curve.SecondDerivative(t);
            Vector3 expectedDeriv = new(1f, t, 0f);
            Vector3 expectedSecond = new(0f, 1f, 0f);
            System.Console.WriteLine($"Derivative({t})={deriv}");
            System.Console.WriteLine($"SecondDerivative({t})={second}");
            AssertVector(expectedDeriv, deriv, 1e-6f, "Derivative");
            AssertVector(expectedSecond, second, 1e-6f, "SecondDerivative");
        }

        /// <summary>
        /// TangentUnit must equal the normalized derivative and the speed must
        /// equal the derivative length: |r'(t)| = sqrt(1 + t^2).
        /// </summary>
        [TestMethod]
        public void Speed_AndTangentUnit_AreConsistent()
        {
            PHCurve3D curve = CreateParabola();
            float t = 0.4f;
            float speed = curve.Speed(t);
            Vector3 tangent = curve.TangentUnit(t);
            float expectedSpeed = MathF.Sqrt(1f + t * t);
            Vector3 expectedTangent = new Vector3(1f, t, 0f) / expectedSpeed;
            System.Console.WriteLine($"Speed({t})={speed}");
            System.Console.WriteLine($"TangentUnit({t})={tangent}");
            Assert.AreEqual(expectedSpeed, speed, 1e-6f, "Speed");
            AssertVector(expectedTangent, tangent, 1e-6f, "TangentUnit");
        }

        /// <summary>
        /// PrincipalNormal should be orthogonal to the derivative and of unit
        /// length. For a planar parabola the expected direction is
        /// N(t) = (-t, 1, 0)/sqrt(1 + t^2).
        /// </summary>
        [TestMethod]
        public void PrincipalNormal_HasExpectedProperties()
        {
            PHCurve3D curve = CreateParabola();
            float t = 0.6f;
            Vector3 normal = curve.PrincipalNormal(t);
            Vector3 deriv = curve.Derivative(t);
            Vector3 expected = new Vector3(-t, 1f, 0f) / MathF.Sqrt(1f + t * t);
            System.Console.WriteLine($"PrincipalNormal({t})={normal}");
            AssertVector(expected, normal, 1e-6f, "PrincipalNormal direction");
            Assert.IsTrue(MathF.Abs(Vector3.Dot(normal, deriv)) < 1e-6f, "Normal not orthogonal to derivative");
            Assert.IsTrue(MathF.Abs(normal.Length() - 1f) < 1e-6f, "Normal not unit length");
        }

        /// <summary>
        /// Verify computations for a curve with higher-order terms where
        /// C, D and E are non-zero.  The analytic formulas are derived by
        /// integrating the hodograph r'(t) = (1 + t³, t + t⁴, t²).
        /// References: Farouki & Dong (2012) for PH-quintic properties.
        /// </summary>
        [TestMethod]
        public void HigherOrderCoefficients_AreHandledCorrectly()
        {
            PHCurve3D curve = CreateHigherOrderCurve();
            float t = 0.4f;

            // Expected position from analytic integration
            Vector3 expectedPos = new(
                t + MathF.Pow(t, 4f) / 4f,
                0.5f * t * t + MathF.Pow(t, 5f) / 5f,
                MathF.Pow(t, 3f) / 3f);
            Vector3 pos = curve.Position(t);
            System.Console.WriteLine($"HigherOrder Position({t})={pos}");
            AssertVector(expectedPos, pos, 1e-6f, "Position");

            // Derivatives from differentiating the position polynomials
            Vector3 expectedDeriv = new(
                1f + MathF.Pow(t, 3f),
                t + MathF.Pow(t, 4f),
                t * t);
            Vector3 deriv = curve.Derivative(t);
            System.Console.WriteLine($"HigherOrder Derivative({t})={deriv}");
            AssertVector(expectedDeriv, deriv, 1e-6f, "Derivative");

            Vector3 expectedSecond = new(
                3f * t * t,
                1f + 4f * MathF.Pow(t, 3f),
                2f * t);
            Vector3 second = curve.SecondDerivative(t);
            System.Console.WriteLine($"HigherOrder SecondDerivative({t})={second}");
            AssertVector(expectedSecond, second, 1e-6f, "SecondDerivative");

            float expectedSpeed = expectedDeriv.Length();
            float speed = curve.Speed(t);
            Vector3 expectedTangent = Vector3.Normalize(expectedDeriv);
            Vector3 tangent = curve.TangentUnit(t);
            System.Console.WriteLine($"HigherOrder Speed({t})={speed}");
            System.Console.WriteLine($"HigherOrder TangentUnit({t})={tangent}");
            Assert.AreEqual(expectedSpeed, speed, 1e-6f, "Speed");
            AssertVector(expectedTangent, tangent, 1e-6f, "TangentUnit");

            Vector3 expectedNormal = Vector3.Normalize(
                (expectedSecond * expectedSpeed - expectedDeriv * Vector3.Dot(expectedDeriv, expectedSecond) / expectedSpeed)
                / (expectedSpeed * expectedSpeed));
            Vector3 normal = curve.PrincipalNormal(t);
            System.Console.WriteLine($"HigherOrder PrincipalNormal({t})={normal}");
            AssertVector(expectedNormal, normal, 1e-6f, "PrincipalNormal");
            Assert.IsTrue(MathF.Abs(Vector3.Dot(normal, deriv)) < 1e-6f, "Normal not orthogonal");
            Assert.IsTrue(MathF.Abs(normal.Length() - 1f) < 1e-6f, "Normal not unit length");
        }

        /// <summary>
        /// Verify that curvature, normal, tangent and bi-tangent are computed
        /// according to the Frenet--Serret formulas for a simple parabola.
        /// </summary>
        [TestMethod]
        public void FrenetFrameQuantities_AreCorrect()
        {
            PHCurve3D curve = CreateParabola();
            float t = 0.25f;

            Vector3 tangent = curve.Tangent(t);
            Vector3 expectedTangent = Vector3.Normalize(new Vector3(1f, t, 0f));
            System.Console.WriteLine($"Tangent({t})={tangent}");
            AssertVector(expectedTangent, tangent, 1e-6f, "Tangent");

            Vector3 normal = curve.Normal(t);
            Vector3 expectedNormal = Vector3.Normalize(new Vector3(-t, 1f, 0f));
            System.Console.WriteLine($"Normal({t})={normal}");
            AssertVector(expectedNormal, normal, 1e-6f, "Normal");

            Vector3 bitan = curve.BiTangent(t);
            Vector3 expectedBitan = Vector3.Cross(expectedTangent, expectedNormal);
            System.Console.WriteLine($"BiTangent({t})={bitan}");
            AssertVector(expectedBitan, bitan, 1e-6f, "BiTangent");

            float curvature = curve.Curvature(t);
            float speed = MathF.Sqrt(1f + t * t);
            float expectedCurv = 1f / (speed * speed * speed);
            System.Console.WriteLine($"Curvature({t})={curvature}");
            Assert.AreEqual(expectedCurv, curvature, 1e-6f, "Curvature");
        }
    }
}
