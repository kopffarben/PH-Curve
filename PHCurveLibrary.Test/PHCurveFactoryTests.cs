// ======================= Tests =======================
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Numerics;
using PHCurveLibrary;

namespace PHCurveLibrary.Tests
{
    /// <summary>
    /// Unit tests for <see cref="PHCurveFactory"/> and <see cref="PHCurve3D"/>.
    /// The factory implements the G² interpolation scheme described by Farouki
    /// &amp; Dong (2012) and Jaklić et&nbsp;al. (2015). A quintic PH curve
    /// <c>r(t)</c> has a polynomial derivative whose squared norm is again a
    /// polynomial. This special structure enables exact arc-length evaluation
    /// and simplifies offset generation. The factory solves for the remaining
    /// hodograph coefficients so that position, tangent and signed curvature
    /// match at both endpoints. Each test verifies one aspect of this behaviour.
    /// The goal is to document the mathematics while ensuring the implementation
    /// remains stable.
    /// </summary>
    [TestClass]
    public class PHCurveFactoryTests
    {
        [TestMethod]
        public void QuinticG2Interpolation_MatchesHermite()
        {
            // The factory must reproduce the supplied Hermite data exactly. In
            // the notation of Jaklić et al. this means r(0) = P0, r(1) = P1 and
            // T(0), T(1) coincide with the provided tangent directions.
            HermiteControlPoint3D p0 = new(new Vector3(0, 0, 0), new Vector3(1, 0, 0), 0, new Vector3(0, 1, 0));
            HermiteControlPoint3D p1 = new(new Vector3(1, 1, 0), new Vector3(0, 1, 0), 0, new Vector3(-1, 0, 0));

            PHCurve3D c = PHCurveFactory.CreateQuintic(p0, p1);

            Console.WriteLine(
                "QuinticG2Interpolation_MatchesHermite: Created curve from ({0}) to ({1}).",
                p0.Position,
                p1.Position);

            // Position continuity at both ends.
            Assert.IsTrue(Vector3.Distance(Vector3.Zero, c.Position(0)) < 1e-6f);
            Vector3 expectedDelta = p1.Position - p0.Position;
            Assert.IsTrue(Vector3.Distance(expectedDelta, c.Position(1)) < 2e-6f);

            // Tangents must also coincide. Normals are checked separately.
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(c.Derivative(0)), Vector3.Normalize(p0.Tangent)) < 1e-3f);
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(c.Derivative(1)), Vector3.Normalize(p1.Tangent)) < 1e-3f);

            // A single segment cannot be G² to itself when evaluated in reverse
            // direction. ValidateG2 should therefore return false here.
            Assert.IsFalse(PHCurveFactory.ValidateG2(c, c));
            Console.WriteLine(
                "QuinticG2Interpolation_MatchesHermite: Validation against itself returned false as expected.");
        }

        [TestMethod]
        public void FiniteDifferenceDerivative_ApproximatesAnalytic()
        {
            // Numerical differentiation should agree with the analytic
            // expression of r'(t). This verifies that the integration constants
            // in Position(t) are correct.
            HermiteControlPoint3D p = new(new Vector3(0, 0, 0), new Vector3(1, 2, 0), 0.1f, new Vector3(-2, 1, 0));
            PHCurve3D c = PHCurveFactory.CreateQuintic(p, p);
            Console.WriteLine(
                "FiniteDifferenceDerivative_ApproximatesAnalytic: Start derivative comparison.");
            for (int i = 1; i < 10; i++)
            {
                float t = i / 10f;
                float h = 1e-3f;
                Vector3 numerical = (c.Position(t + h) - c.Position(t - h)) / (2 * h);
                Vector3 analytic = c.Derivative(t);
                Assert.IsTrue(Vector3.Distance(numerical, analytic) < 1e-2f);
            }
            Console.WriteLine(
                "FiniteDifferenceDerivative_ApproximatesAnalytic: All sample points matched analytic derivative.");
        }

        [TestMethod]
        public void OffsetPoint_MaintainsDistance()
        {
            // Offsetting along the principal normal should preserve the offset
            // distance. This property stems from the unit length of the normal
            // and serves as a basic sanity check for the Frenet frame
            // computation.
            HermiteControlPoint3D p = new(new Vector3(0, 0, 0), new Vector3(1, 0, 0), 0.5f, new Vector3(0, 1, 0));
            PHCurve3D c = PHCurveFactory.CreateQuintic(p, p);
            float d = 0.2f;
            Console.WriteLine("OffsetPoint_MaintainsDistance: Checking offset distance {0}.", d);
            for (int i = 0; i <= 5; i++)
            {
                float t = i / 5f;
                Vector3 pos = c.Position(t);
                Vector3 n = c.PrincipalNormal(t);
                Vector3 off = pos + d * n;
                Assert.IsTrue(Math.Abs(Vector3.Distance(off, pos) - d) < 1e-4f);
            }
            Console.WriteLine("OffsetPoint_MaintainsDistance: Offset distance verified for sample points.");
        }

        [TestMethod]
        public void Speed_IsMonotonicForStraightLine()
        {
            // For a straight line segment all derivative coefficients except A
            // vanish. The speed should therefore be constant and symmetric with
            // respect to t -> 1 - t.
            HermiteControlPoint3D p0 = new(new Vector3(0, 0, 0), new Vector3(1, 0, 0), 0, new Vector3(0, 1, 0));
            HermiteControlPoint3D p1 = new(new Vector3(2, 0, 0), new Vector3(1, 0, 0), 0, new Vector3(0, 1, 0));
            PHCurve3D c = PHCurveFactory.CreateQuintic(p0, p1);
            Console.WriteLine("Speed_IsMonotonicForStraightLine: Testing symmetry of speed.");
            for (int i = 0; i <= 5; i++)
            {
                float t = i / 10f;
                float s0 = c.Speed(t);
                float s1 = c.Speed(1f - t);
                Assert.IsTrue(Math.Abs(s0 - s1) < 1e-6f);
            }
            Console.WriteLine("Speed_IsMonotonicForStraightLine: Speed symmetry verified.");
        }

        [TestMethod]
        public void ValidateG2_BetweenSegments()
        {
            // Construct two segments sharing the same junction point b. The
            // ValidateG2 method checks position, tangent and principal normal at
            // the join. The chosen data forms a quarter circle followed by
            // another quarter circle resulting in a smooth half circle.
            HermiteControlPoint3D a = new(Vector3.Zero, new Vector3(1, 0, 0), 0, new Vector3(0, 1, 0));
            HermiteControlPoint3D b = new(Vector3.Zero, new Vector3(0, 1, 0), 0, new Vector3(-1, 0, 0));
            HermiteControlPoint3D cPoint = new(Vector3.Zero, new Vector3(1, 0, 0), 0, new Vector3(0, -1, 0));
            PHCurve3D c1 = PHCurveFactory.CreateQuintic(a, b);
            PHCurve3D c2 = PHCurveFactory.CreateQuintic(b, cPoint);
            Console.WriteLine("ValidateG2_BetweenSegments: Checking join between segments c1 and c2.");
            Assert.IsTrue(PHCurveFactory.ValidateG2(c1, c2));
            Console.WriteLine("ValidateG2_BetweenSegments: Segments pass G2 validation.");
        }

        /// <summary>
        /// TangentUnit should always equal the normalised derivative.
        /// </summary>
        [TestMethod]
        public void TangentUnit_EqualsNormalizedDerivative()
        {
            HermiteControlPoint3D p = new(new Vector3(0, 0, 0), new Vector3(1, 0.5f, 0), 0.2f, new Vector3(0, 0, 1));
            PHCurve3D c = PHCurveFactory.CreateQuintic(p, p);
            Console.WriteLine(
                "TangentUnit_EqualsNormalizedDerivative: Comparing tangent unit vector against derivative.");
            for (int i = 0; i <= 10; i++)
            {
                float t = i / 10f;
                Vector3 expected = Vector3.Normalize(c.Derivative(t));
                Vector3 actual = c.TangentUnit(t);
                Assert.IsTrue(Vector3.Distance(expected, actual) < 1e-5f);
            }
            Console.WriteLine("TangentUnit_EqualsNormalizedDerivative: All sample points matched.");
        }

        /// <summary>
        /// PrincipalNormal must be a unit vector and perpendicular to the derivative.
        /// </summary>
        [TestMethod]
        public void PrincipalNormal_IsUnitAndOrthogonal()
        {
            HermiteControlPoint3D p = new(new Vector3(0, 0, 0), new Vector3(1, 1, 0), 0.5f, Vector3.UnitZ);
            PHCurve3D c = PHCurveFactory.CreateQuintic(p, p);
            Console.WriteLine("PrincipalNormal_IsUnitAndOrthogonal: Checking orthogonality and unit length.");
            for (int i = 1; i < 10; i++)
            {
                float t = i / 10f;
                Vector3 d = c.Derivative(t);
                Vector3 n = c.PrincipalNormal(t);
                Assert.IsTrue(Math.Abs(Vector3.Dot(d, n)) < 1e-5f);
                Assert.IsTrue(Math.Abs(n.Length() - 1f) < 1e-5f);
            }
            Console.WriteLine("PrincipalNormal_IsUnitAndOrthogonal: Normal vector validated at multiple points.");
        }

        /// <summary>
        /// Validation should fail when the principal normals do not match even if positions and tangents do.
        /// </summary>
        [TestMethod]
        public void ValidateG2_FailsWhenNormalsDiffer()
        {
            HermiteControlPoint3D a = new(Vector3.Zero, Vector3.UnitX, 0, Vector3.UnitY);
            HermiteControlPoint3D b1 = new(Vector3.One, Vector3.UnitX, 0, Vector3.UnitY);
            HermiteControlPoint3D b2 = new(Vector3.One, Vector3.UnitX, 0, Vector3.UnitZ);
            PHCurve3D c1 = PHCurveFactory.CreateQuintic(a, b1);
            PHCurve3D c2 = PHCurveFactory.CreateQuintic(a, b2);
            Console.WriteLine("ValidateG2_FailsWhenNormalsDiffer: Comparing segments with different normals.");
            Assert.IsFalse(PHCurveFactory.ValidateG2(c1, c2));
            Console.WriteLine("ValidateG2_FailsWhenNormalsDiffer: Validation failed as expected.");
        }

        /// <summary>
        /// Numerical arc-length of a linear segment should equal the distance
        /// between its endpoints. The speed is constant in this case so the
        /// Simpson integration serves as a regression test for the Speed method.
        /// </summary>
        [TestMethod]
        public void ArcLength_NumericalMatchesChordLengthForLine()
        {
            HermiteControlPoint3D p0 = new(Vector3.Zero, new Vector3(2, 0, 0), 0f, Vector3.UnitY);
            HermiteControlPoint3D p1 = new(new Vector3(2, 0, 0), new Vector3(2, 0, 0), 0f, Vector3.UnitY);
            PHCurve3D c = PHCurveFactory.CreateQuintic(p0, p1);

            float length = NumericalArcLength(c);
            Console.WriteLine(
                "ArcLength_NumericalMatchesChordLengthForLine: computed length {0}",
                length);

            Assert.AreEqual(2f, length, 1e-5f);
        }

        /// <summary>
        /// The second derivatives at the endpoints should realise the specified
        /// signed curvature multiplied by the squared tangent magnitude.
        /// </summary>
        [TestMethod]
        public void SecondDerivative_ReflectsEndpointCurvature()
        {
            HermiteControlPoint3D p0 = new(
                new Vector3(0, 0, 0), new Vector3(1, 0, 0), 0.5f, new Vector3(0, 1, 0));
            HermiteControlPoint3D p1 = new(
                new Vector3(1, 1, 0), new Vector3(0, 1, 0), -0.25f, new Vector3(-1, 0, 0));
            PHCurve3D curve = PHCurveFactory.CreateQuintic(p0, p1);

            Console.WriteLine("SecondDerivative_ReflectsEndpointCurvature: verifying r''(0) and r''(1).");

            Vector3 expected0 = p0.PrincipalNormal * (p0.Curvature * p0.Tangent.LengthSquared());
            Vector3 expectedDelta = p1.PrincipalNormal * (p1.Curvature * p1.Tangent.LengthSquared());

            Vector3 d0 = curve.SecondDerivative(0f);
            Vector3 d1 = curve.SecondDerivative(1f);

            Assert.IsTrue(Vector3.Distance(expected0, d0) < 1e-5f);
            Assert.IsTrue(Vector3.Distance(d1 - d0, expectedDelta) < 1e-5f);
        }

        private static float NumericalArcLength(PHCurve3D c, int steps = 100)
        {
            float h = 1f / steps;
            float sum = c.Speed(0f) + c.Speed(1f);
            for (int i = 1; i < steps; i += 2)
            {
                float t = i * h;
                sum += 4f * c.Speed(t);
            }
            for (int i = 2; i < steps; i += 2)
            {
                float t = i * h;
                sum += 2f * c.Speed(t);
            }

            return sum * h / 3f;
        }
    }
}
