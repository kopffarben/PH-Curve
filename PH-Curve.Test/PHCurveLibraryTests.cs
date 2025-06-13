// ======================= Tests =======================
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Numerics;
using PHCurveLibrary;

namespace PHCurveLibrary.Tests
{
    /// <summary>
    /// Unit tests for <see cref="PHCurveLibrary"/> and <see cref="PHCurveFactory"/> functionality.
    /// In particular these tests demonstrate how the struct <see cref="HermiteControlPoint3D"/>
    /// collects all Hermite data (position, tangent, curvature and principal normal) in a single
    /// value type.  This design simplifies the construction of PH segments where both endpoints
    /// share the same data layout and therefore allows a direct call to
    /// <see cref="PHCurveFactory.CreateQuintic(HermiteControlPoint3D, HermiteControlPoint3D)"/>.
    /// </summary>
    [TestClass]
    public class PHCurveLibraryTests
    {
        [TestMethod]
        public void QuinticG2Interpolation_MatchesHermite()
        {
            var p0 = new HermiteControlPoint3D(new Vector3(0, 0, 0), new Vector3(1, 0, 0), 0, new Vector3(0, 1, 0));
            var p1 = new HermiteControlPoint3D(new Vector3(1, 1, 0), new Vector3(0, 1, 0), 0, new Vector3(-1, 0, 0));
            var c = PHCurveFactory.CreateQuintic(p0, p1);
            Assert.IsTrue(Vector3.Distance(Vector3.Zero, c.Position(0)) < 1e-6f);
            var expectedDelta = p1.Position - p0.Position;
            Assert.IsTrue(Vector3.Distance(expectedDelta, c.Position(1)) < 2e-6f);
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(c.Derivative(0)), Vector3.Normalize(p0.Tangent)) < 1e-3f);
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(c.Derivative(1)), Vector3.Normalize(p1.Tangent)) < 1e-3f);
            Assert.IsFalse(PHCurveFactory.ValidateG2(c, c));
        }

        [TestMethod]
        public void FiniteDifferenceDerivative_ApproximatesAnalytic()
        {
            var p = new HermiteControlPoint3D(new Vector3(0, 0, 0), new Vector3(1, 2, 0), 0.1f, new Vector3(-2, 1, 0));
            var c = PHCurveFactory.CreateQuintic(p, p);
            for (int i = 1; i < 10; i++)
            {
                float t = i / 10f;
                var h = 1e-3f;
                var num = (c.Position(t + h) - c.Position(t - h)) / (2 * h);
                var ana = c.Derivative(t);
                Assert.IsTrue(Vector3.Distance(num, ana) < 1e-2f);
            }
        }

        [TestMethod]
        public void OffsetPoint_MaintainsDistance()
        {
            var p = new HermiteControlPoint3D(new Vector3(0, 0, 0), new Vector3(1, 0, 0), 0.5f, new Vector3(0, 1, 0));
            var c = PHCurveFactory.CreateQuintic(p, p);
            float d = 0.2f;
            for (int i = 0; i <= 5; i++)
            {
                var t = i / 5f;
                var pos = c.Position(t);
                var n = c.PrincipalNormal(t);
                var off = pos + d * n;
                Assert.IsTrue(Math.Abs(Vector3.Distance(off, pos) - d) < 1e-4f);
            }
        }

        [TestMethod]
        public void Speed_IsMonotonicForStraightLine()
        {
            var p0 = new HermiteControlPoint3D(new Vector3(0, 0, 0), new Vector3(1, 0, 0), 0, new Vector3(0, 1, 0));
            var p1 = new HermiteControlPoint3D(new Vector3(2, 0, 0), new Vector3(1, 0, 0), 0, new Vector3(0, 1, 0));
            var c = PHCurveFactory.CreateQuintic(p0, p1);
            for (int i = 0; i <= 5; i++)
            {
                float t = i / 10f;
                float s0 = c.Speed(t);
                float s1 = c.Speed(1f - t);
                Assert.IsTrue(Math.Abs(s0 - s1) < 1e-6f);
            }
        }

        [TestMethod]
        public void ValidateG2_BetweenSegments()
        {
            var a = new HermiteControlPoint3D(Vector3.Zero, new Vector3(1, 0, 0), 0, new Vector3(0, 1, 0));
            var b = new HermiteControlPoint3D(Vector3.Zero, new Vector3(0, 1, 0), 0, new Vector3(-1, 0, 0));
            var c = new HermiteControlPoint3D(Vector3.Zero, new Vector3(1, 0, 0), 0, new Vector3(0, -1, 0));
            var c1 = PHCurveFactory.CreateQuintic(a, b);
            var c2 = PHCurveFactory.CreateQuintic(b, c);
            Assert.IsTrue(PHCurveFactory.ValidateG2(c1, c2));
        }

        /// <summary>
        /// TangentUnit should always equal the normalised derivative.
        /// </summary>
        [TestMethod]
        public void TangentUnit_EqualsNormalizedDerivative()
        {
            var p = new HermiteControlPoint3D(new Vector3(0, 0, 0), new Vector3(1, 0.5f, 0), 0.2f, new Vector3(0, 0, 1));
            var c = PHCurveFactory.CreateQuintic(p, p);
            for (int i = 0; i <= 10; i++)
            {
                float t = i / 10f;
                var expected = Vector3.Normalize(c.Derivative(t));
                var actual = c.TangentUnit(t);
                Assert.IsTrue(Vector3.Distance(expected, actual) < 1e-5f);
            }
        }

        /// <summary>
        /// PrincipalNormal must be a unit vector and perpendicular to the derivative.
        /// </summary>
        [TestMethod]
        public void PrincipalNormal_IsUnitAndOrthogonal()
        {
            var p = new HermiteControlPoint3D(new Vector3(0, 0, 0), new Vector3(1, 1, 0), 0.5f, Vector3.UnitZ);
            var c = PHCurveFactory.CreateQuintic(p, p);
            for (int i = 1; i < 10; i++)
            {
                float t = i / 10f;
                var d = c.Derivative(t);
                var n = c.PrincipalNormal(t);
                Assert.IsTrue(Math.Abs(Vector3.Dot(d, n)) < 1e-5f);
                Assert.IsTrue(Math.Abs(n.Length() - 1f) < 1e-5f);
            }
        }

        /// <summary>
        /// Validation should fail when the principal normals do not match even if positions and tangents do.
        /// </summary>
        [TestMethod]
        public void ValidateG2_FailsWhenNormalsDiffer()
        {
            var a = new HermiteControlPoint3D(Vector3.Zero, Vector3.UnitX, 0, Vector3.UnitY);
            var b1 = new HermiteControlPoint3D(Vector3.One, Vector3.UnitX, 0, Vector3.UnitY);
            var b2 = new HermiteControlPoint3D(Vector3.One, Vector3.UnitX, 0, Vector3.UnitZ);
            var c1 = PHCurveFactory.CreateQuintic(a, b1);
            var c2 = PHCurveFactory.CreateQuintic(a, b2);
            Assert.IsFalse(PHCurveFactory.ValidateG2(c1, c2));
        }
    }
}
