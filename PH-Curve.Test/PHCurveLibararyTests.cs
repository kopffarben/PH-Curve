// ======================= Tests =======================
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Numerics;
using PHCurveLibrary;

namespace PHCurveLibrary.Tests
{
    /// <summary>
    /// Unit tests for <see cref="PHCurve3D"/> and <see cref="PHCurveFactory"/> functionality.
    /// </summary>
    [TestClass]
    public class PHCurve3DStructTests
    {
        [TestMethod]
        public void QuinticG2Interpolation_MatchesHermite()
        {
            var p0 = new HermiteControlPoint3D(new Vector3(0, 0, 0), new Vector3(1, 0, 0), 0, new Vector3(0, 1, 0));
            var p1 = new HermiteControlPoint3D(new Vector3(1, 1, 0), new Vector3(0, 1, 0), 0, new Vector3(-1, 0, 0));
            var c = PHCurveFactory.CreateQuintic(p0, p1);
            Assert.AreEqual(p0.Position, c.Position(0));
            Assert.AreEqual(p1.Position, c.Position(1));
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(c.Derivative(0)), Vector3.Normalize(p0.Tangent)) < 1e-3f);
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(c.Derivative(1)), Vector3.Normalize(p1.Tangent)) < 1e-3f);
            Assert.IsTrue(PHCurveFactory.ValidateG2(c, c));
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
            float prev = c.Speed(0);
            for (int i = 1; i <= 10; i++)
            {
                var cur = c.Speed(i / 10f);
                Assert.IsTrue(Math.Abs(cur - prev) < 1e-6f);
                prev = cur;
            }
        }

        [TestMethod]
        public void ValidateG2_BetweenSegments()
        {
            var a = new HermiteControlPoint3D(Vector3.Zero, new Vector3(1, 0, 0), 0, new Vector3(0, 1, 0));
            var b = new HermiteControlPoint3D(new Vector3(1, 0, 0), new Vector3(0, 1, 0), 0, new Vector3(-1, 0, 0));
            var c = new HermiteControlPoint3D(new Vector3(1, 1, 0), new Vector3(1, 0, 0), 0, new Vector3(0, -1, 0));
            var c1 = PHCurveFactory.CreateQuintic(a, b);
            var c2 = PHCurveFactory.CreateQuintic(b, c);
            Assert.IsTrue(PHCurveFactory.ValidateG2(c1, c2));
        }
    }
}
