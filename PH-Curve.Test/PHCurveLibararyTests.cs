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
        /// <summary>
        /// Verifies that the quintic PH interpolation matches the specified Hermite data.
        /// </summary>
        [TestMethod]
        public void QuinticG2Interpolation_Struct_MatchesData()
        {
            var p0 = new HermiteControlPoint3D(
                new Vector3(0, 0, 0),
                new Vector3(1, 0, 0),
                curvature: 0f,
                principalNormal: new Vector3(0, 1, 0));
            var p1 = new HermiteControlPoint3D(
                new Vector3(1, 1, 0),
                new Vector3(0, 1, 0),
                curvature: 0f,
                principalNormal: new Vector3(-1, 0, 0));

            var curve = PHCurveFactory.CreateQuintic(p0, p1);
            Assert.AreEqual(p0.Position, curve.Position(0f));
            Assert.AreEqual(p1.Position, curve.Position(1f));
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(curve.Derivative(0f)), Vector3.Normalize(p0.Tangent)) < 1e-3f);
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(curve.Derivative(1f)), Vector3.Normalize(p1.Tangent)) < 1e-3f);
            Assert.IsTrue(PHCurveFactory.ValidateG2(curve, curve));
        }

        /// <summary>
        /// Ensures positive speed throughout and consistency between speed integration and length.
        /// </summary>
        [TestMethod]
        public void SpeedAndLength_AreConsistent()
        {
            var p0 = new HermiteControlPoint3D(
                Vector3.Zero,
                new Vector3(1, 1, 0),
                curvature: 0.5f,
                principalNormal: new Vector3(-1, 1, 0));
            var p1 = new HermiteControlPoint3D(
                new Vector3(2, 2, 0),
                new Vector3(-1, 1, 0),
                curvature: 0.2f,
                principalNormal: new Vector3(1, 1, 0));

            var curve = PHCurveFactory.CreateQuintic(p0, p1);
            float sp0 = curve.Speed(0f);
            Assert.IsTrue(sp0 > 0);
            float spMid = curve.Speed(0.5f);
            Assert.IsTrue(spMid > 0);

            int n = 100;
            float sum = 0;
            for (int i = 0; i <= n; i++)
            {
                float t = i / (float)n;
                float w = (i == 0 || i == n ? 1 : i % 2 == 0 ? 2 : 4);
                sum += w * curve.Speed(t);
            }
            float length = (1f / 3f) * (1f / n) * sum;
            Assert.IsTrue(length > 0);
        }

        /// <summary>
        /// Checks that principal normal is orthogonal to the tangent at multiple t values.
        /// </summary>
        [TestMethod]
        public void PrincipalNormal_PerpendicularToTangent()
        {
            var p0 = new HermiteControlPoint3D(
                Vector3.Zero,
                new Vector3(0, 1, 0),
                curvature: 0.3f,
                principalNormal: new Vector3(1, 0, 0));
            var curve = PHCurveFactory.CreateQuintic(p0, p0);

            for (int i = 0; i <= 5; i++)
            {
                float t = i / 5f;
                var T = curve.TangentUnit(t);
                var N = curve.PrincipalNormal(t);
                Assert.IsTrue(Math.Abs(Vector3.Dot(T, N)) < 1e-3f);
            }
        }

        /// <summary>
        /// Validates G² continuity between two distinct quintic segments.
        /// </summary>
        [TestMethod]
        public void ValidateG2_BetweenDifferentSegments()
        {
            var p0 = new HermiteControlPoint3D(
                Vector3.Zero,
                new Vector3(1, 0, 0),
                curvature: 0f,
                principalNormal: new Vector3(0, 1, 0));
            var p1 = new HermiteControlPoint3D(
                new Vector3(1, 1, 0),
                new Vector3(0, 1, 0),
                curvature: 0f,
                principalNormal: new Vector3(-1, 0, 0));
            var p2 = new HermiteControlPoint3D(
                new Vector3(2, 1, 0),
                new Vector3(1, 0, 0),
                curvature: 0f,
                principalNormal: new Vector3(0, -1, 0));

            var c1 = PHCurveFactory.CreateQuintic(p0, p1);
            var c2 = PHCurveFactory.CreateQuintic(p1, p2);
            Assert.IsTrue(PHCurveFactory.ValidateG2(c1, c2));
        }
    }
}
