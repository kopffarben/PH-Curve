using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Numerics;
using PHCurveLibrary;

namespace PHCurveLibrary.Tests
{
    [TestClass]
    public class PathPlannerTests
    {
        // These tests validate the PathPlanner according to the approach
        // described in the documentation (see AGENTS.md) for G^2 interpolation
        // of PH curves by Jaklič et al. (2015). This helper method creates a
        // Hermite point without an explicit curvature and shortens scenarios
        // in which only position and tangent direction are relevant.
        private static HermiteControlPoint3D CreatePoint(Vector3 pos, Vector3 tan)
        {
            return new HermiteControlPoint3D(pos, tan, 0f, Vector3.UnitY);
        }

        // Overload of <see cref="CreatePoint(Vector3,Vector3)"/> allowing the
        // caller to specify curvature and principal normal as well.
        private static HermiteControlPoint3D CreatePoint(Vector3 pos, Vector3 tan, float k, Vector3 n)
        {
            return new HermiteControlPoint3D(pos, tan, k, n);
        }

        // Computes the curvature of a generated PH segment. By definition
        // \(\kappa = \|r'(t) \times r''(t)\| / \|r'(t)\|^3\) (see Farouki, 2014).
        // This helper allows us to compare the planner results against the
        // prescribed endpoint data.
        private static float Curvature(PHCurve3D c, float t)
        {
            Vector3 d1 = c.Derivative(t);
            Vector3 d2 = c.SecondDerivative(t);
            Vector3 cross = Vector3.Cross(d1, d2);
            return cross.Length() / MathF.Pow(d1.Length(), 3f);
        }

        [TestMethod]
        public void BuildPath_ReturnsAllSegments()
        {
            // This test only checks whether the internal segment list is
            // returned correctly. The points are identical and therefore create
            // degenerate curves but do not affect the expectation regarding the
            // number of segments.
            var planner = new PathPlanner();
            var p0 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p1 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p2 = CreatePoint(Vector3.Zero, Vector3.UnitX);

            planner.AddSegment(p0, p1);
            planner.AddSegment(p1, p2);
            var path = planner.BuildPath();

            // Two segments are expected because AddSegment was called twice.
            Assert.AreEqual(2, path.Count);
        }

        [TestMethod]
        public void ValidatePathG2_ReturnsTrueForContinuousPath()
        {
            // With identical points and tangents G^2 continuity is trivially satisfied.
            var planner = new PathPlanner();
            var p0 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p1 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p2 = CreatePoint(Vector3.Zero, Vector3.UnitX);

            planner.AddSegment(p0, p1);
            planner.AddSegment(p1, p2);

            // ValidatePathG2 internally calls PHCurveFactory.ValidateG2 following Jaklič et al. (2015).
            Assert.IsTrue(planner.ValidatePathG2());
        }

        [TestMethod]
        public void ValidatePathG2_ReturnsFalseForDiscontinuousPath()
        {
            // Tangent and normal direction differ at the joint which
            // deliberately violates the G^2 condition.
            var planner = new PathPlanner();
            var start = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var junctionEnd = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var junctionStart = CreatePoint(Vector3.Zero, Vector3.UnitY);
            var end = CreatePoint(Vector3.Zero, Vector3.UnitY);

            planner.AddSegment(start, junctionEnd);
            planner.AddSegment(junctionStart, end);

            Assert.IsFalse(planner.ValidatePathG2());
        }

        [TestMethod]
        public void BuildPath_WithStraightLinePoints()
        {
            // Three collinear points with tangents pointing along the x-axis.
            // Each segment should therefore be a pure translation without
            // curvature.
            var planner = new PathPlanner();
            var points = new[]
            {
                CreatePoint(new Vector3(0f, 0f, 0f), Vector3.UnitX),
                CreatePoint(new Vector3(1f, 0f, 0f), Vector3.UnitX),
                CreatePoint(new Vector3(2f, 0f, 0f), Vector3.UnitX)
            };

            for (int i = 0; i < points.Length - 1; i++)
            {
                planner.AddSegment(points[i], points[i + 1]);
            }

            var path = planner.BuildPath();
            // The number of generated segments must equal the number of points minus one.
            Assert.AreEqual(points.Length - 1, path.Count);
        }

        [TestMethod]
        public void BuildPath_WithSemicirclePoints()
        {
            // Points and tangents lie on a semicircle of radius one. This test
            // ensures the planner creates consistent segments for curved
            // trajectories as well.
            var planner = new PathPlanner();
            var points = new[]
            {
                CreatePoint(new Vector3(1f, 0f, 0f), new Vector3(0f, 1f, 0f)),
                CreatePoint(new Vector3(0f, 1f, 0f), new Vector3(-1f, 0f, 0f)),
                CreatePoint(new Vector3(-1f, 0f, 0f), new Vector3(0f, -1f, 0f))
            };

            for (int i = 0; i < points.Length - 1; i++)
            {
                planner.AddSegment(points[i], points[i + 1]);
            }

            var path = planner.BuildPath();
            Assert.AreEqual(points.Length - 1, path.Count);
        }

        [TestMethod]
        public void BuildPath_WithSinePoints()
        {
            // This test approximates a sine segment. The points lie on
            // y = sin(x) for x in [0, \pi] and the tangents correspond to the
            // derivatives of the sine function. It mainly demonstrates varying
            // curvature directions.
            var planner = new PathPlanner();
            var points = new[]
            {
                CreatePoint(new Vector3(0f, 0f, 0f), new Vector3(1f, 1f, 0f)),
                CreatePoint(new Vector3(MathF.PI / 2f, 1f, 0f), new Vector3(1f, 0f, 0f)),
                CreatePoint(new Vector3(MathF.PI, 0f, 0f), new Vector3(1f, -1f, 0f))
            };

            for (int i = 0; i < points.Length - 1; i++)
            {
                planner.AddSegment(points[i], points[i + 1]);
            }

            var path = planner.BuildPath();
            Assert.AreEqual(points.Length - 1, path.Count);
        }

        [TestMethod]
        public void BuildPath_PreservesCurvatureAndPrincipalNormals()
        {
            // Using two Hermite points we verify that the resulting PH spline
            // exactly preserves the specified curvature and principal normals at
            // the ends. This follows the equations for G^2 interpolation given
            // by Jaklič et al. (2015).
            var planner = new PathPlanner();
            var start = CreatePoint(new Vector3(0f, 0f, 0f), Vector3.UnitX, 0.1f, Vector3.UnitY);
            var end = CreatePoint(new Vector3(1f, 1f, 0f), Vector3.UnitY, 0.2f, -Vector3.UnitX);

            planner.AddSegment(start, end);
            var path = planner.BuildPath();
            Assert.AreEqual(1, path.Count);

            var seg = path[0];
            // Compare curvature at the segment ends. The tolerance is based on
            // numerical examples from the PHquintic Library (Farouki & Dong,
            // 2012).
            Assert.AreEqual(start.Curvature, Curvature(seg, 0f), 1e-3f);
            Assert.AreEqual(end.Curvature, Curvature(seg, 1f), 1e-3f);

            // Additionally the principal normals must match.
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(start.PrincipalNormal), seg.PrincipalNormal(0f)) < 1e-3f);
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(end.PrincipalNormal), seg.PrincipalNormal(1f)) < 1e-3f);
        }

        [TestMethod]
        public void ValidatePathG2_FailsWhenNormalsDiffer()
        {
            // A different normal direction is deliberately specified at the
            // joint. According to the definition this breaks G^2 continuity.
            var planner = new PathPlanner();
            var start = CreatePoint(Vector3.Zero, Vector3.UnitX, 0.1f, Vector3.UnitY);
            var jointEnd = CreatePoint(Vector3.One, Vector3.UnitX, 0.1f, Vector3.UnitY);
            var jointStart = CreatePoint(Vector3.One, Vector3.UnitX, 0.1f, Vector3.UnitZ);
            var end = CreatePoint(new Vector3(2f, 1f, 0f), Vector3.UnitY, 0.2f, -Vector3.UnitX);

            planner.AddSegment(start, jointEnd);
            planner.AddSegment(jointStart, end);

            // The G^2 check must fail because the normals at the joint do not
            // match.
            Assert.IsFalse(planner.ValidatePathG2());
        }
    }
}
