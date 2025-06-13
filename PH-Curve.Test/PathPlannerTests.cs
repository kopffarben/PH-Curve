using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Numerics;
using PHCurveLibrary;

namespace PHCurveLibrary.Tests
{
    [TestClass]
    public class PathPlannerTests
    {
        private static HermiteControlPoint3D CreatePoint(Vector3 pos, Vector3 tan)
        {
            return new HermiteControlPoint3D(pos, tan, 0f, Vector3.UnitY);
        }

        private static HermiteControlPoint3D CreatePoint(Vector3 pos, Vector3 tan, float k, Vector3 n)
        {
            return new HermiteControlPoint3D(pos, tan, k, n);
        }

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
            var planner = new PathPlanner();
            var p0 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p1 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p2 = CreatePoint(Vector3.Zero, Vector3.UnitX);

            planner.AddSegment(p0, p1);
            planner.AddSegment(p1, p2);
            var path = planner.BuildPath();

            Assert.AreEqual(2, path.Count);
        }

        [TestMethod]
        public void ValidatePathG2_ReturnsTrueForContinuousPath()
        {
            var planner = new PathPlanner();
            var p0 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p1 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p2 = CreatePoint(Vector3.Zero, Vector3.UnitX);

            planner.AddSegment(p0, p1);
            planner.AddSegment(p1, p2);

            Assert.IsTrue(planner.ValidatePathG2());
        }

        [TestMethod]
        public void ValidatePathG2_ReturnsFalseForDiscontinuousPath()
        {
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
            Assert.AreEqual(points.Length - 1, path.Count);
        }

        [TestMethod]
        public void BuildPath_WithSemicirclePoints()
        {
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
            var planner = new PathPlanner();
            var start = CreatePoint(new Vector3(0f, 0f, 0f), Vector3.UnitX, 0.1f, Vector3.UnitY);
            var end = CreatePoint(new Vector3(1f, 1f, 0f), Vector3.UnitY, 0.2f, -Vector3.UnitX);

            planner.AddSegment(start, end);
            var path = planner.BuildPath();
            Assert.AreEqual(1, path.Count);

            var seg = path[0];
            Assert.AreEqual(start.Curvature, Curvature(seg, 0f), 1e-3f);
            Assert.AreEqual(end.Curvature, Curvature(seg, 1f), 1e-3f);
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(start.PrincipalNormal), seg.PrincipalNormal(0f)) < 1e-3f);
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(end.PrincipalNormal), seg.PrincipalNormal(1f)) < 1e-3f);
        }

        [TestMethod]
        public void ValidatePathG2_FailsWhenNormalsDiffer()
        {
            var planner = new PathPlanner();
            var start = CreatePoint(Vector3.Zero, Vector3.UnitX, 0.1f, Vector3.UnitY);
            var jointEnd = CreatePoint(Vector3.One, Vector3.UnitX, 0.1f, Vector3.UnitY);
            var jointStart = CreatePoint(Vector3.One, Vector3.UnitX, 0.1f, Vector3.UnitZ);
            var end = CreatePoint(new Vector3(2f, 1f, 0f), Vector3.UnitY, 0.2f, -Vector3.UnitX);

            planner.AddSegment(start, jointEnd);
            planner.AddSegment(jointStart, end);

            Assert.IsFalse(planner.ValidatePathG2());
        }
    }
}
