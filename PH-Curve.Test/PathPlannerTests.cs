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
    }
}
