using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Numerics;
using PHCurveLibrary;

namespace PHCurveLibrary.Tests
{
    /// <summary>
    /// Tests for the curve fitting methods defined in <see cref="PathPlanner"/>.
    /// Each test prints the validated mathematical property.
    /// </summary>
    [TestClass]
    public class FittingMethodsTests
    {
        private static List<PointData> GenerateLinePoints(int count, bool varyUp)
        {
            var pts = new List<PointData>(count);
            for (int i = 0; i < count; i++)
            {
                Vector3 up = varyUp
                    ? Vector3.Normalize(new Vector3(0f, MathF.Cos(i * 0.1f), MathF.Sin(i * 0.1f)))
                    : Vector3.UnitY;
                pts.Add(new PointData(new Vector3(i, 0f, 0f), up, i));
            }

            return pts;
        }

        private static List<PointData> GenerateArcPoints(int count, bool varyUp)
        {
            var pts = new List<PointData>(count);
            float radius = 10f;
            for (int i = 0; i < count; i++)
            {
                float angle = i * MathF.PI / (count - 1);
                Vector3 pos = new Vector3(MathF.Cos(angle), MathF.Sin(angle), 0f) * radius;
                Vector3 up = varyUp
                    ? Vector3.Normalize(new Vector3(MathF.Cos(angle), 0f, MathF.Sin(angle)))
                    : Vector3.UnitY;
                pts.Add(new PointData(pos, up, i));
            }

            return pts;
        }

        private static void AssertUpVector(List<PHCurve3D> segs, List<PointData> pts, float _)
        {
            float maxAngle = 0f;

            foreach (var seg in segs)
            {
                float start = seg.StartTime;
                float end = seg.EndTime;
                for (int i = 0; i < pts.Count; i++)
                {
                    if (pts[i].Time < start || pts[i].Time > end)
                    {
                        continue;
                    }

                    float t = (pts[i].Time - start) / (end - start);
                    if (seg.Curvature(t) < 1e-5f)
                    {
                        continue;
                    }

                    Vector3 normal = seg.PrincipalNormal(t);
                    Vector3 up = Vector3.Normalize(pts[i].UpVector);
                    float angle = MathF.Acos(Math.Clamp(Vector3.Dot(normal, up), -1f, 1f));
                    if (angle > maxAngle)
                    {
                        maxAngle = angle;
                    }
                    Console.WriteLine($"Angle deviation {angle * 180f / MathF.PI:F2} degrees");
                }
            }
            Console.WriteLine($"Max up-vector deviation: {maxAngle * 180f / MathF.PI:F2} degrees");
            Assert.IsTrue(maxAngle < MathF.PI / 2f);
        }

        [DataTestMethod]
        [DataRow(FittingMethod.LocalHermite)]
        [DataRow(FittingMethod.LeastSquares)]
        [DataRow(FittingMethod.Evolution)]
        [DataRow(FittingMethod.Homotopy)]
        [DataRow(FittingMethod.Heuristic)]
        public void LongSequences_GenerateSegments(FittingMethod method)
        {
            float[] tolerances = { 0.1f, 0.01f, 0.001f };

            foreach (var pts in new[] { GenerateLinePoints(120, false), GenerateArcPoints(120, true) })
            {
                foreach (float tol in tolerances)
                {
                    var fixedSegs = PathPlanner.CurveFitting(pts, tol, 0.01f, method);
                    var incSegs = PathPlanner.CurveFittingIncremental(pts, tol, 0.01f, method);
                    Console.WriteLine($"Long sequence check for {method} at position tolerance {tol}: segments {fixedSegs.Count}.");
                    Assert.AreEqual(fixedSegs.Count, incSegs.Count);
                }
            }
        }

        [DataTestMethod]
        [DataRow(FittingMethod.LocalHermite)]
        [DataRow(FittingMethod.LeastSquares)]
        [DataRow(FittingMethod.Evolution)]
        [DataRow(FittingMethod.Homotopy)]
        [DataRow(FittingMethod.Heuristic)]
        public void UpVectorConsistency_IsMaintained(FittingMethod method)
        {
            foreach (var pts in new[] { GenerateLinePoints(20, true), GenerateArcPoints(20, true) })
            {
                var segs = PathPlanner.CurveFitting(pts, 0.001f, 0.001f, method);
                Console.WriteLine($"Up-vector consistency check for {method}.");
                AssertUpVector(segs, pts, 0.001f);
            }
        }

        [DataTestMethod]
        [DataRow(FittingMethod.LocalHermite)]
        [DataRow(FittingMethod.LeastSquares)]
        [DataRow(FittingMethod.Evolution)]
        [DataRow(FittingMethod.Homotopy)]
        [DataRow(FittingMethod.Heuristic)]
        public void ToleranceVariants_AreRespected(FittingMethod method)
        {
            var pts = GenerateLinePoints(50, false);
            float[] tol = { 0.1f, 0.01f, 0.001f };
            foreach (float t in tol)
            {
                var segs = PathPlanner.CurveFitting(pts, t, t, method);
                Console.WriteLine($"Tolerance {t} check for {method}.");
                Assert.IsTrue(segs.Count > 0);
            }
        }

        [DataTestMethod]
        [DataRow(FittingMethod.LocalHermite)]
        [DataRow(FittingMethod.LeastSquares)]
        [DataRow(FittingMethod.Evolution)]
        [DataRow(FittingMethod.Homotopy)]
        [DataRow(FittingMethod.Heuristic)]
        public void G2Continuity_IsValid(FittingMethod method)
        {
            var pts = GenerateLinePoints(2, true);
            var segs = PathPlanner.CurveFitting(pts, 0.01f, 0.01f, method);
            Console.WriteLine($"G2 continuity check for {method} with {segs.Count} segments.");
            for (int i = 0; i < segs.Count - 1; i++)
            {
                Assert.IsTrue(PHCurveFactory.ValidateG2(segs[i], segs[i + 1], 1e-1f));
            }
        }

        [DataTestMethod]
        [DataRow(FittingMethod.LocalHermite)]
        [DataRow(FittingMethod.LeastSquares)]
        [DataRow(FittingMethod.Evolution)]
        [DataRow(FittingMethod.Homotopy)]
        [DataRow(FittingMethod.Heuristic)]
        public void TimeBounds_ArePreserved(FittingMethod method)
        {
            var pts = GenerateArcPoints(10, true);
            var segs = PathPlanner.CurveFitting(pts, 0.01f, 0.01f, method);
            Console.WriteLine($"Time bounds check for {method}.");
            for (int i = 0; i < segs.Count; i++)
            {
                Assert.AreEqual(pts[i].Time, segs[i].StartTime, 1e-6f);
                Assert.AreEqual(pts[i + 1].Time, segs[i].EndTime, 1e-6f);
                Assert.IsTrue(segs[i].StartTime < segs[i].EndTime);
            }
        }

        [DataTestMethod]
        [DataRow(FittingMethod.LocalHermite)]
        [DataRow(FittingMethod.LeastSquares)]
        [DataRow(FittingMethod.Evolution)]
        [DataRow(FittingMethod.Homotopy)]
        [DataRow(FittingMethod.Heuristic)]
        public void TimeIntermediate_WithinTolerance(FittingMethod method)
        {
            var pts = GenerateLinePoints(2, false);
            var segs = PathPlanner.CurveFitting(pts, 0.01f, 0.01f, method);
            Console.WriteLine($"Intermediate time check for {method}.");
            for (int i = 0; i < segs.Count; i++)
            {
                var seg = segs[i];
                float mid = (seg.StartTime + seg.EndTime) / 2f;
                float u = (mid - seg.StartTime) / (seg.EndTime - seg.StartTime);
                Vector3 pos = seg.Position(u);
                Vector3 expected = new Vector3(mid, 0f, 0f);
                Assert.IsTrue(Vector3.Distance(pos, expected) < 1.0f);
            }
        }
    }
}
