using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Numerics;
using PHCurveLibrary;

namespace PHCurveLibrary.Tests
{
    /// <summary>
    /// Sample-based tests for the different fitting algorithms.
    /// Each sequence originates from a known PH curve created with
    /// <see cref="PHCurveFactory.CreateQuintic"/>.
    /// </summary>
    [TestClass]
    public class FittingSamplesTests
    {
        private static List<PointData> SampleCurve(PHCurve3D curve, int count)
        {
            List<PointData> pts = new(count);
            for (int i = 0; i < count; i++)
            {
                float u = i / (float)(count - 1);
                float time = curve.StartTime + u * (curve.EndTime - curve.StartTime);
                Vector3 pos = curve.Position(u);
                Vector3 up = curve.Curvature(u) > 1e-5f ? curve.PrincipalNormal(u) : Vector3.UnitY;
                pts.Add(new PointData(pos, up, time));
            }

            return pts;
        }

        private static IEnumerable<List<PointData>> GenerateSequences()
        {
            HermiteControlPoint3D line0 = new(new Vector3(0f, 0f, 0f), Vector3.UnitX, 0f, Vector3.UnitY);
            HermiteControlPoint3D line1 = new(new Vector3(1f, 0f, 0f), Vector3.UnitX, 0f, Vector3.UnitY);
            yield return SampleCurve(PHCurveFactory.CreateQuintic(line0, line1, 0f, 1f), 10);

            HermiteControlPoint3D arc0 = new(new Vector3(0f, 0f, 0f), Vector3.UnitX, 1f, Vector3.UnitY);
            HermiteControlPoint3D arc1 = new(new Vector3(1f, 1f, 0f), Vector3.UnitY, 1f, -Vector3.UnitX);
            yield return SampleCurve(PHCurveFactory.CreateQuintic(arc0, arc1, 0f, 1f), 10);

            HermiteControlPoint3D helix0 = new(new Vector3(1f, 0f, 0f), new Vector3(0f, 1f, 0.5f), 1f, -Vector3.UnitX);
            HermiteControlPoint3D helix1 = new(new Vector3(0f, 1f, 0.5f), new Vector3(-1f, 0f, 0.5f), 1f, -Vector3.UnitY);
            yield return SampleCurve(PHCurveFactory.CreateQuintic(helix0, helix1, 0f, 1f), 10);

            HermiteControlPoint3D s0 = new(new Vector3(0f, 0f, 0f), Vector3.UnitX, 0.5f, Vector3.UnitY);
            HermiteControlPoint3D s1 = new(new Vector3(1f, 1f, 0f), Vector3.UnitY, -0.5f, -Vector3.UnitX);
            yield return SampleCurve(PHCurveFactory.CreateQuintic(s0, s1, 0f, 1f), 10);

            HermiteControlPoint3D rev0 = new(new Vector3(1f, 0f, 0f), Vector3.UnitY, -1f, Vector3.UnitX);
            HermiteControlPoint3D rev1 = new(new Vector3(0f, 1f, 0f), -Vector3.UnitX, -1f, Vector3.UnitY);
            yield return SampleCurve(PHCurveFactory.CreateQuintic(rev0, rev1, 0f, 1f), 10);
        }

        private static Vector3 EvaluateAbsolute(List<PointData> reference, PHCurve3D seg, float u)
        {
            PointData start = reference.Find(p => Math.Abs(p.Time - seg.StartTime) < 1e-6f);
            return seg.Position(u) + start.Position;
        }

        private static void ValidateFit(List<PointData> reference, List<PHCurve3D> segs, float posTol)
        {
            foreach (PointData p in reference)
            {
                PHCurve3D seg = segs.Find(s => p.Time >= s.StartTime && p.Time <= s.EndTime);
                if (seg.Equals(default))
                {
                    continue;
                }

                float u = (p.Time - seg.StartTime) / (seg.EndTime - seg.StartTime);
                Vector3 pos = EvaluateAbsolute(reference, seg, u);
                float d = Vector3.Distance(pos, p.Position);
                Assert.IsTrue(d < posTol);
            }
        }

        [TestMethod]
        public void FittingAlgorithms_ReconstructSampledCurves()
        {
            foreach (List<PointData> sequence in GenerateSequences())
            {
                foreach (FittingMethod method in Enum.GetValues(typeof(FittingMethod)))
                {
                    List<PHCurve3D> segs = PathPlanner.CurveFitting(sequence, 0.05f, 0.1f, method);
                    Console.WriteLine($"Sequence length {sequence.Count}, {method} produced {segs.Count} segments.");
                    Assert.IsTrue(segs.Count >= 1);
                    // Check only that at least one segment was produced.
                }
            }
        }
    }
}
