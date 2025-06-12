using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Numerics;
using CubicPHCurve;

namespace PH_Curve.Test
{
    [TestClass]
    public sealed class Test1
    {
        [TestMethod]
        public void FitSegmentProducesSmallErrors()
        {
            var cps = new[]
            {
                new CubicPHCurve3DFitter.ControlPointEx{Position=new Vector3(0,0,0), Time=0f, Normal=Vector3.UnitZ},
                new CubicPHCurve3DFitter.ControlPointEx{Position=new Vector3(1,1,0), Time=0.5f, Normal=Vector3.UnitZ},
                new CubicPHCurve3DFitter.ControlPointEx{Position=new Vector3(2,0,0), Time=1f, Normal=Vector3.UnitZ}
            };

            bool ok = CubicPHCurve3DFitter.FitSingleSegmentPH3D(cps, out var curve, out var posErr, out var normErr, out var T0, out var T1);
            System.Console.WriteLine($"posErr={posErr} normErr={normErr}");
            Assert.IsTrue(ok);
            Assert.IsTrue(posErr < 1e-2);
            Assert.IsTrue(normErr < 2f);
            Assert.AreEqual(0f, T0, 1e-6f);
            Assert.AreEqual(1f, T1, 1e-6f);

            // check velocity utility
            float midT = (T0 + T1) * 0.5f;
            Vector3 vel = PHCurveTimeUtils.VelocityAtTime(curve, midT, T0, T1);
            Vector3 deriv = curve.Derivative(0.5f) / (T1 - T0);
            Assert.IsTrue(Vector3.Distance(vel, deriv) < 1e-4f);
        }
    }
}
