using System;
using System.Linq;
using System.Numerics;
using MathNet.Numerics.Optimization;
using MathNet.Numerics.Optimization.LeastSquares; // Required for NonlinearModelProblem

namespace CubicPHCurve
{
    // C# Implementation for Cubic Pythagorean-Hodograph (PH) Curve Fitting and Time-Based Evaluation
    // This class fits a single cubic PH curve segment to given control points with absolute times and normals,
    // optimizing both positional and orientation (normal) matching via Levenberg-Marquardt least squares.
    public class CubicPHCurve3DFitter
    {
        /// <summary>
        /// Represents a control point with 3D position, an absolute timestamp, and a unit normal vector.
        /// </summary>
        /// <remarks>
        /// Position: 3D world-coordinate point to approximate.
        /// Time: Absolute time (e.g., seconds) at which the curve should pass through this point.
        /// Normal: Unit-length normal vector for orientation matching.
        /// </remarks>
        public struct ControlPointEx
        {
            /// <summary>3D world-coordinate of the control point.</summary>
            public Vector3 Position;
            /// <summary>Absolute time (in seconds) at which the curve must reach this point.</summary>
            public float Time;
            /// <summary>Normalized normal vector for orientation fitting.</summary>
            public Vector3 Normal;
        }

        /// <summary>
        /// Fits a single cubic PH curve segment to the provided array of control points.
        /// </summary>
        /// <param name="cps">Array of ControlPointEx structs (must contain at least two points).</param>
        /// <param name="fitted">Output fitted CubicPHCurve3D segment.</param>
        /// <param name="rmsPosError">Output RMS positional error (same units as Position).</param>
        /// <param name="rmsNormError">Output RMS normal orientation error (unitless).</param>
        /// <param name="T0">Output start time of the segment (seconds).</param>
        /// <param name="T1">Output end time of the segment (seconds).</param>
        /// <returns>True if the solver converged successfully; false otherwise.</returns>
        public static bool FitSingleSegmentPH3D(
            ControlPointEx[] cps,
            out CubicPHCurve3D fitted,
            out float rmsPosError,
            out float rmsNormError,
            out float T0,
            out float T1)
        {
            // Validate input
            if (cps == null || cps.Length < 2)
                throw new ArgumentException("At least two control points are required.");

            // Extract absolute times and compute normalization constants
            float[] absTimes = cps.Select(cp => cp.Time).ToArray();
            T0 = absTimes.Min();           // segment start time
            T1 = absTimes.Max();           // segment end time
            float duration = T1 - T0;

            // Map absolute times to normalized parameter t in [0,1]
            float[] times = absTimes.Select(t => (t - T0) / duration).ToArray();

            // Extract endpoint positions
            Vector3 p0 = cps.First().Position;
            Vector3 p1 = cps.Last().Position;

            // Compute endpoint tangent directions using immediate neighbors
            Vector3 u0 = Vector3.Normalize(cps[1].Position - cps[0].Position);
            Vector3 u1 = Vector3.Normalize(cps[^1].Position - cps[^2].Position);

            int N = cps.Length;
            Vector3[] posS = cps.Select(cp => cp.Position).ToArray();  // sample positions
            Vector3[] nrmS = cps.Select(cp => cp.Normal).ToArray();    // target normals
            int totalResiduals = 6 * N;  // 3 position + 3 normal components per point

            // Setup non-linear least squares problem: unknowns are tangent magnitudes [m0, m1]
            var problem = new NonlinearModelProblem(
                initialGuess: new[] { Vector3.Distance(p0, p1), Vector3.Distance(p0, p1) },
                evaluationFunction: (vars, residuals) =>
                {
                    float m0 = vars[0];
                    float m1 = vars[1];

                    // Construct Hermite control points for PH segment
                    var cpH0 = new CubicPHCurve3D.ControlPoint(p0, u0 * m0);
                    var cpH1 = new CubicPHCurve3D.ControlPoint(p1, u1 * m1);
                    var segment = CubicPHCurve3D.FromControlPoints(cpH0, cpH1);

                    // Compute residuals for each sample
                    for (int i = 0; i < N; ++i)
                    {
                        // Position residual (3 components)
                        Vector3 dp = segment.Position(times[i]) - posS[i];
                        residuals[6 * i + 0] = dp.X;
                        residuals[6 * i + 1] = dp.Y;
                        residuals[6 * i + 2] = dp.Z;

                        // Normal residual via Frenet normal
                        Vector3 d1 = segment.Derivative(times[i], 1);                   // first derivative r'
                        Vector3 T = Vector3.Normalize(d1);                              // tangent unit vector
                        Vector3 d2 = segment.Derivative(times[i], 2);                   // second derivative r''
                        Vector3 Nf = Vector3.Normalize(d2 - Vector3.Dot(d2, T) * T);    // Frenet normal
                        Vector3 dn = Nf - nrmS[i];                                      // difference to control-point normal
                        residuals[6 * i + 3] = dn.X;
                        residuals[6 * i + 4] = dn.Y;
                        residuals[6 * i + 5] = dn.Z;
                    }
                },
                residualCount: totalResiduals
            );

            // Use Levenberg-Marquardt to solve the non-linear least squares
            var solver = new LevenbergMarquardtMinimizer();
            var result = solver.FindMinimum(problem);

            // Check for convergence
            if (!result.ReasonForExit.IsConverged())
            {
                fitted = null;
                rmsPosError = float.NaN;
                rmsNormError = float.NaN;
                return false;
            }

            // Extract optimized tangent lengths
            float m0_fit = (float)result.MinimizingPoint[0];
            float m1_fit = (float)result.MinimizingPoint[1];

            // Build final PH segment with optimal tangents
            var finalCP0 = new CubicPHCurve3D.ControlPoint(p0, u0 * m0_fit);
            var finalCP1 = new CubicPHCurve3D.ControlPoint(p1, u1 * m1_fit);
            fitted = CubicPHCurve3D.FromControlPoints(finalCP0, finalCP1);

            // Compute root-mean-square errors for position and normal
            double sumPos2 = 0, sumNorm2 = 0;
            for (int i = 0; i < N; ++i)
            {
                // Position error contribution
                var dp = fitted.Position(times[i]) - posS[i];
                sumPos2 += dp.LengthSquared();

                // Normal error contribution
                Vector3 d1 = fitted.Derivative(times[i], 1);
                Vector3 T = Vector3.Normalize(d1);
                Vector3 d2 = fitted.Derivative(times[i], 2);
                Vector3 Nf = Vector3.Normalize(d2 - Vector3.Dot(d2, T) * T);
                var dn = Nf - nrmS[i];
                sumNorm2 += dn.LengthSquared();
            }
            rmsPosError = (float)Math.Sqrt(sumPos2 / N);
            rmsNormError = (float)Math.Sqrt(sumNorm2 / N);

            return true;
        }
    }

    /// <summary>
    /// Utility methods for evaluating a fitted PH curve in world time.
    /// Compute velocity and scalar speed at arbitrary absolute times.
    /// </summary>
    public static class PHCurveTimeUtils
    {
        /// <summary>
        /// Computes the instantaneous 3D velocity vector at absolute time T.
        /// </summary>
        /// <param name="curve">Fitted PH curve segment (parameterized in [0,1]).</param>
        /// <param name="T">Absolute time at which to evaluate (seconds).</param>
        /// <param name="T0">Segment start time (seconds).</param>
        /// <param name="T1">Segment end time (seconds).</param>
        /// <returns>Velocity vector dr/dT in world space units per second.</returns>
        public static Vector3 VelocityAtTime(
            CubicPHCurve3D curve,
            float T,
            float T0,
            float T1)
        {
            // Map world time T to normalized parameter t_param in [0,1]
            float tParam = (T - T0) / (T1 - T0);

            // Compute derivative dr/dt_param
            Vector3 dr_dtp = curve.Derivative(tParam, 1);

            // Chain rule: dt_param/dT = 1 / duration
            float invDur = 1.0f / (T1 - T0);

            // Velocity dr/dT = (dr/dt_param) * (dt_param/dT)
            return dr_dtp * invDur;
        }

        /// <summary>
        /// Computes the scalar speed (magnitude of velocity) at absolute time T.
        /// </summary>
        /// <param name="curve">Fitted PH curve segment (parameterized in [0,1]).</param>
        /// <param name="T">Absolute time at which to evaluate (seconds).</param>
        /// <param name="T0">Segment start time (seconds).</param>
        /// <param name="T1">Segment end time (seconds).</param>
        /// <returns>Scalar speed in world space units per second.</returns>
        public static float SpeedAtTime(
            CubicPHCurve3D curve,
            float T,
            float T0,
            float T1)
        {
            return VelocityAtTime(curve, T, T0, T1).Length();
        }
    }

    /*
    HLSL Snippet:
    // Computes velocity and speed directly in a shader, given r'(t) and time bounds.
    float3 VelocityAtTime(float3 r1, float T0, float T1)
    {
        float invDur = 1.0 / (T1 - T0);
        return r1 * invDur;
    }

    float SpeedAtTime(float3 r1, float T0, float T1)
    {
        return length(VelocityAtTime(r1, T0, T1));
    }
    */
}
