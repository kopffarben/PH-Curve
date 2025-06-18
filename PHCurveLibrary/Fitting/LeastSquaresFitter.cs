// LeastSquaresFitter.cs
// Robust algebraic least-squares fitter for PH-quintic segments
// Handles degenerate inputs, singular normal equations, zero-length segments,
// orientation checks, and recursive subdivision for error control.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace PHCurveLibrary.Fitting
{
    /// <summary>
    /// Provides methods to fit a sequence of 3D points over time with Pythagorean-hodograph
    /// quintic curves using a robust algebraic least-squares approach. Supports position and
    /// orientation tolerances, handles degenerate cases, and subdivides the point set recursively
    /// when tolerances are exceeded.
    /// 
    /// <para>
    /// The <paramref name="points"/> include <c>UpVector</c> information at each sample to guide the principal normal estimation.
    /// When the provided up-vectors are consistent, they produce accurate normals and satisfy orientation tolerances with minimal subdivision.
    /// If up-vectors vary or are unreliable, the algorithm falls back to approximating normals from the fitted curve geometry,
    /// ensuring a reasonable orientation even with imperfect up-vector data.
    /// </para>
    /// </summary>
    public static class LeastSquaresFitter
    {
        // Prevent infinite recursion when subdividing point sequences
        private const int MaxRecursionDepth = 16;
        // Minimum threshold for treating near-zero values as singular
        private const float SingularThreshold = 1e-6f;
        // Default scale for tangent magnitudes if solving normal equations fails
        private const float DefaultAlphaScale = 1.0f;

        /// <summary>
        /// Public entry point: fits one or multiple PH-quintic segments to the given
        /// timed point data, ensuring position and orientation errors do not exceed
        /// specified tolerances. Automatically handles degenerate inputs and recursion
        /// depth limits.
        /// </summary>
        /// <param name="points">Ordered list of time-stamped 3D points with up-vectors.</param>
        /// <param name="positionTolerance">Maximum permitted Euclidean error (units of length).</param>
        /// <param name="orientationTolerance">Maximum permitted angular deviation (radians).</param>
        /// <returns>List of <see cref="PHCurve3D"/> segments approximating the data.</returns>
        /// <exception cref="ArgumentNullException">Thrown if <paramref name="points"/> is null.</exception>
        /// <exception cref="ArgumentException">Thrown if fewer than two points are provided.</exception>
        public static List<PHCurve3D> Fit(
            List<PointData> points,
            float positionTolerance,
            float orientationTolerance)
        {
            if (points == null)
                throw new ArgumentNullException(nameof(points));
            if (points.Count < 2)
                throw new ArgumentException("At least two points required", nameof(points));

            // Detect degenerate case: all points coincide within numerical tolerance
            bool allSame = points.All(p => Vector3.Distance(p.Position, points[0].Position) < SingularThreshold);
            if (allSame)
            {
                // Return single zero-length PH segment at that location
                var h = new HermiteControlPoint3D(
                    points[0].Position,
                    Vector3.Zero,
                    curvature: 0f,
                    principalNormal: Vector3.UnitY);
                var curve = PHCurveFactory.CreateQuintic(h, h, 0f, 1f);
                return new List<PHCurve3D> { curve };
            }

            // Use recursive subdivision to fit tolerances
            return FitRecursive(points, positionTolerance, orientationTolerance, depth: 0);
        }

        /// <summary>
        /// Internal recursive method: attempts to fit one PH segment to the full point set.
        /// If error tolerances are exceeded or singular conditions occur, splits the set at
        /// the worst-fitting point and recurses, up to <see cref="MaxRecursionDepth"/>.
        /// </summary>
        /// <param name="pts">Current subset of the original point list.</param>
        /// <param name="posTol">Position tolerance.</param>
        /// <param name="oriTol">Orientation tolerance (radians).</param>
        /// <param name="depth">Current recursion depth.</param>
        /// <returns>List of fitted segments covering the subset.</returns>
        private static List<PHCurve3D> FitRecursive(
            List<PointData> pts,
            float posTol,
            float oriTol,
            int depth)
        {
            int m = pts.Count;
            if (m < 2)
                throw new ArgumentException("Insufficient points for fitting.");
            if (depth > MaxRecursionDepth)
            {
                // Fallback: connect points with zero-curvature PH segments (polyline)
                return FallbackPolyline(pts);
            }

            // Extract first and last points
            var p0 = pts.First();
            var pN = pts.Last();
            float dt = pN.Time - p0.Time;
            if (Math.Abs(dt) < SingularThreshold)
            {
                // Avoid zero or negative time span
                dt = 1f;
            }

            // Estimate end tangents via finite differences
            Vector3 dir0 = EstimateDirection(pts, idx: 0);
            Vector3 dirN = EstimateDirection(pts, idx: m - 1);

            // Estimate endpoint curvatures from nearby triples
            float curvature0 = EstimateCurvatureEndpoint(pts, atStart: true);
            float curvatureN = EstimateCurvatureEndpoint(pts, atStart: false);

            // Derive principal normals from measured up-vectors
            Vector3 pn0 = EstimatePrincipalNormal(p0.UpVector, dir0);
            Vector3 pnN = EstimatePrincipalNormal(pN.UpVector, dirN);

            // Solve 2x2 normal equations for tangent scale factors α0, α1
            SolveTangentScales(
                pts,
                p0,
                pN,
                dir0,
                dirN,
                dt,
                out float alpha0,
                out float alpha1);

            // Clamp tangent magnitudes to a multiple of chord length
            float chordLength = Vector3.Distance(p0.Position, pN.Position);
            alpha0 = Math.Clamp(alpha0, min: 0f, max: chordLength * 10f);
            alpha1 = Math.Clamp(alpha1, min: 0f, max: chordLength * 10f);

            // Build Hermite boundary conditions for this arc
            var hStart = new HermiteControlPoint3D(
                position: p0.Position,
                tangent: dir0 * alpha0,
                curvature: curvature0,
                principalNormal: pn0);
            var hEnd = new HermiteControlPoint3D(
                position: pN.Position,
                tangent: dirN * alpha1,
                curvature: curvatureN,
                principalNormal: pnN);

            // Create the PH quintic segment
            var segment = PHCurveFactory.CreateQuintic(
                hStart,
                hEnd,
                p0.Time,
                pN.Time);

            // Evaluate max position and orientation error over the sample points
            float maxPosErr = 0f, maxOriErr = 0f;
            int worstIdx = 0;
            for (int i = 0; i < m; ++i)
            {
                float tNorm = (pts[i].Time - p0.Time) / dt;
                Vector3 predicted = segment.Position(tNorm);
                // Euclidean position error
                float posErr = Vector3.Distance(predicted, pts[i].Position);
                if (posErr > maxPosErr)
                {
                    maxPosErr = posErr;
                    worstIdx = i;
                }
                // Normal-based orientation error
                Vector3 curveNormal = segment.Normal(tNorm);
                Vector3 measuredNormal = EstimatePrincipalNormal(
                    pts[i].UpVector,
                    Vector3.Normalize(segment.Tangent(tNorm)));
                float oriErr = AngleBetween(curveNormal, measuredNormal);
                if (oriErr > maxOriErr)
                    maxOriErr = oriErr;
            }

            // Check tolerances
            if (maxPosErr <= posTol && maxOriErr <= oriTol)
            {
                // Good fit: return single segment
                return new List<PHCurve3D> { segment };
            }

            // If only two points remain we cannot subdivide further
            if (m < 3)
            {
                return FallbackPolyline(pts);
            }

            // Subdivide at the point of maximum error (avoid splitting at endpoints)
            worstIdx = Math.Clamp(worstIdx, 1, m - 2);
            var leftPts = pts.GetRange(0, worstIdx + 1);
            var rightPts = pts.GetRange(worstIdx, m - worstIdx);
            // Recursively fit left and right subsets
            var leftRes = FitRecursive(leftPts, posTol, oriTol, depth + 1);
            var rightRes = FitRecursive(rightPts, posTol, oriTol, depth + 1);
            // Concatenate results
            var results = new List<PHCurve3D>(leftRes.Count + rightRes.Count);
            results.AddRange(leftRes);
            results.AddRange(rightRes);
            return results;
        }

        #region Helper Methods

        /// <summary>
        /// Estimates a unit tangent vector at index <paramref name="idx"/>
        /// by forward, backward, or central difference, depending on
        /// boundary.
        /// </summary>
        private static Vector3 EstimateDirection(List<PointData> pts, int idx)
        {
            if (idx == 0)
                return Vector3.Normalize(pts[1].Position - pts[0].Position);
            if (idx == pts.Count - 1)
                return Vector3.Normalize(pts.Last().Position - pts[^2].Position);
            // Central difference
            var prev = pts[idx].Position - pts[idx - 1].Position;
            var next = pts[idx + 1].Position - pts[idx].Position;
            return Vector3.Normalize(prev + next);
        }

        /// <summary>
        /// Approximates curvature at the first or last sample via three-point circle fit.
        /// </summary>
        private static float EstimateCurvatureEndpoint(List<PointData> pts, bool atStart)
        {
            if (pts.Count < 3)
                return 0f;
            if (atStart)
                return EstimateCurvature(pts[0].Position, pts[1].Position, pts[2].Position);
            else
                return EstimateCurvature(pts[^3].Position, pts[^2].Position, pts[^1].Position);
        }

        /// <summary>
        /// Computes the principal normal direction from a measured up-vector
        /// and tangent, with fallback to a world-up if degenerate.
        /// </summary>
        /// <summary>
        /// Computes the principal normal direction from a measured up-vector
        /// and tangent, with fallback to a world-up if unreliable.
        /// </summary>
        private static Vector3 EstimatePrincipalNormal(Vector3 up, Vector3 tangent)
        {
            // Check reliability of provided up-vector: it should be roughly orthogonal to tangent
            float alignmentUp = Math.Abs(Vector3.Dot(up, tangent) / (up.Length() * tangent.Length()));
            const float alignmentThreshold = 0.9f;
            if (alignmentUp < alignmentThreshold)
            {
                // Use user-provided up-vector: project onto plane orthogonal to tangent
                var n = Vector3.Cross(Vector3.Cross(tangent, up), tangent);
                if (n.LengthSquared() >= SingularThreshold)
                    return Vector3.Normalize(n);
            }
            // Fallback: choose a stable world-up vector not parallel to tangent
            Vector3 worldUp = Vector3.UnitY;
            if (Math.Abs(Vector3.Dot(worldUp, tangent) / tangent.Length()) > alignmentThreshold)
                worldUp = Vector3.UnitX;
            // Project worldUp onto plane orthogonal to tangent
            var fallback = Vector3.Cross(Vector3.Cross(tangent, worldUp), tangent);
            if (fallback.LengthSquared() < SingularThreshold)
            {
                // Last resort: any perpendicular direction
                fallback = Vector3.Cross(tangent, new Vector3(1, 0, 0));
            }
            return Vector3.Normalize(fallback);
        }

        /// <summary>
        /// Solves the 2x2 linear system for Hermite tangent scale factors alpha0, alpha1
        /// by assembling normal equations from least-squares over the point set.
        /// Uses default estimates if the system is singular.
        /// </summary>
        private static void SolveTangentScales(
            List<PointData> pts,
            PointData p0,
            PointData pN,
            Vector3 dir0,
            Vector3 dirN,
            float dt,
            out float alpha0,
            out float alpha1)
        {
            // Accumulate normal matrix entries and right-hand terms
            float S00 = 0f, S01 = 0f, S11 = 0f;
            float B0 = 0f, B1 = 0f;
            foreach (var pd in pts)
            {
                float t = (pd.Time - p0.Time) / dt;
                float h10 = HermiteH10(t), h11 = HermiteH11(t);
                var c0 = dir0 * h10;
                var c1 = dirN * h11;
                var basePos = p0.Position * HermiteH00(t)
                            + pN.Position * HermiteH01(t);
                var diff = pd.Position - basePos;
                S00 += Vector3.Dot(c0, c0);
                S01 += Vector3.Dot(c0, c1);
                S11 += Vector3.Dot(c1, c1);
                B0 += Vector3.Dot(c0, diff);
                B1 += Vector3.Dot(c1, diff);
            }
            float det = S00 * S11 - S01 * S01;
            if (Math.Abs(det) < SingularThreshold)
            {
                // Solve failed: use default chord-based scales
                alpha0 = Vector3.Distance(pts[0].Position, pts[1].Position) * DefaultAlphaScale;
                alpha1 = Vector3.Distance(pts[^1].Position, pts[^2].Position) * DefaultAlphaScale;
            }
            else
            {
                // Invert normal system
                alpha0 = (B0 * S11 - B1 * S01) / det;
                alpha1 = (-B0 * S01 + B1 * S00) / det;
            }
        }

        /// <summary>
        /// Computes the absolute angle between two vectors (0 to π).
        /// </summary>
        private static float AngleBetween(Vector3 a, Vector3 b)
        {
            float cos = Math.Clamp(Vector3.Dot(a, b) / (a.Length() * b.Length()), -1f, 1f);
            return (float)Math.Acos(cos);
        }

        /// <summary>
        /// Computes curvature at the center of a three-point arc via
        /// the radius of the circumscribed circle.
        /// </summary>
        private static float EstimateCurvature(Vector3 pA, Vector3 pB, Vector3 pC)
        {
            var a = pB - pA;
            var b = pC - pB;
            float area2 = Vector3.Cross(a, b).Length();
            float denom = a.Length() * b.Length() * Vector3.Distance(pA, pC);
            if (area2 < SingularThreshold || denom < SingularThreshold)
                return 0f;
            float radius = denom / (2f * area2);
            return radius > 1e6f ? 0f : 1f / radius;
        }

        /// <summary>
        /// Fallback polyline: constructs zero-curvature PH segments connecting
        /// each consecutive pair of points when recursion limit is reached.
        /// </summary>
        private static List<PHCurve3D> FallbackPolyline(List<PointData> pts)
        {
            var list = new List<PHCurve3D>(pts.Count - 1);
            for (int i = 0; i < pts.Count - 1; ++i)
            {
                var h0 = new HermiteControlPoint3D(
                    pts[i].Position,
                    Vector3.Zero,
                    curvature: 0f,
                    principalNormal: Vector3.UnitY);
                var h1 = new HermiteControlPoint3D(
                    pts[i + 1].Position,
                    Vector3.Zero,
                    curvature: 0f,
                    principalNormal: Vector3.UnitY);
                list.Add(PHCurveFactory.CreateQuintic(
                    h0, h1,
                    pts[i].Time, pts[i + 1].Time));
            }
            return list;
        }

        #region Quintic Hermite Basis Functions
        /// <summary>Hermite basis H00(t) for quintic interpolation.</summary>
        private static float HermiteH00(float t)
        {
            float t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
            return 1f - 10f * t3 + 15f * t4 - 6f * t5;
        }
        /// <summary>Hermite basis H01(t) for quintic interpolation.</summary>
        private static float HermiteH01(float t)
        {
            float t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
            return 10f * t3 - 15f * t4 + 6f * t5;
        }
        /// <summary>Hermite basis H10(t) for quintic interpolation.</summary>
        private static float HermiteH10(float t)
        {
            float t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
            return t - 6f * t3 + 8f * t4 - 3f * t5;
        }
        /// <summary>Hermite basis H11(t) for quintic interpolation.</summary>
        private static float HermiteH11(float t)
        {
            float t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
            return -4f * t3 + 7f * t4 - 3f * t5;
        }
        #endregion

        #endregion
    }

    /// <summary>
    /// Extension for <see cref="float"/> to allow functional-style usage of Abs().
    /// </summary>
    static class FloatExtensions
    {
        public static float Abs(this float v) => Math.Abs(v);
    }
}
