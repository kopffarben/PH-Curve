using System.Collections.Generic;

namespace PHCurveLibrary.Fitting
{
    /// <summary>
    /// Placeholder implementation forwarding to <see cref="LocalHermiteFitter"/>.
    /// </summary>
    public static class EvolutionFitter
    {
        /// <summary>
        /// Fit points using an evolution based approach.
        /// </summary>
        public static List<PHCurve3D> Fit(
            List<PointData> points,
            float positionTolerance,
            float orientationTolerance)
        {
            return LocalHermiteFitter.Fit(points, positionTolerance, orientationTolerance);
        }

        /// <summary>
        /// Incremental evolution fitting.
        /// </summary>
        public static List<PHCurve3D> FitIncremental(
            List<PointData> buffer,
            float positionTolerance,
            float orientationTolerance)
        {
            return LocalHermiteFitter.FitIncremental(buffer, positionTolerance, orientationTolerance);
        }
    }
}
