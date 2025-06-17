using System.Collections.Generic;

namespace PHCurveLibrary.Fitting
{
    /// <summary>
    /// Placeholder implementation forwarding to <see cref="LocalHermiteFitter"/>.
    /// </summary>
    public static class HomotopyFitter
    {
        /// <summary>
        /// Fit points using a homotopy continuation approach.
        /// </summary>
        public static List<PHCurve3D> Fit(
            List<PointData> points,
            float positionTolerance,
            float orientationTolerance)
        {
            return LocalHermiteFitter.Fit(points, positionTolerance, orientationTolerance);
        }

        /// <summary>
        /// Incremental homotopy fitting.
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
