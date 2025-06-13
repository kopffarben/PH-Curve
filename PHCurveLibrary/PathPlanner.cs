// PathPlanner.cs
//
// References:
// Albrecht & Farouki (1996): Homotopy Methods for PH Splines
//
using System.Collections.Generic;

namespace PHCurveLibrary
{
    /// <summary>
    /// Builds a multi-segment path composed of <see cref="PHCurve3D"/> segments.
    /// Each segment is created from successive Hermite control points using
    /// <see cref="PHCurveFactory.CreateQuintic(PHCurveLibrary.HermiteControlPoint3D, PHCurveLibrary.HermiteControlPoint3D)"/>.
    /// </summary>
    public class PathPlanner
    {
        private readonly List<PHCurve3D> segments = new();

        /// <summary>
        /// Add a new curve segment defined by two Hermite control points.
        /// </summary>
        /// <param name="start">Start Hermite data.</param>
        /// <param name="end">End Hermite data.</param>
        public void AddSegment(HermiteControlPoint3D start, HermiteControlPoint3D end)
        {
            PHCurve3D curve = PHCurveFactory.CreateQuintic(start, end);
            segments.Add(curve);
        }

        /// <summary>
        /// Build the complete path consisting of all added segments.
        /// </summary>
        /// <returns>List of PH curve segments.</returns>
        public List<PHCurve3D> BuildPath()
        {
            return new List<PHCurve3D>(segments);
        }

        /// <summary>
        /// Validate <c>G²</c> continuity between successive segments of the path.
        /// </summary>
        /// <param name="tolerance">Comparison tolerance.</param>
        /// <returns><c>true</c> if all joins satisfy <c>G²</c> continuity.</returns>
        public bool ValidatePathG2(float tolerance = 1e-4f)
        {
            for (int i = 0; i < segments.Count - 1; i++)
            {
                if (!PHCurveFactory.ValidateG2(segments[i], segments[i + 1], tolerance))
                {
                    return false;
                }
            }

            return true;
        }
    }
}
