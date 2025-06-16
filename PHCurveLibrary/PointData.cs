using System.Numerics;

namespace PHCurveLibrary
{
    /// <summary>
    /// Represents a measured sample point with an up vector and time stamp.
    /// </summary>
    public struct PointData
    {
        /// <summary>3D position of the point.</summary>
        public Vector3 Position;

        /// <summary>Measured up vector at the sample location.</summary>
        public Vector3 UpVector;

        /// <summary>Absolute time value of the sample.</summary>
        public float Time;

        /// <summary>
        /// Initializes a new instance of the <see cref="PointData"/> struct.
        /// </summary>
        /// <param name="position">Spatial position.</param>
        /// <param name="upVector">Measured up vector.</param>
        /// <param name="time">Absolute time stamp.</param>
        public PointData(Vector3 position, Vector3 upVector, float time)
        {
            Position = position;
            UpVector = upVector;
            Time = time;
        }
    }
}
