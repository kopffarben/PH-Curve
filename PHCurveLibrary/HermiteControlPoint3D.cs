// HermiteControlPoint3D.cs
//
// References:
// Farouki & Dong (2012): PHquintic Library
// Jaklić et al. (2015): G² Quintic PH Interpolation
//
using System.Numerics;

namespace PHCurveLibrary
{
    /// <summary>
    /// Describes a single Hermite boundary condition in three dimensions.
    /// Position and tangent are mandatory while curvature and principal normal
    /// allow constructing <see cref="PHCurve3D"/> segments with <c>G²</c> continuity.
    /// The curvature value represents the signed magnitude κ and the principal
    /// normal specifies the direction of <c>dT/ds</c>.
    /// </summary>
    public struct HermiteControlPoint3D
    {
        /// <summary>The point's coordinates.</summary>
        public Vector3 Position;

        /// <summary>The tangent vector at this point.</summary>
        public Vector3 Tangent;

        /// <summary>The signed curvature magnitude κ.</summary>
        public float Curvature;

        /// <summary>The unit principal normal pointing towards the center of curvature.</summary>
        public Vector3 PrincipalNormal;

        /// <summary>
        /// Initializes a new instance of the <see cref="HermiteControlPoint3D"/> struct.
        /// </summary>
        /// <param name="position">Point position in 3D.</param>
        /// <param name="tangent">First derivative direction.</param>
        /// <param name="curvature">Optional curvature magnitude.</param>
        /// <param name="principalNormal">Optional principal normal direction.</param>
        public HermiteControlPoint3D(Vector3 position, Vector3 tangent, float curvature = 0f, Vector3 principalNormal = default)
        {
            Position = position;
            Tangent = tangent;
            Curvature = curvature;
            PrincipalNormal = principalNormal;
        }
    }
}
