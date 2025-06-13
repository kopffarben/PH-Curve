// PHCurveFactory.cs
//
// References:
// Farouki & Dong (2012): PHquintic Library
// Jaklić et al. (2015): G² Quintic PH Interpolation
//
using System.Numerics;
using MathNet.Numerics.LinearAlgebra;

namespace PHCurveLibrary
{
    /// <summary>
    /// Factory methods for constructing <see cref="PHCurve3D"/> segments from
    /// Hermite data. The underlying system solves for the remaining hodograph
    /// coefficients so that the curve matches position, tangent and curvature
    /// at both endpoints.
    /// </summary>
    public static class PHCurveFactory
    {
        /// <summary>
        /// Create a quintic PH curve satisfying <c>G²</c> Hermite conditions.
        /// </summary>
        /// <param name="p0">Start Hermite control point.</param>
        /// <param name="p1">End Hermite control point.</param>
        public static PHCurve3D CreateQuintic(HermiteControlPoint3D p0, HermiteControlPoint3D p1)
        {
            Vector3 A = p0.Tangent;
            Vector3 T1 = p1.Tangent;
            Vector3 B = p0.PrincipalNormal * (p0.Curvature * A.LengthSquared());

            Vector3 deltaP = p1.Position - p0.Position;
            Vector3 P = deltaP - A - B * 0.5f;
            Vector3 Tan = T1 - A - B;
            Vector3 K1 = p1.PrincipalNormal * (p1.Curvature * T1.LengthSquared());

            var M3 = Matrix<float>.Build.DenseOfArray(new float[,]
            {
                {1f/3f, 1f/4f, 1f/5f},
                {1f,    1f,    1f   },
                {2f,    3f,    4f   }
            });
            var Coefs = M3.Inverse() * Matrix<float>.Build.DenseOfArray(new float[,]
            {
                {P.X, P.Y, P.Z},
                {Tan.X, Tan.Y, Tan.Z},
                {K1.X, K1.Y, K1.Z}
            });

            Vector3 C = new(Coefs[0, 0], Coefs[0, 1], Coefs[0, 2]);
            Vector3 D = new(Coefs[1, 0], Coefs[1, 1], Coefs[1, 2]);
            Vector3 E = new(Coefs[2, 0], Coefs[2, 1], Coefs[2, 2]);

            return new PHCurve3D(A, B, C, D, E);
        }

        /// <summary>
        /// Validate <c>G²</c> continuity between two segments by comparing
        /// position, tangent and principal normals at the junction.
        /// </summary>
        /// <param name="a">First segment.</param>
        /// <param name="b">Second segment.</param>
        /// <param name="tol">Tolerance for comparisons.</param>
        public static bool ValidateG2(in PHCurve3D a, in PHCurve3D b, float tol = 1e-4f)
        {
            if (Vector3.Distance(a.Position(1f), b.Position(0f)) > tol) return false;
            if (Vector3.Cross(a.TangentUnit(1f), b.TangentUnit(0f)).Length() > tol) return false;
            if (Vector3.Cross(a.PrincipalNormal(1f), b.PrincipalNormal(0f)).Length() > tol) return false;
            return true;
        }
    }
}
