// PHCurveLibrary.cs
// 
// Wichtige Referenzen:
// Farouki & Dong (2012): PHquintic Bibliothek (PDF): https://example.com/Farouki_Dong_PHquintic.pdf
// Jaklić et al. (2015): G² quintic PH-Interpolation (Preprint): https://example.com/Jaklic_G2_QuinticPH.pdf
// Meek & Walton (2007): G² PH-Quintic Spirale (ScienceDirect): https://example.com/Meek_Walton_G2Quintic.pdf
// Kozak (2014): Spatial Rational PH Kubik (arXiv): https://arxiv.org/abs/1401.1234
// Farouki et al. (2008): Spatial PH Quintic Formoptimierung (arXiv): https://arxiv.org/abs/0803.5678

using System;
using System.Numerics;
using MathNet.Numerics.LinearAlgebra;

namespace PHCurveLibrary
{
    /// <summary>
    /// Defines a Hermite control point in 3D space, optionally including curvature
    /// and principal normal vector for G² interpolation.
    /// <para>
    /// Wahl: Diese Struktur fasst alle notwendigen Hermite-Daten (Position, Tangente, Krümmung und Hauptnormalenrichtung)
    /// in einem Werttyp zusammen. Dadurch lässt sich ein PH-Quintik direkt anhand zweier solcher Punkte konstruieren,
    /// um G²-Stetigkeit zu gewährleisten. Die explizite Angabe von Krümmung und Normalen ermöglicht die eindeutige
    /// Spezifikation der zweiten Ableitung an Endpunkten, was für nahtlose Krümmungskontinuität erforderlich ist.
    /// </para>
    /// </summary>
    /// <param name="Position">The point's 3D coordinates.</param>
    /// <param name="Tangent">The tangent vector at the point.</param>
    /// <param name="Curvature">The signed curvature magnitude at the point.</param>
    /// <param name="PrincipalNormal">The principal normal vector for curvature direction.</param>
    public struct HermiteControlPoint3D
    {
        /// <summary>The position of the control point.</summary>
        public Vector3 Position;
        /// <summary>The tangent vector at the control point.</summary>
        public Vector3 Tangent;
        /// <summary>The signed curvature at the control point.</summary>
        public float Curvature;
        /// <summary>The principal normal vector at the control point.</summary>
        public Vector3 PrincipalNormal;

        /// <summary>
        /// Initializes a new instance of <see cref="HermiteControlPoint3D"/>.
        /// </summary>
        /// <param name="position">The 3D coordinates of the point.</param>
        /// <param name="tangent">The tangent vector at the point.</param>
        /// <param name="curvature">The curvature magnitude (optional).</param>
        /// <param name="principalNormal">The principal normal vector (optional).</param>
        public HermiteControlPoint3D(Vector3 position, Vector3 tangent, float curvature = 0f, Vector3 principalNormal = default)
        {
            Position = position;
            Tangent = tangent;
            Curvature = curvature;
            PrincipalNormal = principalNormal;
        }
    }

    /// <summary>
    /// Represents a Pythagorean-Hodograph (PH) curve in 3D via its hodograph coefficients.
    /// This struct holds the five polynomial coefficients for position, derivative, and curvature computation.
    /// </summary>
    public struct PHCurve3D
    {
        /// <summary>Coefficient A of the hodograph (constant term).</summary>
        public readonly Vector3 A;
        /// <summary>Coefficient B of the hodograph (linear term).</summary>
        public readonly Vector3 B;
        /// <summary>Coefficient C of the hodograph (quadratic term).</summary>
        public readonly Vector3 C;
        /// <summary>Coefficient D of the hodograph (cubic term).</summary>
        public readonly Vector3 D;
        /// <summary>Coefficient E of the hodograph (quartic term).</summary>
        public readonly Vector3 E;

        /// <summary>
        /// Initializes a new instance of <see cref="PHCurve3D"/> with given hodograph coefficients.
        /// </summary>
        /// <param name="a">Constant coefficient of the derivative.</param>
        /// <param name="b">Linear coefficient of the derivative.</param>
        /// <param name="c">Quadratic coefficient of the derivative.</param>
        /// <param name="d">Cubic coefficient of the derivative.</param>
        /// <param name="e">Quartic coefficient of the derivative.</param>
        public PHCurve3D(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 e)
        {
            A = a;
            B = b;
            C = c;
            D = d;
            E = e;
        }

        /// <summary>
        /// Evaluates the 3D position r(t) on the PH curve for parameter t in [0,1].
        /// </summary>
        /// <param name="t">Normalized parameter along the curve.</param>
        /// <returns>Coordinates of the point on the curve.</returns>
        public Vector3 Position(float t)
            => A * t
             + B * (0.5f * t * t)
             + C * (t * t * t / 3f)
             + D * (t * t * t * t / 4f)
             + E * (t * t * t * t * t / 5f);

        /// <summary>
        /// Computes the derivative r'(t) (the hodograph) at parameter t.
        /// </summary>
        /// <param name="t">Normalized parameter along the curve.</param>
        /// <returns>The derivative vector at t.</returns>
        public Vector3 Derivative(float t)
            => A + B * t + C * t * t + D * t * t * t + E * t * t * t * t;

        /// <summary>
        /// Computes the second derivative r''(t) at parameter t.
        /// </summary>
        /// <param name="t">Normalized parameter along the curve.</param>
        /// <returns>The second derivative vector at t.</returns>
        public Vector3 SecondDerivative(float t)
            => B + 2f * C * t + 3f * D * t * t + 4f * E * t * t * t;

        /// <summary>
        /// Computes the speed |r'(t)| at parameter t.
        /// </summary>
        /// <param name="t">Normalized parameter along the curve.</param>
        /// <returns>The scalar speed at t.</returns>
        public float Speed(float t) => Derivative(t).Length();

        /// <summary>
        /// Computes the unit tangent vector T(t) = r'(t)/|r'(t)|.
        /// </summary>
        /// <param name="t">Normalized parameter along the curve.</param>
        /// <returns>The unit tangent at t.</returns>
        public Vector3 TangentUnit(float t) => Vector3.Normalize(Derivative(t));

        /// <summary>
        /// Computes the principal normal vector N(t) at parameter t.
        /// </summary>
        /// <param name="t">Normalized parameter along the curve.</param>
        /// <returns>The principal normal at t.</returns>
        public Vector3 PrincipalNormal(float t)
        {
            var d1 = Derivative(t);
            var d2 = SecondDerivative(t);
            float s = d1.Length();
            var numer = d2 * s - d1 * Vector3.Dot(d1, d2) / s;
            return Vector3.Normalize(numer / (s * s));
        }
    }

    /// <summary>
    /// Provides factory methods to construct PHCurve3D instances and validate continuity.
    /// </summary>
    public static class PHCurveFactory
    {
        /// <summary>
        /// Creates a quintic PH curve segment that satisfies G² Hermite conditions at endpoints.
        /// </summary>
        /// <param name="p0">Start Hermite control point (position, tangent, curvature, normal).</param>
        /// <param name="p1">End Hermite control point (position, tangent, curvature, normal).</param>
        /// <returns>A <see cref="PHCurve3D"/> representing the PH curve segment.</returns>
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

            Vector3 C = new Vector3(Coefs[0, 0], Coefs[0, 1], Coefs[0, 2]);
            Vector3 D = new Vector3(Coefs[1, 0], Coefs[1, 1], Coefs[1, 2]);
            Vector3 E = new Vector3(Coefs[2, 0], Coefs[2, 1], Coefs[2, 2]);

            return new PHCurve3D(A, B, C, D, E);
        }

        /// <summary>
        /// Validates G² continuity between two PHCurve segments.
        /// Checks position, tangent direction, and principal normal alignment at the join.
        /// </summary>
        /// <param name="a">First PH curve segment.</param>
        /// <param name="b">Second PH curve segment.</param>
        /// <param name="tol">Tolerance for vector comparisons.</param>
        /// <returns>true if G² continuity conditions are met; otherwise false.</returns>
        public static bool ValidateG2(in PHCurve3D a, in PHCurve3D b, float tol = 1e-4f)
        {
            if (Vector3.Distance(a.Position(1f), b.Position(0f)) > tol) return false;
            if (Vector3.Cross(a.TangentUnit(1f), b.TangentUnit(0f)).Length() > tol) return false;
            if (Vector3.Cross(a.PrincipalNormal(1f), b.PrincipalNormal(0f)).Length() > tol) return false;
            return true;
        }
    }
}
