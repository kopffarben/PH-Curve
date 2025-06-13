// PHCurve3D.cs
//
// References:
// Farouki & Dong (2012): PHquintic Library
// Jaklić et al. (2015): G² Quintic PH Interpolation
//
using System.Numerics;

namespace PHCurveLibrary
{
    /// <summary>
    /// Represents a spatial quintic Pythagorean Hodograph (PH) curve. The
    /// derivative of the curve is a polynomial r'(t) = A + Bt + Ct² + Dt³ + Et´
    /// whose squared norm is itself a polynomial. This property enables exact
    /// arc-length evaluation and simple offsetting.
    /// </summary>
    public struct PHCurve3D
    {
        /// <summary>Constant coefficient A of the hodograph.</summary>
        public readonly Vector3 A;

        /// <summary>Linear coefficient B of the hodograph.</summary>
        public readonly Vector3 B;

        /// <summary>Quadratic coefficient C of the hodograph.</summary>
        public readonly Vector3 C;

        /// <summary>Cubic coefficient D of the hodograph.</summary>
        public readonly Vector3 D;

        /// <summary>Quartic coefficient E of the hodograph.</summary>
        public readonly Vector3 E;

        /// <summary>
        /// Creates a PH curve from derivative coefficients.
        /// </summary>
        public PHCurve3D(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 e)
        {
            A = a;
            B = b;
            C = c;
            D = d;
            E = e;
        }

        /// <summary>
        /// Evaluate the curve position r(t) by integrating the hodograph.
        /// </summary>
        /// <param name="t">Normalized parameter in [0,1].</param>
        public Vector3 Position(float t)
            => A * t
             + B * (0.5f * t * t)
             + C * (t * t * t / 3f)
             + D * (t * t * t * t / 4f)
             + E * (t * t * t * t * t / 5f);

        /// <summary>
        /// Compute the first derivative r'(t).
        /// </summary>
        /// <param name="t">Normalized parameter.</param>
        public Vector3 Derivative(float t)
            => A + B * t + C * t * t + D * t * t * t + E * t * t * t * t;

        /// <summary>
        /// Compute the second derivative r''(t).
        /// </summary>
        /// <param name="t">Normalized parameter.</param>
        public Vector3 SecondDerivative(float t)
            => B + 2f * C * t + 3f * D * t * t + 4f * E * t * t * t;

        /// <summary>
        /// Compute the speed |r'(t)|.
        /// </summary>
        public float Speed(float t) => Derivative(t).Length();

        /// <summary>
        /// Unit tangent vector T(t) = r'(t) / |r'(t)|.
        /// </summary>
        public Vector3 TangentUnit(float t) => Vector3.Normalize(Derivative(t));

        /// <summary>
        /// Principal normal vector computed from derivative and second derivative.
        /// </summary>
        public Vector3 PrincipalNormal(float t)
        {
            var d1 = Derivative(t);
            var d2 = SecondDerivative(t);
            float s = d1.Length();
            var numer = d2 * s - d1 * Vector3.Dot(d1, d2) / s;
            return Vector3.Normalize(numer / (s * s));
        }

        /// <summary>
        /// Curvature &kappa;(t) = ||r'(t) × r''(t)|| / ||r'(t)||³.
        /// </summary>
        /// <param name="t">Normalized parameter.</param>
        /// <returns>The unsigned curvature magnitude.</returns>
        public float Curvature(float t)
        {
            Vector3 d1 = Derivative(t);
            Vector3 d2 = SecondDerivative(t);
            Vector3 cross = Vector3.Cross(d1, d2);
            float len = d1.Length();
            if (len < 1e-8f)
            {
                return 0f;
            }

            return cross.Length() / (len * len * len);
        }

        /// <summary>
        /// Normalized tangent vector &lt;see cref="TangentUnit(float)"/&gt;.
        /// </summary>
        /// <param name="t">Normalized parameter.</param>
        /// <returns>The unit tangent vector.</returns>
        public Vector3 Tangent(float t) => TangentUnit(t);

        /// <summary>
        /// Unit principal normal vector &lt;see cref="PrincipalNormal(float)"/&gt;.
        /// </summary>
        /// <param name="t">Normalized parameter.</param>
        /// <returns>The unit principal normal.</returns>
        public Vector3 Normal(float t) => PrincipalNormal(t);

        /// <summary>
        /// Bi-tangent vector B(t) = T(t) × N(t).
        /// </summary>
        /// <param name="t">Normalized parameter.</param>
        /// <returns>The unit bi-tangent vector.</returns>
        public Vector3 BiTangent(float t) => Vector3.Cross(Tangent(t), Normal(t));
    }
}
