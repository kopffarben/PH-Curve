using System;
using System.Numerics;

public class CubicPHCurve3D
{
    // q0, q1, q2: quaternion coefficients defining the PH-curve
    private readonly Quaternion q0, q1, q2;
    // Polynomial coefficients for the derivative: r'(t) = A + B t + C t^2 + D t^3 + E t^4
    private readonly Vector3 A, B, C, D, E;

    public CubicPHCurve3D(Quaternion q0, Quaternion q1, Quaternion q2)
    {
        this.q0 = q0;
        this.q1 = q1;
        this.q2 = q2;

        // Precompute quaternion products for derivative coefficients
        Quaternion a = q0, b = q1, c = q2;
        Quaternion ai = Quaternion.Multiply(a, new Quaternion(1, 0, 0, 0));
        Quaternion bi = Quaternion.Multiply(b, new Quaternion(1, 0, 0, 0));
        Quaternion ci = Quaternion.Multiply(c, new Quaternion(1, 0, 0, 0));
        Quaternion aConj = Quaternion.Conjugate(a);
        Quaternion bConj = Quaternion.Conjugate(b);
        Quaternion cConj = Quaternion.Conjugate(c);

        // Coefficient quaternions:
        Quaternion coeff0 = Quaternion.Multiply(ai, aConj);
        Quaternion coeff1 = Quaternion.Add(
            Quaternion.Multiply(ai, bConj),
            Quaternion.Multiply(bi, aConj)
        );
        Quaternion coeff2 = Quaternion.Add(
            Quaternion.Add(
                Quaternion.Multiply(ai, cConj),
                Quaternion.Multiply(bi, bConj)
            ),
            Quaternion.Multiply(ci, aConj)
        );
        Quaternion coeff3 = Quaternion.Add(
            Quaternion.Multiply(bi, cConj),
            Quaternion.Multiply(ci, bConj)
        );
        Quaternion coeff4 = Quaternion.Multiply(ci, cConj);

        // Store vector parts
        A = new Vector3(coeff0.X, coeff0.Y, coeff0.Z);
        B = new Vector3(coeff1.X, coeff1.Y, coeff1.Z);
        C = new Vector3(coeff2.X, coeff2.Y, coeff2.Z);
        D = new Vector3(coeff3.X, coeff3.Y, coeff3.Z);
        E = new Vector3(coeff4.X, coeff4.Y, coeff4.Z);
    }

    // First derivative r'(t)
    public Vector3 Derivative(float t)
    {
        return ((((E * t + D) * t + C) * t + B) * t + A);
    }

    // Speed = ||r'(t)||
    public float Speed(float t)
    {
        return Derivative(t).Length();
    }

    // Second derivative r''(t)
    public Vector3 SecondDerivative(float t)
    {
        return B + 2f * C * t + 3f * D * t * t + 4f * E * t * t * t;
    }

    // Unit tangent T(t)
    public Vector3 Tangent(float t)
    {
        return Vector3.Normalize(Derivative(t));
    }

    // Principal normal N(t)
    public Vector3 Normal(float t)
    {
        Vector3 d1 = Derivative(t);
        Vector3 d2 = SecondDerivative(t);
        float speed = d1.Length();
        Vector3 numerator = d2 * speed - d1 * (Vector3.Dot(d1, d2) / speed);
        return Vector3.Normalize(numerator / (speed * speed));
    }

    // Compute bi-tangent B(t) = T(t) × N(t)
    public Vector3 BiTangent(float t)
    {
        Vector3 T = Tangent(t);
        Vector3 N = Normal(t);
        return Vector3.Cross(T, N);
    }

    // Offset point r(t) + d * N(t) using analytic position
    public Vector3 OffsetPoint(float t, float d)
    {
        Vector3 point = Position(t);
        Vector3 N = Normal(t);
        return point + d * N;
    }

    // Analytic position r(t) = ∫0^t r'(u) du
    public Vector3 Position(float t)
    {
        return A * t
             + B * (t * t / 2f)
             + C * (t * t * t / 3f)
             + D * (t * t * t * t / 4f)
             + E * (t * t * t * t * t / 5f);
    }
}
