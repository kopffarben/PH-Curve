using System;
using System.Numerics;
using MathNet.Numerics.LinearAlgebra;

namespace CubicPHCurve
{
    public class CubicPHCurve3D
    {
        // Polynomial coefficients for r'(t) = A + B t + C t^2 + D t^3 + E t^4
        private readonly Vector3 A, B, C, D, E;
        // Start position of the curve
        private readonly Vector3 P0;
        // Quaternion representation of the unit X basis vector
        private static readonly Quaternion BasisI = new(1f, 0f, 0f, 0f);

        /// <summary>
        /// Hermite control point used to construct the curve.
        /// </summary>
        public struct ControlPoint
        {
            /// <summary>Position of the point.</summary>
            public Vector3 Position;
            /// <summary>Tangent (derivative) at the point.</summary>
            public Vector3 Tangent;
            /// <summary>Optional normal at the point.</summary>
            public Vector3 Normal;
            /// <summary>Optional curvature value.</summary>
            public float Curvature;
            /// <summary>Absolute time parameter of the point.</summary>
            public float Time;

            /// <summary>
            /// Initializes a new instance of the <see cref="ControlPoint"/> struct.
            /// </summary>
            /// <param name="pos">Point position.</param>
            /// <param name="tan">Tangent vector.</param>
            /// <param name="normal">Optional normal vector.</param>
            /// <param name="curvature">Optional curvature.</param>
            /// <param name="time">Absolute time parameter.</param>
            public ControlPoint(Vector3 pos, Vector3 tan, Vector3 normal = default, float curvature = 0f, float time = 0f)
            {
                Position = pos;
                Tangent = tan;
                Normal = normal;
                Curvature = curvature;
                Time = time;
            }
        }

        private static Vector3 V(Quaternion q) => new(q.X, q.Y, q.Z);

        private static Quaternion Scale(Quaternion q, float s) => new(q.X * s, q.Y * s, q.Z * s, q.W * s);

        private static Quaternion FrameQuaternion(Vector3 tangent, Vector3 normal)
        {
            Vector3 T = Vector3.Normalize(tangent);
            Vector3 N = Vector3.Normalize(normal);
            Vector3 B = Vector3.Normalize(Vector3.Cross(T, N));
            var rot = new Matrix4x4(
                T.X, T.Y, T.Z, 0f,
                N.X, N.Y, N.Z, 0f,
                B.X, B.Y, B.Z, 0f,
                0f, 0f, 0f, 1f);
            return Quaternion.CreateFromRotationMatrix(rot);
        }

        private static Quaternion QuaternionFromDerivative(Vector3 tangent, Vector3 normal)
        {
            float len = tangent.Length();
            if (len < 1e-8f)
                return Quaternion.Identity;
            Quaternion rot = FrameQuaternion(tangent, normal == Vector3.Zero ? Vector3.UnitY : normal);
            return Scale(rot, MathF.Sqrt(len));
        }

        /// <summary>
        /// Create a PH curve segment directly from two Hermite control points
        /// by computing derivative polynomial coefficients algebraically.
        /// </summary>
        public static CubicPHCurve3D FromControlPoints(ControlPoint cp0, ControlPoint cp1)
        {
            Vector3 delta = cp1.Position - cp0.Position;
            Vector3 A = cp0.Tangent;
            Vector3 S = cp1.Tangent - cp0.Tangent;
            Vector3 T = delta - cp0.Tangent;
            Vector3 B = 6f * T - 2f * S;
            Vector3 C = 3f * S - 6f * T;
            return new CubicPHCurve3D(A, B, C, Vector3.Zero, Vector3.Zero, cp0.Position);
        }

        /// <summary>
        /// Constructor from polynomial derivative coefficients
        /// </summary>
        public CubicPHCurve3D(Vector3 A, Vector3 B, Vector3 C, Vector3 D, Vector3 E, Vector3 start)
        {
            this.A = A;
            this.B = B;
            this.C = C;
            this.D = D;
            this.E = E;
            this.P0 = start;
        }

        /// <summary>
        /// Constructor from polynomial derivative coefficients with start position at the origin.
        /// </summary>
        public CubicPHCurve3D(Vector3 A, Vector3 B, Vector3 C, Vector3 D, Vector3 E)
            : this(A, B, C, D, E, Vector3.Zero)
        {
        }

        /// <summary>
        /// Compute PH segment coefficients via quaternionic formulation taking
        /// G² constraints (tangent, normal and curvature) at both end points
        /// into account.
        /// </summary>
        private static CubicPHCurve3D ComputePolynomialCoefficientsQuaternion(ControlPoint c0, ControlPoint c1)
        {
            Vector3 t0 = c0.Tangent;
            Vector3 t1 = c1.Tangent;

            Quaternion q0 = QuaternionFromDerivative(t0, c0.Normal);
            Quaternion q1 = QuaternionFromDerivative(t1, c1.Normal);

            Vector3 k0Vec = c0.Curvature * t0.LengthSquared() * (c0.Normal == Vector3.Zero ? Vector3.UnitY : Vector3.Normalize(c0.Normal));
            Vector3 k1Vec = c1.Curvature * t1.LengthSquared() * (c1.Normal == Vector3.Zero ? Vector3.UnitY : Vector3.Normalize(c1.Normal));

            Quaternion[] basis = new[]
            {
                new Quaternion(1f,0f,0f,0f),
                new Quaternion(0f,1f,0f,0f),
                new Quaternion(0f,0f,1f,0f),
                new Quaternion(0f,0f,0f,1f)
            };

            var M = Matrix<float>.Build.Dense(6,4);
            for (int j=0;j<4;++j)
            {
                Quaternion e = basis[j];
                Vector3 col0 = V(e*BasisI*Quaternion.Conjugate(q0) + q0*BasisI*Quaternion.Conjugate(e));
                Vector3 col1 = V(-e*BasisI*Quaternion.Conjugate(q1) - q1*BasisI*Quaternion.Conjugate(e));
                M[0,j] = col0.X; M[1,j]=col0.Y; M[2,j]=col0.Z;
                M[3,j] = col1.X; M[4,j]=col1.Y; M[5,j]=col1.Z;
            }

            Quaternion K = Scale(q1, 2f) - Scale(q0, 2f);
            Vector3 const1 = V(K*BasisI*Quaternion.Conjugate(q1) + q1*BasisI*Quaternion.Conjugate(K));

            var b = MathNet.Numerics.LinearAlgebra.Vector<float>.Build.Dense(new float[]
            {
                k0Vec.X*0.5f, k0Vec.Y*0.5f, k0Vec.Z*0.5f,
                k1Vec.X*0.5f - const1.X, k1Vec.Y*0.5f - const1.Y, k1Vec.Z*0.5f - const1.Z
            });

            MathNet.Numerics.LinearAlgebra.Vector<float> x = M.Svd(true).Solve(b);

            Quaternion qd1 = new(x[0], x[1], x[2], x[3]);
            Quaternion qd2 = q1 - q0 - qd1;

            var coeffs = DerivativeCoefficientsFromQuaternions(q0, qd1, qd2);
            return new CubicPHCurve3D(coeffs.A, coeffs.B, coeffs.C, coeffs.D, coeffs.E);
        }

        private static (Vector3 A, Vector3 B, Vector3 C, Vector3 D, Vector3 E) DerivativeCoefficientsFromQuaternions(Quaternion q0, Quaternion q1, Quaternion q2)
        {
            Quaternion c0 = Quaternion.Conjugate(q0);
            Quaternion c1 = Quaternion.Conjugate(q1);
            Quaternion c2 = Quaternion.Conjugate(q2);

            Vector3 A = V(q0*BasisI*c0);
            Vector3 B = V(q0*BasisI*c1 + q1*BasisI*c0);
            Vector3 C = V(q0*BasisI*c2 + q1*BasisI*c1 + q2*BasisI*c0);
            Vector3 D = V(q1*BasisI*c2 + q2*BasisI*c1);
            Vector3 E = V(q2*BasisI*c2);
            return (A,B,C,D,E);
        }

        /// <summary>Hodograph r'(t)</summary> r'(t)</summary>
        public Vector3 Derivative(float t) => ((((E * t + D) * t + C) * t + B) * t + A);
        /// <summary>Speed = ||r'(t)||</summary>
        public float Speed(float t) => Derivative(t).Length();
        /// <summary>Curvature of the curve.</summary>
        public float Curvature(float t)
        {
            Vector3 d1 = Derivative(t);
            Vector3 d2 = SecondDerivative(t);
            Vector3 cross = Vector3.Cross(d1, d2);
            float len = d1.Length();
            if (len < 1e-8f)
                return 0f;
            return cross.Length() / (len * len * len);
        }
        /// <summary>Unit tangent T(t)</summary>
        public Vector3 Tangent(float t) => Vector3.Normalize(Derivative(t));
        /// <summary>Second derivative r''(t)</summary>
        public Vector3 SecondDerivative(float t) => B + 2f * C * t + 3f * D * t * t + 4f * E * t * t * t;
        /// <summary>Principal normal N(t)</summary>
        public Vector3 Normal(float t)
        {
            var d1 = Derivative(t);
            var d2 = SecondDerivative(t);
            float s = d1.Length();
            var num = d2 * s - d1 * (Vector3.Dot(d1, d2) / s);
            return Vector3.Normalize(num / (s * s));
        }
        /// <summary>Bi-tangent B(t) = T×N</summary>
        public Vector3 BiTangent(float t) => Vector3.Cross(Tangent(t), Normal(t));
        /// <summary>Offset point = r(t) + d N(t)</summary>
        public Vector3 OffsetPoint(float t, float d) => Position(t) + d * Normal(t);
        /// <summary>Position r(t) = P0 + ∫₀ᵗ r'(u)du</summary>
        public Vector3 Position(float t) =>
            P0 + A * t + B * (t * t / 2f) + C * (t * t * t / 3f) + D * (t * t * t * t / 4f) + E * (t * t * t * t * t / 5f);

        /// <summary>Velocity at absolute time T within [T0,T1].</summary>
        public Vector3 VelocityAtTime(float T, float T0, float T1)
        {
            float tParam = (T - T0) / (T1 - T0);
            Vector3 dr_dtp = Derivative(tParam);
            float invDur = 1.0f / (T1 - T0);
            return dr_dtp * invDur;
        }

        /// <summary>Speed at absolute time T within [T0,T1].</summary>
        public float SpeedAtTime(float T, float T0, float T1) => VelocityAtTime(T, T0, T1).Length();
    }
}
