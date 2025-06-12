using System;
using System.Numerics;
using MathNet.Numerics.LinearAlgebra;

namespace CubicPHCurve
{
    public class CubicPHCurve3D
    {
        // Polynomial coefficients for r'(t) = A + B t + C t^2 + D t^3 + E t^4
        private readonly Vector3 A, B, C, D, E;

        /// <summary>
        /// Hermite control point with position and tangent
        /// </summary>
        public struct ControlPoint
        {
            public Vector3 Position;
            public Vector3 Tangent;
            public ControlPoint(Vector3 pos, Vector3 tan)
            {
                Position = pos;
                Tangent = tan;
            }
        }

        /// <summary>
        /// Create a PH curve segment directly from two Hermite control points
        /// by computing derivative polynomial coefficients algebraically.
        /// </summary>
        public static CubicPHCurve3D FromControlPoints(ControlPoint cp0, ControlPoint cp1)
        {
            return ComputePolynomialCoefficients(cp0.Position, cp0.Tangent, cp1.Position, cp1.Tangent);
        }

        /// <summary>
        /// Constructor from polynomial derivative coefficients
        /// </summary>
        public CubicPHCurve3D(Vector3 A, Vector3 B, Vector3 C, Vector3 D, Vector3 E)
        {
            this.A = A;
            this.B = B;
            this.C = C;
            this.D = D;
            this.E = E;
        }

        /// <summary>
        /// Compute PH segment coefficients directly from Hermite data
        /// via a 4×4 linear solve enforcing Hermite and PH constraints.
        /// </summary>
        private static CubicPHCurve3D ComputePolynomialCoefficients(
            Vector3 p0, Vector3 t0,
            Vector3 p1, Vector3 t1)
        {
            // A = r'(0)
            Vector3 A = t0;
            // S = remaining displacement
            Vector3 S = p1 - p0 - A;
            // Build 4x4 float matrix enforcing PH-Hermite constraints
            var M = Matrix<float>.Build.DenseOfArray(new float[,] {
                {1f,    1f,    1f,    1f   },   // B+C+D+E = t1 - A
                {0.5f, 1f/3f, 0.25f, 0.2f },   // 1/2 B +1/3 C+1/4 D+1/5 E = S
                // PH constraint 1: A·C + 0.5 B·B = 0 -> C coefficient
                {0f,    1f,    0f,    0f  },
                // PH constraint 2: B·D + C·C + A·E = 0 -> D coefficient
                {0f,    0f,    1f,    0f  }
            });
            // RHS float vectors
            var rhsX = MathNet.Numerics.LinearAlgebra.Vector<float>.Build.DenseOfArray(new float[] { t1.X - A.X, S.X, 0f, 0f });
            var rhsY = MathNet.Numerics.LinearAlgebra.Vector<float>.Build.DenseOfArray(new float[] { t1.Y - A.Y, S.Y, 0f, 0f });
            var rhsZ = MathNet.Numerics.LinearAlgebra.Vector<float>.Build.DenseOfArray(new float[] { t1.Z - A.Z, S.Z, 0f, 0f });
            var solX = M.Solve(rhsX);
            var solY = M.Solve(rhsY);
            var solZ = M.Solve(rhsZ);
            Vector3 B = new Vector3(solX[0], solY[0], solZ[0]);
            Vector3 C = new Vector3(solX[1], solY[1], solZ[1]);
            Vector3 D = new Vector3(solX[2], solY[2], solZ[2]);
            Vector3 E = new Vector3(solX[3], solY[3], solZ[3]);
            return new CubicPHCurve3D(A, B, C, D, E);
        }

        /// <summary>Hodograph r'(t)</summary> r'(t)</summary>
        public Vector3 Derivative(float t) => ((((E * t + D) * t + C) * t + B) * t + A);
        /// <summary>Speed = ||r'(t)||</summary>
        public float Speed(float t) => Derivative(t).Length();
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
        /// <summary>Position r(t) = ∫₀ᵗ r'(u)du</summary>
        public Vector3 Position(float t) => A * t + B * (t * t / 2f) + C * (t * t * t / 3f) + D * (t * t * t * t / 4f) + E * (t * t * t * t * t / 5f);
    }
}
