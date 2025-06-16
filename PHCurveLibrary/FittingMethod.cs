namespace PHCurveLibrary
{
    /// <summary>
    /// Enumerates the available curve fitting strategies.
    /// </summary>
    public enum FittingMethod
    {
        /// <summary>Local Hermite interpolation.</summary>
        LocalHermite,

        /// <summary>Global least-squares approximation.</summary>
        LeastSquares,

        /// <summary>Evolution based fitting.</summary>
        Evolution,

        /// <summary>Homotopy continuation approach.</summary>
        Homotopy,

        /// <summary>Heuristic segmentation with post smoothing.</summary>
        Heuristic
    }
}
