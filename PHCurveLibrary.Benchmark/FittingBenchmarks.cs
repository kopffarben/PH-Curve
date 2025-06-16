using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using System.Collections.Generic;
using System.Numerics;
using PHCurveLibrary;

namespace PHCurveLibrary.Tests
{
    /// <summary>
    /// BenchmarkDotNet benchmarks comparing the fitting algorithms.
    /// </summary>
    [MemoryDiagnoser]
    public class FittingBenchmarks
    {
        private List<PointData> points = new();

        [GlobalSetup]
        public void Setup()
        {
            for (int i = 0; i <= 1000; i++)
            {
                points.Add(new PointData(new Vector3(i, 0f, 0f), Vector3.UnitY, i));
            }
        }

        [Benchmark]
        public int LocalHermite()
        {
            var segs = PathPlanner.CurveFitting(points, 0.01f, 0.01f, FittingMethod.LocalHermite);
            return segs.Count;
        }

        [Benchmark]
        public int LeastSquares()
        {
            var segs = PathPlanner.CurveFitting(points, 0.01f, 0.01f, FittingMethod.LeastSquares);
            return segs.Count;
        }

        [Benchmark]
        public int Evolution()
        {
            var segs = PathPlanner.CurveFitting(points, 0.01f, 0.01f, FittingMethod.Evolution);
            return segs.Count;
        }

        [Benchmark]
        public int Homotopy()
        {
            var segs = PathPlanner.CurveFitting(points, 0.01f, 0.01f, FittingMethod.Homotopy);
            return segs.Count;
        }

        [Benchmark]
        public int Heuristic()
        {
            var segs = PathPlanner.CurveFitting(points, 0.01f, 0.01f, FittingMethod.Heuristic);
            return segs.Count;
        }

        [Benchmark]
        public int LocalHermiteIncremental()
        {
            var segs = PathPlanner.CurveFittingIncremental(points, 0.01f, 0.01f, FittingMethod.LocalHermite);
            return segs.Count;
        }

        [Benchmark]
        public int LeastSquaresIncremental()
        {
            var segs = PathPlanner.CurveFittingIncremental(points, 0.01f, 0.01f, FittingMethod.LeastSquares);
            return segs.Count;
        }

        [Benchmark]
        public int EvolutionIncremental()
        {
            var segs = PathPlanner.CurveFittingIncremental(points, 0.01f, 0.01f, FittingMethod.Evolution);
            return segs.Count;
        }

        [Benchmark]
        public int HomotopyIncremental()
        {
            var segs = PathPlanner.CurveFittingIncremental(points, 0.01f, 0.01f, FittingMethod.Homotopy);
            return segs.Count;
        }

        [Benchmark]
        public int HeuristicIncremental()
        {
            var segs = PathPlanner.CurveFittingIncremental(points, 0.01f, 0.01f, FittingMethod.Heuristic);
            return segs.Count;
        }
    }
}
