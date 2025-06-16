using BenchmarkDotNet.Running;
using PHCurveLibrary.Tests;

namespace PHCurveLibrary.Benchmark
{
    internal class Program
    {
        static void Main(string[] args)
        {
            BenchmarkRunner.Run<FittingBenchmarks>();
        }
    }
}
