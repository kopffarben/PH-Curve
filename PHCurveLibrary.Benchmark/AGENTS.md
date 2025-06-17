# Benchmark Guidelines

This project contains BenchmarkDotNet benchmarks used to measure the performance of curve-fitting algorithms. It is part of the same solution as `PHCurveLibrary` and `PHCurveLibrary.Test`.

## Running Benchmarks

Build and execute the benchmarks in Release mode:

```bash
dotnet run -c Release --project PHCurveLibrary.Benchmark
```

BenchmarkDotNet will write a summary under `BenchmarkDotNet.Artifacts` and print the results to the console.

## Documentation Requirements

- All benchmark source files and comments must be written in English.
- Use the same code style as the main library (PascalCase for types, camelCase for variables).
- Document new benchmarks with XML comments explaining which algorithm is measured.

Follow these guidelines to maintain clear and reproducible performance tests.
