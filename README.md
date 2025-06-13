# PH-Curve

PH-Curve is a C# library implementing **quintic Pythagorean hodograph (PH) curves** in three dimensions. The project targets **.NET&nbsp;8.0** and exposes a minimal API for constructing and analysing PH segments as well as building multi segment paths.

## Features

- Construction of `PHCurve3D` segments from Hermite control points via `PHCurveFactory`
- Evaluation of derivatives and Frenet frame vectors
- Exact or numerical arc-length evaluation
- Path assembly utilities in `PathPlanner`
- Comprehensive MSTest suite demonstrating the mathematical algorithms

## Building

The solution consists of two projects:

1. `PHCurveLibrary` – the class library containing the curve implementation
2. `PHCurveLibrary.Test` – MSTest project with unit tests

Restore dependencies and build the solution with

```bash
dotnet build
```

## Running the tests

Execute all unit tests from the repository root:

```bash
dotnet test
```

The tests cover curve creation, Frenet-frame computations and arc-length evaluation.

## Usage example

```csharp
using System.Numerics;
using PHCurveLibrary;

var p0 = new HermiteControlPoint3D(new Vector3(0,0,0), Vector3.UnitX, 0f, Vector3.UnitY);
var p1 = new HermiteControlPoint3D(new Vector3(1,1,0), Vector3.UnitY, 0f, -Vector3.UnitX);
PHCurve3D curve = PHCurveFactory.CreateQuintic(p0, p1);

Vector3 position = curve.Position(0.5f);
Vector3 tangent = curve.TangentUnit(0.5f);
```

The snippet constructs a PH segment from two Hermite points and queries the position and tangent at the parameter midpoint.
