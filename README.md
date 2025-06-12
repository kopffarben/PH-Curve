# PH-Curve

PH-Curve is a small C# library that implements cubic Pythagorean hodograph (PH) curves in three dimensions. The project targets **.NET 8.0** and provides a minimal API for creating and evaluating PH curves as well as fitting a curve to a sequence of control points.

## Features

- Construction of `CubicPHCurve3D` segments from Hermite control points
- Evaluation of derivatives, tangents and normals
- Support for offset curves and Frenet frame vectors
- Curve fitting utilities via `CubicPHCurve3DFitter`

## Building

The solution consists of two projects:

1. **PH-Curve** – class library containing the curve implementation
2. **PH-Curve.Test** – MSTest project with unit tests

To restore dependencies and build the solution run:

```bash
dotnet build
```

## Running the Tests

Execute all unit tests using the following command from the repository root:

```bash
dotnet test
```

The tests verify the curve fitting functionality and a variety of geometric computations in `CubicPHCurve3D`.

## Usage Example

```csharp
using System.Numerics;
using CubicPHCurve;

var cp0 = new CubicPHCurve3D.ControlPoint(new Vector3(0, 0, 0), new Vector3(1, 0, 0));
var cp1 = new CubicPHCurve3D.ControlPoint(new Vector3(1, 1, 0), new Vector3(1, 1, 0));
CubicPHCurve3D curve = CubicPHCurve3D.FromControlPoints(cp0, cp1);

Vector3 position = curve.Position(0.5f);
Vector3 tangent = curve.Tangent(0.5f);
```

The above snippet creates a PH curve segment from two Hermite points and queries its midpoint position and tangent.

