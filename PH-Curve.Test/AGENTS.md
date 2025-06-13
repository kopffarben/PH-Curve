# Test Suite Guidelines

This document defines the expectations for all tests inside the `PH-Curve.Test` project.

## Running Tests

Execute the test suite from the repository root or this directory with:

```bash
dotnet test
```

This command builds the solution and runs every MSTest class. Ensure tests pass before committing changes.

## Documentation Requirements

- **Language:** All test source files and comments must be written in English.
- **Mathematical context:** Each test should include XML documentation summarising the underlying mathematics. Cite relevant formulas or algorithms and include academic references when appropriate.
- **Console output:** Tests are expected to write informative messages using `Console.WriteLine` so that the output explains which mathematical properties are being validated.
- **Purpose:** The goal of every test is to confirm the mathematical correctness of PH-curve computation and related algorithms.

Follow these guidelines to maintain a clear and well-documented test suite.
