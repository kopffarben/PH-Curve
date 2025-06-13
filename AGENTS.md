# Agent Instructions

This repository contains C# code targeting **.NET 8.0**. The following guidelines apply when working with this project.
Always name the codex tasks in English.

## Using .NET

.NET is a platform for developing modern applications supporting multiple languages, including C#, F#, and Visual Basic. With the .NET SDK, you can create, run, and manage applications efficiently.

### Checking Installation:

```bash
dotnet --version
```

This command shows the installed version of the .NET SDK.

### Creating a New Project:

```bash
dotnet new console -o MyApp
cd MyApp
dotnet run
```

This creates and runs a simple console application.

## Development Guidelines

- All branches and commits should be written in English.
- Questions will be asked in German and responses should also be in German, unless explicitly stated otherwise.
- All program code will generally be written in C# targeting .NET 8.0.
- All public members should include XML documentation comments.
- Tests should always be run after every code change.

### Recommended Code Style

- Indentation: 4 spaces, no tabs.
- Naming:
  - Classes and methods: PascalCase
  - Local variables and parameters: camelCase
  - Constants: UPPERCASE\_WITH\_UNDERSCORES
- Brackets:
  - Always place curly brackets on new lines for methods and classes.
  - Single-line expressions can use inline brackets.
- Use explicit types wherever possible (avoid `var` except when clearly inferred).
- Maximum line length: 120 characters.

Example of XML documentation:

```csharp
/// <summary>
/// Adds two numbers together.
/// </summary>
/// <param name="x">The first number.</param>
/// <param name="y">The second number.</param>
/// <returns>The sum of the two numbers.</returns>
public int Add(int x, int y) => x + y;
```

## Solving Algebraic Problems with F\#

F# is particularly well-suited for mathematical and algebraic problems due to its interactive environment.

### Using the F# Console

Start the F# console with:

```bash
dotnet fsi
```

Within the console, you can directly enter and evaluate algebraic operations:

```fsharp
let square x = x * x
square 5
```

The result (`25`) is shown immediately.

### More Complex Algebra

For complex problems, you can directly define functions:

```fsharp
let solveQuadratic a b c =
    let discriminant = b*b - 4.0*a*c
    let sqrtDiscriminant = sqrt discriminant
    ((-b + sqrtDiscriminant) / (2.0*a), (-b - sqrtDiscriminant) / (2.0*a))

// Example usage:
solveQuadratic 1.0 -3.0 2.0
```

This provides the solutions to the quadratic equation.

### Exiting the Console

To exit the F# console, type:

```fsharp
#quit;;
```

## Using dotnet-script

`dotnet-script` allows you to write and execute C# scripts (.csx files) directly from the command line.

### Installing dotnet-script (if not already installed):

```bash
dotnet tool install -g dotnet-script
```

Make sure your path includes the .NET tools directory:

```bash
export PATH="$PATH:$HOME/.dotnet/tools"
```

### Creating and Running a Script:

Create a new file, e.g. `hello.csx`, with the following content:

```csharp
Console.WriteLine("Hello from dotnet-script!");
```

Run it using:

```bash
dotnet script hello.csx
```

You can also import packages and use top-level code, making it perfect for quick testing or automation scripts.

## Running Tests

Use the following command to run tests:

```bash
dotnet test
```

This automatically executes all available unit tests in your project.

You are now well-equipped to efficiently tackle algebraic problems using .NET, particularly F#, and leverage C# scripting with dotnet-script.

