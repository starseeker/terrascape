# Terrascape Demo

A simplified Terra-based height field mesh generation demo using CMake.

## Description

This project provides a CMake build system for the Terra height field simplification software and includes a demo program (`terrascape_demo`) that processes terrain data and generates 3D mesh output.

## Features

- **CMake Build System**: Modern CMake-based build configuration
- **PGM Input**: Reads PGM (Portable Gray Map) height field data
- **OBJ Output**: Generates OBJ format 3D mesh files
- **Cross-platform**: Uses modern C++ standards for portability

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

```bash
./build/bin/terrascape_demo
```

The demo will:
1. Read `crater.pgm` height field data
2. Generate a simplified mesh representation
3. Output `crater_mesh.obj` file with 3D triangulated mesh

## Output

The generated OBJ file contains:
- Vertex coordinates with height data from the DEM
- Triangulated faces for 3D mesh representation
- Sample spacing of every 10th pixel for manageable file size

## Original Terra Software

This is based on the Terra height field simplification software by Michael Garland and Paul Heckbert. The original Terra implementation uses advanced greedy algorithms for optimal terrain approximation.

## Current Status

- ✅ CMake build system working
- ✅ PGM file reading functional
- ✅ Basic mesh generation and OBJ output
- ⚠️  Advanced greedy mesh algorithms temporarily disabled due to complex data structure issues

## Files Added

- `CMakeLists.txt` - Build configuration
- `main.cpp` - Demo application entry point
- `Vec2.h`, `Vec3.h` - Vector math classes
- `Array.h` - Template array classes
- `.gitignore` - Git ignore patterns

## Dependencies

- CMake 3.12+
- C++11 compatible compiler
- Standard C++ library