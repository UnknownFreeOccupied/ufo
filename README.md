# UFO (Unknown Free Occupied)

An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown.

[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](https://raw.githubusercontent.com/UnknownFreeOccupied/ufo/0b998a3bbadd0a59eb25c0be5b9af0e56a5755a2/LICENSE)
[![Standard](https://img.shields.io/badge/C%2B%2B-23-blue.svg)](https://en.cppreference.com/w/cpp/23)

UFO is a high-performance, modular 3D mapping framework designed for modern robotics and computer vision. It provides a robust probabilistic representation of space that explicitly handles free, occupied, and unknown regions, making it ideal for path planning and exploration in complex environments.

## Key Features

- **Modern C++23**: Leverages the latest language features for maximum performance and readability.
- **Probabilistic Mapping**: Accurately represents uncertainty in 3D environments.
- **High Performance**: Optimized with SIMD (SSE2, AVX, etc.) and Link Time Optimization (LTO).
- **Modular Architecture**: Easy to extend or use individual components like `Core`, `Math`, and `Utility`.
- **Cross-Platform**: Supports Linux, macOS, and Windows.

## Quick Start

### Prerequisites

- **CMake** (v3.23 or newer)
- **C++23 Compatible Compiler** (GCC 14+, Clang 18+, MSVC 19.40+)
- Dependencies (automatically fetched if not found):
  - [Catch2](https://github.com/catchorg/Catch2) (Testing)
  - [CLI11](https://github.com/CLIUtils/CLI11) (CLI Parser)
  - [Doxygen](https://www.doxygen.nl/) (Documentation)

### Build Instructions

```bash
# Clone the repository
git clone https://github.com/UnknownFreeOccupied/ufo.git
cd ufo

# Configure and build
cmake -B build -DUFO_DEV_MODE=ON
cmake --build build -j$(nproc)
```

### Running the App

The main `ufo` utility provides system diagnostics and diagnostic information about the build environment:

```bash
./build/apps/ufo --version
```

## Project Structure

- [`lib/core`](lib/core): Fundamental types and core mapping logic (Confidence, Semantic, etc.).
- [`lib/math`](lib/math): Optimized linear algebra and geometric primitives.
- [`lib/utility`](lib/utility): Threading, string manipulation, and system helpers.
- [`apps/`](apps): Command-line tools and applications.
- [`tests/`](tests): Global test suite (Catch2).

## Documentation

API documentation can be generated using Doxygen:

```bash
cmake -B build -DUFO_BUILD_DOCS=ON
cmake --build build --target Docs
```

The output will be available in `build/docs/html/index.html`.

## License

UFO is released under the [BSD 3-Clause License](https://raw.githubusercontent.com/UnknownFreeOccupied/ufo/0b998a3bbadd0a59eb25c0be5b9af0e56a5755a2/LICENSE).
