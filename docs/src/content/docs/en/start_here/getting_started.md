---
title: Getting Started
description: Get familiar with UFO.
---

UFO (Unknown Free Occupied) is a high-performance 3D mapping framework that explicitly represents free, occupied, and unknown space. This guide will help you get started with the core concepts and basic usage.

## Basic Concepts

- **Spatial Containers**: UFO provides various tree-based structures like `OctreeMap`, `QuadtreeMap`, and `HextreeMap` to represent space at different resolutions and dimensions.
- **Probabilistic Representation**: Each cell in a map can store occupancy information, confidence, semantics, and more.
- **Efficient Queries**: The framework is optimized for fast spatial queries, such as nearest neighbor searches and intersections.

## Basic Usage

Here is a simple example of how to use an `OctreeMap` to store and query 3D points.

### Code Example

```cpp
#include <ufo/container/octree_map.hpp>
#include <iostream>

int main() {
    // 1. Initialize an OctreeMap with 'int' as the data type
    ufo::OctreeMap<int> map;

    // 2. Insert data at specific 3D coordinates (x, y, z)
    map.insert({{0.0, 0.0, 0.0}, 42});
    map.insert({{1.0, 2.0, 3.0}, 100});

    // 3. Perform a nearest neighbor search
    ufo::Vec3f query_point(0.1, 0.1, 0.1);
    auto nearest = map.nearest(query_point);

    if (!nearest.empty()) {
        auto [pos, value] = *nearest.begin();
        std::cout << "Nearest point at " << pos << " has value: " << value << std::endl;
    }

    // 4. Iterate over all elements in the map
    std::cout << "All elements in map:" << std::endl;
    for (auto [pos, value] : map) {
        std::cout << pos << ": " << value << std::endl;
    }

    return 0;
}
```

## Building a Simple App

To use UFO in your own project, you can find it via CMake.

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.23)
project(MyUfoApp)

# Set C++23 standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find UFO package
find_package(UFO REQUIRED)

# Add your executable
add_executable(my_app main.cpp)

# Link against UFO
target_link_libraries(my_app PRIVATE UFO::UFO)
```

### Build Instructions

```bash
cmake -B build
cmake --build build
./build/my_app
```