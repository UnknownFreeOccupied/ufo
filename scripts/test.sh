#!/usr/bin/env sh

echo "Setting up test environment..."
rm -rf pixi_build

cmake -B pixi_build -S .  \
  -DCMAKE_GENERATOR=Ninja \
	-DUFO_BUILD_TESTS=ON    \
	-DUSE_SYSTEM_CATCH2=ON  \
	-DCMAKE_BUILD_TYPE=Release

echo "Building environment..."
cmake --build pixi_build --config Release

echo "Running tests..."
ctest --test-dir pixi_build --output-on-failure --build-config Release

echo "Cleaning up..."
rm -rf pixi_build