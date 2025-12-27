#!/bin/bash

echo "=========================================="
echo "Building pVisionDetector"
echo "=========================================="

# Go to build directory
cd ~/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/

# Clean previous build
echo "Cleaning previous build..."
rm -rf CMakeFiles/ CMakeCache.txt

# Run CMake
echo "Running CMake..."
cmake ../

# Build
echo "Building..."
make -j$(nproc)

# Install
echo "Installing..."
sudo make install

echo ""
echo "✓ Build complete!"
echo ""
echo "To run:"
echo "  cd ~/Documents/automation/moos-ivp-morgan-stern-drive-mercury/missions/"
echo "  pVisionDetector pVisionDetector.moos"
echo ""
