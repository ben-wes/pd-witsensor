#!/bin/bash
# Build script for witsensor external with SimpleBLE
# Set Pure Data path and build

export PDINCLUDEDIR="/Applications/Pd-0.56-1.app/Contents/Resources/src"

echo "Building with Pure Data from: $PDINCLUDEDIR"
echo "SimpleBLE will be built automatically as part of the build process"

# Clean and build (SimpleBLE will be built automatically)
make clean
make

echo "Build complete!"
echo "External created: witsensor.pd_darwin"
