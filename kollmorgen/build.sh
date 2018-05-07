#!/bin/bash

export SRC_DIR=$(pwd)
export BUILD_DIR=$(pwd)/build

# Recreate the build directory
rm -rf $BUILD_DIR
mkdir $BUILD_DIR
cd $BUILD_DIR

# Run CMake to configure & generate Visual Studio solution
cmake $SRC_DIR

# Print the version of the make utility and build the binaries
make