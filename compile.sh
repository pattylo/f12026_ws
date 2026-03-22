#!/bin/bash

set -e

if [ -n "$1" ]; then
    echo "Building package: $1 with symlink-install"
    colcon build --symlink-install --packages-select "$1"
else
    echo "Building all packages with symlink-install"
    colcon build --symlink-install
fi