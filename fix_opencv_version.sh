#!/bin/bash

cd /usr/lib/aarch64-linux-gnu/

# Run the automated fix for all OpenCV libraries
for lib in libopencv_*.so; do
    if [ -L "$lib" ] && readlink "$lib" | grep -q "4\.5"; then
        base=$(basename "$lib" .so)
        echo "Fixing $lib -> ${base}.so.4.2"
        sudo rm "$lib"
        sudo ln -sf "${base}.so.4.2" "$lib"
    fi
done
