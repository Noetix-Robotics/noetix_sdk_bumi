#!/bin/bash
if [ "$1" = "clean" ]; then
    rm -rf build example_low example_high
    exit 0
fi
get_arch=$(uname -m)
case $get_arch in
    "x86_64")
        echo "this is x86_64"
        arch=x86_64;;
    "aarch64")
        echo "this is arm64"
        arch=arm64;;
    *)
    echo "unknown!!"
    exit 1
esac

mkdir -p build
cd build

if [ $arch = "arm64" ];then
    cmake ..
else
    cmake ..
fi

make -j4
