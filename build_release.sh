#!/bin/bash
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
esac
mkdir build
cd build
if [ $arch = "arm64" ];then
    cmake -DRK3588=ON ..
else
    cmake ..
fi
make -j4
cd ..
rm -rf build
chmod +x highcontrol
#chmod +x lowcontrol

