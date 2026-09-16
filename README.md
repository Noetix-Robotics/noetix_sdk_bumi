# noetix_sdk_bumi
noetix robot Bumi sdk.

### Prebuild environment
* OS  (Ubuntu 22.04 LTS)
* CPU  (aarch64 and x86_64)
* Compiler  (gcc version 11.4.0)

### Environment Setup

Before building or running the SDK, ensure the following dependencies are installed:

- CMake (version 3.3 or higher)
- GCC (version 11.4.0)
- Make

You can install the required packages on Ubuntu 22.04 with:

```bash
apt-get update
apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libfmt-dev pybind11-dev
```

### Build examples

To build the examples inside this repository:

```bash
./build.sh
```

### Notice
For more reference information, please go to [Bumi Document Center](https://web.noetixrobotics.com/docs/).
