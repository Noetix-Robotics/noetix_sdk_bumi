#
# Copyright(c) 2020 to 2022 ZettaScale Technology and others
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License v. 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Eclipse Distribution License
# v. 1.0 which is available at
# http://www.eclipse.org/org/documents/edl-v10.php.
#
# SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause
#
cmake_minimum_required(VERSION 3.16)
project(ddstest LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto=auto")
# 找到 Python3 的库文件路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
                ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)
        link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)
        set(DYNAMIC_LIB_DIR ${CMAKE_SOURCE_DIR}/lib/aarch64)
        set(DDS_LIB_DIR ${CMAKE_SOURCE_DIR}/dds/aarch64/lib)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
                ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
        set(DYNAMIC_LIB_DIR ${CMAKE_SOURCE_DIR}/lib/x86_64)
        link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
        set(DDS_LIB_DIR ${CMAKE_SOURCE_DIR}/dds/x86_64/lib)
endif()

find_package(onnxruntime REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(highcontrol_py src/highcontrol_pybind.cpp)

pybind11_add_module(lowcontrol_py src/lowcontrol_pybind.cpp)

pybind11_add_module(mediacontrol_py src/mediacontrol_pybind.cpp)

target_link_libraries(highcontrol_py
    PRIVATE
    driver
    ${DDS_LIB_DIR}/libddsc.so
    ${DDS_LIB_DIR}/libddscxx.so
)

target_link_libraries(lowcontrol_py
    PRIVATE
    driver
    ${DDS_LIB_DIR}/libddsc.so
    ${DDS_LIB_DIR}/libddscxx.so
)

target_link_libraries(mediacontrol_py
    PRIVATE
    driver
    ${DDS_LIB_DIR}/libddsc.so
    ${DDS_LIB_DIR}/libddscxx.so
)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/thirdlib/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
        include_directories(${CMAKE_SOURCE_DIR}/dds/aarch64/include/ddscxx)
        include_directories(${CMAKE_SOURCE_DIR}/dds/aarch64/include/ddsc)
        include_directories(${CMAKE_SOURCE_DIR}/dds/aarch64/include)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
        include_directories(${CMAKE_SOURCE_DIR}/dds/x86_64/include/ddscxx)
        include_directories(${CMAKE_SOURCE_DIR}/dds/x86_64/include/ddsc)
        include_directories(${CMAKE_SOURCE_DIR}/dds/x86_64/include)
endif()
add_compile_definitions(SDK_ROOT="${CMAKE_BINARY_DIR}/..")
include_directories(${EIGEN3_INCLUDE_DIRS})


add_executable(example_high examples_cpp/example_high.cpp)
add_executable(example_low examples_cpp/example_low.cpp)
add_executable(example_policy_walk examples_cpp/example_policy_walk.cpp)
add_executable(example_media examples_cpp/example_media.cpp)

target_include_directories(example_high
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(example_low
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(example_policy_walk
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(example_media
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(example_high
        Threads::Threads 
        driver
        ${DDS_LIB_DIR}/libddsc.so
        ${DDS_LIB_DIR}/libddscxx.so
)
target_link_libraries(example_low
        Threads::Threads
        driver
        ${DDS_LIB_DIR}/libddsc.so
        ${DDS_LIB_DIR}/libddscxx.so
)
target_link_libraries(example_policy_walk
        Threads::Threads
        onnxruntime
        yaml-cpp
        driver
        ${DDS_LIB_DIR}/libddsc.so
        ${DDS_LIB_DIR}/libddscxx.so
)
target_link_libraries(example_media
        Threads::Threads
        driver
        ${DDS_LIB_DIR}/libddsc.so
        ${DDS_LIB_DIR}/libddscxx.so
)
