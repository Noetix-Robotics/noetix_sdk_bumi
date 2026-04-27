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
project(ddsdemo LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto=auto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
# 找到 Python3 的库文件路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
        # add_definitions(-DRK3588)
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)
        set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib/aarch64)
        set(CMAKE_INSTALL_RPATH "$ORIGIN/lib/aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
        set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib/x86_64)
        set(CMAKE_INSTALL_RPATH "$ORIGIN/lib/x86_64")
endif()

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(pybind11 REQUIRED)
pybind11_add_module(highcontrol_py src/highcontrol_pybind.cpp src/highcontroller.cpp)
pybind11_add_module(lowcontrol_py src/lowcontrol_pybind.cpp src/lowcontroller.cpp)
#find_package(onnxruntime REQUIRED)
set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/idl)

add_compile_definitions(SDK_ROOT="${CMAKE_BINARY_DIR}/..")
include_directories(${EIGEN3_INCLUDE_DIRS})

target_link_libraries(highcontrol_py
    PRIVATE
    driver
    ddsc
    ddscxx
    yaml-cpp
)

target_link_libraries(lowcontrol_py
    PRIVATE
    driver
    ddsc
    ddscxx
    yaml-cpp
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

add_executable(example_low example_low.cpp src/lowcontroller.cpp)

add_executable(example_high example_high.cpp src/highcontroller.cpp)
# Link both executables to idl data type library and ddscxx.
target_include_directories(example_high  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(example_low  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
        target_link_directories(example_high PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)
        target_link_directories(example_low PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)
        target_link_directories(lowcontrol_py PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)
        target_link_directories(highcontrol_py PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)
        target_link_libraries(example_low
                Threads::Threads
                onnxruntime
                ${LIB_DIR}/aarch64/libdriver.a
                ${LIB_DIR}/aarch64/libyaml-cpp.a
                ${LIB_DIR}/aarch64/libddsc.so
                ${LIB_DIR}/aarch64/libddscxx.so
        )
        target_link_libraries(example_high
                Threads::Threads
                yaml-cpp
                ${LIB_DIR}/aarch64/libdriver.a
                ${LIB_DIR}/aarch64/libyaml-cpp.a
                ddsc
                ddscxx
        )
        target_link_libraries(highcontrol_py
                PRIVATE
                ${LIB_DIR}/aarch64/libdriver.a
                ${LIB_DIR}/aarch64/libyaml-cpp.a
                ${LIB_DIR}/aarch64/libddsc.so
                ${LIB_DIR}/aarch64/libddscxx.so
        )

        target_link_libraries(lowcontrol_py
                PRIVATE
                ${LIB_DIR}/aarch64/libdriver.a
                ${LIB_DIR}/aarch64/libyaml-cpp.a
                ${LIB_DIR}/aarch64/libddsc.so
                ${LIB_DIR}/aarch64/libddscxx.so
        )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
        target_link_directories(example_high PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
        target_link_directories(example_low PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
        target_link_directories(lowcontrol_py PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
        target_link_directories(highcontrol_py PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
        target_link_libraries(example_low
                Threads::Threads
                onnxruntime
                ${LIB_DIR}/x86_64/libdriver.a
                ${LIB_DIR}/x86_64/libyaml-cpp.a
                ${LIB_DIR}/x86_64/libddsc.so
                ${LIB_DIR}/x86_64/libddscxx.so
        )
        target_link_libraries(example_high
                Threads::Threads
                ${LIB_DIR}/x86_64/libdriver.a
                ${LIB_DIR}/x86_64/libyaml-cpp.a
                ${LIB_DIR}/x86_64/libddsc.so
                ${LIB_DIR}/x86_64/libddscxx.so
        )
        target_link_libraries(highcontrol_py
                PRIVATE
                ${LIB_DIR}/x86_64/libdriver.a
                ${LIB_DIR}/x86_64/libyaml-cpp.a
                ${LIB_DIR}/x86_64/libddsc.so
                ${LIB_DIR}/x86_64/libddscxx.so
        )
        target_link_libraries(lowcontrol_py
                 PRIVATE
                 ${LIB_DIR}/x86_64/libdriver.a
                 ${LIB_DIR}/x86_64/libyaml-cpp.a
                 ${LIB_DIR}/x86_64/libddsc.so
                 ${LIB_DIR}/x86_64/libddscxx.so
        )
endif()
# Disable the static analyzer in GCC to avoid crashing the GNU C++ compiler
# on Azure Pipelines
if(DEFINED ENV{SYSTEM_TEAMFOUNDATIONSERVERURI})
        if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND ANALYZER STREQUAL "on")
                target_compile_options(example_high PRIVATE -fno-analyzer)
                target_compile_options(example_low PRIVATE -fno-analyzer)
        endif()
endif()
