# M5Paper DOS - ESP-IDF v5.5 Project
# DOS emulator for M5Paper e-ink display

cmake_minimum_required(VERSION 3.16)

# Set the IDF path
set(IDF_PATH "c:/Espressif/frameworks/esp-idf-v5.5")

# Include the ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Disable warnings as errors globally (for GCC 14 compatibility with -O2)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error=maybe-uninitialized -Wno-error")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=maybe-uninitialized -Wno-error")

idf_build_set_property(COMPILE_OPTIONS "-Wno-error" "-Wno-error=maybe-uninitialized" "-Wno-error=unused-but-set-variable" APPEND)

# Project name
project(m5paper_dos)

# M5Unified (managed component) triggers a GCC false-positive `-Wmaybe-uninitialized`
# in `I2C_Class.hpp` when ESP-IDF enables `-Werror=all`. Keep warnings enabled, but
# avoid failing the build for this specific component.
if(TARGET __idf_m5stack__m5unified)
    target_compile_options(__idf_m5stack__m5unified PRIVATE -Wno-error=maybe-uninitialized)
endif()
