cmake_minimum_required(VERSION 3.20)
project(airspy_readable LANGUAGES C CXX)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)
find_package(Threads REQUIRED)

set(legacy_source_dir
    ${CMAKE_CURRENT_SOURCE_DIR}/../current/libairspy/src)

add_library(airspy-readable SHARED
    src/legacy_translation_unit.cpp
    ../model/src/buffer_pool.cpp
    ../model/src/legacy_unpack.cpp
    ../model/src/legacy_unpack_x86.cpp
    ../model/src/sample_conversion.cpp
    ../model/src/transfer_lifecycle.cpp
    ${legacy_source_dir}/iqconverter_float.c
    ${legacy_source_dir}/iqconverter_int16.c)

# The candidate proves that the parity source has valid C++ representation.
# The production `current/` target still compiles these files as C.
set_source_files_properties(
    ${legacy_source_dir}/iqconverter_float.c
    ${legacy_source_dir}/iqconverter_int16.c
    PROPERTIES LANGUAGE CXX)

target_include_directories(airspy-readable PUBLIC
    ${legacy_source_dir}
    ${CMAKE_CURRENT_SOURCE_DIR}/../model/include)
target_link_libraries(airspy-readable PRIVATE
    PkgConfig::LIBUSB
    Threads::Threads)
target_compile_features(airspy-readable PRIVATE cxx_std_20)
set_target_properties(airspy-readable PROPERTIES
    OUTPUT_NAME airspy_readable
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO)

if(MSVC)
    target_compile_options(airspy-readable PRIVATE /W4)
else()
    target_compile_options(airspy-readable PRIVATE
        -Wall -Wextra -Wpedantic)
endif()

add_executable(airspy-readable-hardware-smoke
    tests/hardware_smoke.cpp)
target_link_libraries(airspy-readable-hardware-smoke PRIVATE
    airspy-readable)
target_compile_features(airspy-readable-hardware-smoke PRIVATE cxx_std_20)
