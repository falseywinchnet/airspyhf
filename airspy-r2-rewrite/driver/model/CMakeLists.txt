if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.20)
    project(airspy_driver_model_standalone LANGUAGES C CXX)
    enable_testing()
    add_subdirectory(../../contract contract)
endif()

add_library(airspy_driver_model STATIC
    src/legacy_unpack.cpp
    src/stream_accounting.cpp
    src/stream_lifecycle.cpp)
target_include_directories(airspy_driver_model PUBLIC include)
target_link_libraries(airspy_driver_model PUBLIC airspy_one_contract)
target_compile_features(airspy_driver_model PUBLIC cxx_std_20)
set_target_properties(airspy_driver_model PROPERTIES
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO)

add_executable(airspy_driver_model_test tests/driver_model_test.cpp)
target_link_libraries(airspy_driver_model_test PRIVATE airspy_driver_model)
add_test(NAME driver.model COMMAND airspy_driver_model_test)
