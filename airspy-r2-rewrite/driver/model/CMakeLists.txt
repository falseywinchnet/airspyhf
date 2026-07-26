if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.20)
    project(airspy_driver_model_standalone LANGUAGES C CXX)
    enable_testing()
    add_subdirectory(../../contract contract)
endif()

add_library(airspy_driver_model STATIC
    src/buffer_pool.cpp
    src/completion_path.cpp
    src/legacy_unpack.cpp
    src/legacy_unpack_x86.cpp
    src/prepared_start.cpp
    src/sample_conversion.cpp
    src/stream_accounting.cpp
    src/stream_lifecycle.cpp
    src/transfer_lifecycle.cpp)
target_include_directories(airspy_driver_model PUBLIC include)
target_link_libraries(airspy_driver_model PUBLIC airspy_one_contract)
target_compile_features(airspy_driver_model PUBLIC cxx_std_20)
set_target_properties(airspy_driver_model PROPERTIES
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO)

add_executable(airspy_driver_model_test tests/driver_model_test.cpp)
target_link_libraries(airspy_driver_model_test PRIVATE airspy_driver_model)
add_test(NAME driver.model COMMAND airspy_driver_model_test)

add_executable(airspy_legacy_unpack_guard_test
    tests/legacy_unpack_guard_test.cpp)
target_link_libraries(airspy_legacy_unpack_guard_test
    PRIVATE airspy_driver_model)
add_test(NAME driver.legacy_unpack_guard
    COMMAND airspy_legacy_unpack_guard_test)

add_executable(airspy_stream_transport_test tests/stream_transport_test.cpp)
target_link_libraries(airspy_stream_transport_test PRIVATE airspy_driver_model)
add_test(NAME driver.stream_transport COMMAND airspy_stream_transport_test)

add_executable(airspy_abi_contract_test tests/abi_contract_test.cpp)
target_include_directories(airspy_abi_contract_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../current/libairspy/src)
target_compile_features(airspy_abi_contract_test PRIVATE cxx_std_20)
add_test(NAME driver.abi_contract COMMAND airspy_abi_contract_test)

add_executable(airspy_control_contract_test tests/control_contract_test.cpp)
target_link_libraries(airspy_control_contract_test PRIVATE airspy_driver_model)
add_test(NAME driver.control_contract COMMAND airspy_control_contract_test)

if(MSVC)
    target_compile_options(airspy_driver_model PRIVATE /W4)
    target_compile_options(airspy_driver_model_test PRIVATE /W4)
    target_compile_options(airspy_legacy_unpack_guard_test PRIVATE /W4)
    target_compile_options(airspy_stream_transport_test PRIVATE /W4)
    target_compile_options(airspy_abi_contract_test PRIVATE /W4)
    target_compile_options(airspy_control_contract_test PRIVATE /W4)
else()
    target_compile_options(airspy_driver_model PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    target_compile_options(airspy_driver_model_test PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    target_compile_options(airspy_legacy_unpack_guard_test PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    target_compile_options(airspy_stream_transport_test PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    target_compile_options(airspy_abi_contract_test PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    target_compile_options(airspy_control_contract_test PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()
