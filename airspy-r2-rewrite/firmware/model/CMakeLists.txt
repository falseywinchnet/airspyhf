if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    cmake_minimum_required(VERSION 3.20)
    project(airspy_firmware_model_standalone LANGUAGES C)
    enable_testing()
    add_subdirectory(../../contract contract)
endif()

add_library(airspy_firmware_model STATIC
    src/buffer_model.c
    src/steering_model.c
    src/stream_status_model.c
    src/transport_pump_model.c
    src/usb_queue_model.c)
target_include_directories(airspy_firmware_model PUBLIC include)
target_link_libraries(airspy_firmware_model PUBLIC airspy_one_contract)
target_compile_features(airspy_firmware_model PUBLIC c_std_11)
set_target_properties(airspy_firmware_model PROPERTIES
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO)

add_executable(airspy_firmware_model_test tests/buffer_model_test.c)
target_link_libraries(airspy_firmware_model_test PRIVATE airspy_firmware_model)
add_test(NAME firmware.buffer_model COMMAND airspy_firmware_model_test)

add_executable(airspy_usb_queue_model_test tests/usb_queue_model_test.c)
target_link_libraries(airspy_usb_queue_model_test PRIVATE airspy_firmware_model)
add_test(NAME firmware.usb_queue_model COMMAND airspy_usb_queue_model_test)

add_executable(airspy_steering_model_test tests/steering_model_test.c)
target_link_libraries(airspy_steering_model_test PRIVATE airspy_firmware_model)
add_test(NAME firmware.steering_model COMMAND airspy_steering_model_test)
