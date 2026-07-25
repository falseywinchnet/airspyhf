cmake_minimum_required(VERSION 3.20)
project(airspy_r2_rewrite LANGUAGES C CXX)

option(AIRSPY_REWRITE_WARNINGS_AS_ERRORS
    "Treat warnings as errors in the executable rewrite models" ON)

if(MSVC)
    add_compile_options(/W4)
    if(AIRSPY_REWRITE_WARNINGS_AS_ERRORS)
        add_compile_options(/WX)
    endif()
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(AIRSPY_REWRITE_WARNINGS_AS_ERRORS)
        add_compile_options(-Werror)
    endif()
endif()

enable_testing()

add_subdirectory(contract)
add_subdirectory(firmware/model)
add_subdirectory(driver/model)
