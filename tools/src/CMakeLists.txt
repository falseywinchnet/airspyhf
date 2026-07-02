# Copyright 2012 Jared Boone
# Copyright 2013/2014 Benjamin Vernoux
# Copyright 2018 Andrea Montefusco IW0HDV <andrew@montefusco.com>

# This file is part of AirSpy (based on HackRF project).
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

# Based heavily upon the libftdi cmake setup.

set(INSTALL_DEFAULT_BINDIR "${CMAKE_INSTALL_BINDIR}" CACHE STRING "Appended to CMAKE_INSTALL_PREFIX")

if(MSVC)
add_library(libgetopt_static STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../getopt/getopt.c
)
endif()

add_executable(airspyhf_lib_version airspyhf_lib_version.c)
install(TARGETS airspyhf_lib_version RUNTIME DESTINATION ${INSTALL_DEFAULT_BINDIR})

add_executable(airspyhf_info airspyhf_info.c)
install(TARGETS airspyhf_info RUNTIME DESTINATION ${INSTALL_DEFAULT_BINDIR})

add_executable(airspyhf_rx airspyhf_rx.c)
install(TARGETS airspyhf_rx RUNTIME DESTINATION ${INSTALL_DEFAULT_BINDIR})

add_executable(airspyhf_gpio airspyhf_gpio.c)
install(TARGETS airspyhf_gpio RUNTIME DESTINATION ${INSTALL_DEFAULT_BINDIR})

add_executable(airspyhf_calibrate airspyhf_calibrate.c)
install(TARGETS airspyhf_calibrate RUNTIME DESTINATION ${INSTALL_DEFAULT_BINDIR})

add_executable(airspyhf_stream_break_test airspyhf_stream_break_test.c)
install(TARGETS airspyhf_stream_break_test RUNTIME DESTINATION ${INSTALL_DEFAULT_BINDIR})

if(UNIX AND NOT WIN32)
add_executable(airspyhf-helper
    ${CMAKE_SOURCE_DIR}/libairspyhf/wine-bridge/helper.c
)
target_include_directories(airspyhf-helper PRIVATE
    ${CMAKE_SOURCE_DIR}/libairspyhf/wine-bridge
)
install(TARGETS airspyhf-helper RUNTIME DESTINATION ${INSTALL_DEFAULT_BINDIR})

set(AIRSPYHF_WINE_SHIM_DIR "${CMAKE_CURRENT_BINARY_DIR}/wine")
set(AIRSPYHF_WINE_SHIM "${AIRSPYHF_WINE_SHIM_DIR}/airspyhf.dll")
set(AIRSPYHF_MINGW_CC_DEFAULT x86_64-w64-mingw32-gcc)
find_program(AIRSPYHF_MINGW_CC
    NAMES ${AIRSPYHF_MINGW_CC_DEFAULT}
)
if(AIRSPYHF_MINGW_CC)
    add_custom_command(
        OUTPUT ${AIRSPYHF_WINE_SHIM}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${AIRSPYHF_WINE_SHIM_DIR}
        COMMAND ${AIRSPYHF_MINGW_CC}
            -O2 -Wall -Wextra -shared
            -I${CMAKE_SOURCE_DIR}/libairspyhf/wine-bridge
            -I${CMAKE_SOURCE_DIR}/libairspyhf/src
            -o ${AIRSPYHF_WINE_SHIM}
            ${CMAKE_SOURCE_DIR}/libairspyhf/wine-bridge/shim.c
            -lws2_32
        DEPENDS
            ${CMAKE_SOURCE_DIR}/libairspyhf/wine-bridge/shim.c
            ${CMAKE_SOURCE_DIR}/libairspyhf/wine-bridge/proto.h
            ${CMAKE_SOURCE_DIR}/libairspyhf/src/airspyhf.h
        VERBATIM
    )
    add_custom_target(airspyhf-wine-shim ALL DEPENDS ${AIRSPYHF_WINE_SHIM})
    install(FILES ${AIRSPYHF_WINE_SHIM}
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/airspyhf/wine
    )
else()
    message(STATUS "x86_64-w64-mingw32-gcc not found; skipping Wine airspyhf.dll shim")
endif()
endif()

if(NOT libairspyhf_SOURCE_DIR)
include_directories(${LIBAIRSPYHF_INCLUDE_DIR})
LIST(APPEND TOOLS_LINK_LIBS ${LIBAIRSPYHF_LIBRARIES})
else()
LIST(APPEND TOOLS_LINK_LIBS airspyhf)
endif()

if(MSVC)
LIST(APPEND TOOLS_LINK_LIBS libgetopt_static)
endif()

LIST(APPEND TOOLS_LINK_LIBS -lm)

target_link_libraries(airspyhf_lib_version ${TOOLS_LINK_LIBS})
target_link_libraries(airspyhf_info ${TOOLS_LINK_LIBS})
target_link_libraries(airspyhf_rx ${TOOLS_LINK_LIBS})
target_link_libraries(airspyhf_gpio ${TOOLS_LINK_LIBS})
target_link_libraries(airspyhf_calibrate ${TOOLS_LINK_LIBS})
target_link_libraries(airspyhf_stream_break_test ${TOOLS_LINK_LIBS})
if(TARGET airspyhf-helper)
target_link_libraries(airspyhf-helper ${TOOLS_LINK_LIBS} ${CMAKE_THREAD_LIBS_INIT})
endif()
