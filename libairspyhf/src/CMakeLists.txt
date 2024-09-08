#
# Copyright (c) 2012, Jared Boone <jared@sharebrained.com>
# Copyright (c) 2013, Michael Ossmann <mike@ossmann.com>
# Copyright (c) 2013, Youssef Touil <youssef@airspy.com>
# Copyright (c) 2013/2014, Benjamin Vernoux <bvernoux@airspy.com>
#
# This file is part of AirSpy HF+ (based on HackRF project).
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
#     Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#     Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
# 	documentation and/or other materials provided with the distribution.
#     Neither the name of Great Scott Gadgets nor the names of its contributors may be used to endorse or promote products derived from this software
# 	without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Based heavily upon the libftdi cmake setup.
#
# Copyright (c) 2012, Jared Boone <jared@sharebrained.com>
# Copyright (c) 2013, Michael Ossmann <mike@ossmann.com>
# Copyright (c) 2013, Youssef Touil <youssef@airspy.com>
# Copyright (c) 2013/2014, Benjamin Vernoux <bvernoux@airspy.com>
#
# This file is part of AirSpy HF+
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
#     Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#     Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
# 	documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

cmake_minimum_required(VERSION 3.14)

# Set the project name and version
project(AirSpyHFPlus VERSION 1.0 LANGUAGES CXX)

# Enable C++17 or later, and use GNU extensions if necessary
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Sources and headers (updated for .cpp)
set(cpp_sources ${CMAKE_CURRENT_SOURCE_DIR}/airspyhf.cpp ${CMAKE_CURRENT_SOURCE_DIR}/iqbalancer.cpp CACHE INTERNAL "List of C++ sources")
set(cpp_headers ${CMAKE_CURRENT_SOURCE_DIR}/airspyhf.h ${CMAKE_CURRENT_SOURCE_DIR}/iqbalancer.h ${CMAKE_CURRENT_SOURCE_DIR}/airspyhf_commands.h CACHE INTERNAL "List of C++ headers")

# Enable stricter compiling and additional features like restrict
add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -fno-strict-aliasing -fno-omit-frame-pointer)

# Add support for 'restrict' keyword
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-fno-common)
endif()

# Dynamic library
add_library(airspyhf SHARED ${cpp_sources} ${AIRSPYHF_DLL_SRCS})
set_target_properties(airspyhf PROPERTIES VERSION ${AIRSPYHF_VER_MAJOR}.${AIRSPYHF_VER_MINOR}.${AIRSPYHF_VER_REVISION})
set_target_properties(airspyhf PROPERTIES SOVERSION 0)

# Static library
add_library(airspyhf-static STATIC ${cpp_sources})
if(MSVC)
	set_target_properties(airspyhf-static PROPERTIES OUTPUT_NAME "airspyhf_static")
else()
	set_target_properties(airspyhf-static PROPERTIES OUTPUT_NAME "airspyhf")
endif()

set_target_properties(airspyhf PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties(airspyhf-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

# Dependencies
target_link_libraries(airspyhf ${LIBUSB_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

# Check for functions (cos, etc.)
include(CheckFunctionExists)

if(NOT COS_FUNCTION_EXISTS AND NOT NEED_LINKING_AGAINST_LIBM)
  CHECK_FUNCTION_EXISTS(cos COS_FUNCTION_EXISTS)
  if(NOT COS_FUNCTION_EXISTS)
    unset(COS_FUNCTION_EXISTS CACHE)
    list(APPEND CMAKE_REQUIRED_LIBRARIES m)
    CHECK_FUNCTION_EXISTS(cos COS_FUNCTION_EXISTS)
    if(COS_FUNCTION_EXISTS)
        set(NEED_LINKING_AGAINST_LIBM True CACHE BOOL "" FORCE)
    else()
        message(FATAL_ERROR "Failed making the cos() function available")
    endif()
  endif()
endif()

if (NEED_LINKING_AGAINST_LIBM)
     target_link_libraries(airspyhf m)
endif()

# Platform-specific installation rules
if(UNIX)
   install(TARGETS airspyhf
           LIBRARY DESTINATION lib${LIB_SUFFIX}
           COMPONENT sharedlibs
           )
   install(TARGETS airspyhf-static
           ARCHIVE DESTINATION lib${LIB_SUFFIX}
           COMPONENT staticlibs
           )
   install(FILES ${cpp_headers}
           DESTINATION include/${PROJECT_NAME}
           COMPONENT headers
           )
endif()

if(WIN32)
   install(TARGETS airspyhf
           DESTINATION bin
           COMPONENT sharedlibs
           )
   install(TARGETS airspyhf-static
           DESTINATION bin
           COMPONENT staticlibs
           )
   install(FILES ${cpp_headers}
           DESTINATION include/${PROJECT_NAME}
           COMPONENT headers
           )
endif()

