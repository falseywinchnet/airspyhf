#top dir cmake project for libairspyhf

cmake_minimum_required(VERSION 3.10)
project(airspyhf_all C CXX)
include(GNUInstallDirs)

if(POLICY CMP0068)
    cmake_policy(SET CMP0068 NEW)
endif()

if(APPLE)
    set(CMAKE_MACOSX_RPATH ON)
    set(CMAKE_INSTALL_NAME_DIR "@rpath" CACHE PATH "macOS dynamic library install name directory" FORCE)
    set(CMAKE_INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}" CACHE STRING "macOS installed executable rpath" FORCE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH OFF CACHE BOOL "Use install rpath while building" FORCE)
endif()

#provide missing strtoull() for VC11
if(MSVC11)
    add_definitions(-Dstrtoull=_strtoui64)
endif(MSVC11)

add_subdirectory(libairspyhf)
add_subdirectory(tools)
########################################################################
# Create uninstall target
########################################################################

configure_file(
    ${PROJECT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)


add_custom_target(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)

########################################################################
# Copy Files Win32 only
########################################################################

if(WIN32 AND NOT CMAKE_CROSSCOMPILING)
 if (NOT DEFINED LIBUSB_LIBRARIES)
 configure_file(
     ${CMAKE_CURRENT_BINARY_DIR}/../libs_win32/libusb-1.0.dll
 COPYONLY)
 endif()

 if (NOT DEFINED THREADS_PTHREADS_WIN32_LIBRARY)
 configure_file(
     ${CMAKE_CURRENT_BINARY_DIR}/../libs_win32/pthreadGC2.dll
 COPYONLY)
 endif()
endif()
