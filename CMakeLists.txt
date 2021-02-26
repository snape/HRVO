# -*- mode: cmake; -*-
# vi: set ft=cmake:

#
# CMakeLists.txt
# HRVO Library
#
# Copyright 2009 University of North Carolina at Chapel Hill
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Please send all bug reports to <geom@cs.unc.edu>.
#
# The authors may be contacted via:
#
# Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
# Dept. of Computer Science
# 201 S. Columbia St.
# Frederick P. Brooks, Jr. Computer Science Bldg.
# Chapel Hill, N.C. 27599-3175
# United States of America
#
# <https://gamma.cs.unc.edu/HRVO/>
#

cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(HRVO
  VERSION 1.1.0
  DESCRIPTION "The Hybrid Reciprocal Velocity Obstacle"
  LANGUAGES CXX)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # cmake-lint: disable=C0301
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build; options are Debug Release RelWithDebInfo MinSizeRel"
    FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
    STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

include(CTest)

if(WIN32)
  set(BUILD_SHARED_LIBS OFF)
else()
  option(BUILD_SHARED_LIBS "Build all libraries as shared" ON)
endif()

include(GNUInstallDirs)

set(HRVO_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
set(HRVO_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
set(HRVO_LIBRARY_DIR ${CMAKE_INSTALL_LIBDIR})
set(HRVO_LIBRARY HRVO)

set(HRVO_NAME "HRVO Library")
set(HRVO_HOMEPAGE_URL https://gamma.cs.unc.edu/HRVO/)

set(CMAKE_CXX_STANDARD 98)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ENABLE_INTERPROCEDURAL_OPTIMIZATION
  "Enable interprocedural optimization if supported" OFF)

if(ENABLE_INTERPROCEDURAL_OPTIMIZATION)
  include(CheckIPOSupported)

  check_ipo_supported(RESULT HRVO_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED
    LANGUAGES CXX)

  if(HRVO_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
    message(STATUS "Interprocedural optimization is supported")
  else()
    message(STATUS "Interprocedural optimization is NOT supported")
  endif()
else()
  set(HRVO_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
endif()

include(CheckCXXCompilerFlag)
include(CheckCXXLinkerFlag)

option(ENABLE_HARDENING
  "Enable hardening compiler and linker flags if supported" OFF)

if(ENABLE_HARDENING)
  if(MSVC)
    check_cxx_compiler_flag(/GS HRVO_COMPILER_SUPPORTS_GS)
    check_cxx_compiler_flag(/guard:cf HRVO_COMPILER_SUPPORTS_GUARD_CF)

    if(HRVO_COMPILER_SUPPORTS_GS)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GS")
    endif()

    check_cxx_linker_flag(/DYNAMICBASE HRVO_LINKER_SUPPORTS_DYNAMICBASE)
    check_cxx_linker_flag(/GUARD:CF HRVO_LINKER_SUPPORTS_GUARD_CF)
    check_cxx_linker_flag(/NXCOMPAT HRVO_LINKER_SUPPORTS_NXCOMPAT)

    if(HRVO_COMPILER_SUPPORTS_GUARD_CF
        AND HRVO_LINKER_SUPPORTS_DYNAMICBASE
        AND HRVO_LINKER_SUPPORTS_GUARD_CF)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /guard:cf")
      set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} /DYNAMICBASE /GUARD:CF")
      set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} /DYNAMICBASE /GUARD:CF")
    endif()

    if(HRVO_LINKER_SUPPORTS_NXCOMPAT)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NXCOMPAT")
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /NXCOMPAT")
    endif()
  else()
    check_cxx_compiler_flag(-D_FORTIFY_SOURCE=2
      HRVO_COMPILER_SUPPORTS_D_FORTIFY_SOURCE_2)
    check_cxx_compiler_flag(-fcf-protection
      HRVO_COMPILER_SUPPORTS_FCF_PROTECTION)
    check_cxx_compiler_flag(-fno-common HRVO_COMPILER_SUPPORTS_FNO_COMMON)
    check_cxx_compiler_flag(-fsanitize=safe-stack
      HRVO_COMPILER_SUPPORTS_FSANITIZE_SAFE_STACK)
    check_cxx_compiler_flag(-fstack-clash-protection
      HRVO_COMPILER_SUPPORTS_FSTACK_CLASH_PROTECTION)
    check_cxx_compiler_flag(-fstack-protector-strong
      HRVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR_STRONG)

    if(HRVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR_STRONG)
      set(HRVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR)
    else()
      check_cxx_compiler_flag(-fstack-protector
        HRVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR)
    endif()

    if(HRVO_COMPILER_SUPPORTS_D_FORTIFY_SOURCE_2)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORTIFY_SOURCE=2")
    endif()
    if(HRVO_COMPILER_SUPPORTS_FCF_PROTECTION)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcf-protection")
    endif()
    if(HRVO_COMPILER_SUPPORTS_FNO_COMMON)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-common")
    endif()
    if(HRVO_COMPILER_SUPPORTS_FSTACK_CLASH_PROTECTION)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-clash-protection")
    endif()
    if(HRVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR_STRONG)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector-strong")
    elseif(HRVO_COMPILER_SUPPORTS_FSTACK_PROTECTOR)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector")
    endif()

    check_cxx_linker_flag(-Wl,-Bsymbolic-functions
      HRVO_LINKER_SUPPORTS_BSYMBOLIC_FUNCTIONS)
    check_cxx_linker_flag(-fsanitize=safe-stack
      HRVO_LINKER_SUPPORTS_FSANITIZE_SAFE_STACK)
    check_cxx_linker_flag(-Wl,-z,defs HRVO_LINKER_SUPPORTS_Z_DEFS)
    check_cxx_linker_flag(-Wl,-z,noexecheap HRVO_LINKER_SUPPORTS_Z_NOEXECHEAP)
    check_cxx_linker_flag(-Wl,-z,noexecstack HRVO_LINKER_SUPPORTS_Z_NOEXECSTACK)
    check_cxx_linker_flag(-Wl,-z,now HRVO_LINKER_SUPPORTS_Z_NOW)
    check_cxx_linker_flag(-Wl,-z,relro HRVO_LINKER_SUPPORTS_Z_RELRO)

    if(HRVO_LINKER_SUPPORTS_BSYMBOLIC_FUNCTIONS)
      set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Bsymbolic-functions")
      set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-Bsymbolic-functions")
    endif()
    if(HRVO_COMPILER_SUPPORTS_FSANITIZE_SAFE_STACK
        AND HRVO_LINKER_SUPPORTS_FSANITIZE_SAFE_STACK)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=safe-stack")
      set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=safe-stack")
      set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=safe-stack")
    endif()
    if(HRVO_LINKER_SUPPORTS_Z_DEFS)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,defs")
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,defs")
    endif()
    if(HRVO_LINKER_SUPPORTS_Z_NOEXECHEAP)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecheap")
      set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecheap")
    endif()
    if(HRVO_LINKER_SUPPORTS_Z_NOEXECSTACK)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,noexecstack")
      set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,noexecstack")
    endif()
    if(HRVO_LINKER_SUPPORTS_Z_NOW)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,now")
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,now")
    endif()
    if(HRVO_LINKER_SUPPORTS_Z_RELRO)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,relro")
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,relro")
    endif()
  endif()
endif()

if(MSVC)
  check_cxx_compiler_flag(/W4 HRVO_COMPILER_SUPPORTS_W4)

  if(HRVO_COMPILER_SUPPORTS_W4)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  endif()
else()
  check_cxx_compiler_flag(-Wall HRVO_COMPILER_SUPPORTS_WALL)
  check_cxx_compiler_flag(-Wformat-security
    HRVO_COMPILER_SUPPORTS_WFORMAT_SECURITY)
  check_cxx_compiler_flag(-Werror=format-security
    HRVO_COMPILER_SUPPORTS_WERROR_FORMAT_SECURITY)
  check_cxx_compiler_flag(-Wno-unused
    HRVO_COMPILER_SUPPORTS_WNO_UNUSED)

  if(HRVO_COMPILER_SUPPORTS_WALL)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
  endif()
  if(HRVO_COMPILER_SUPPORTS_WFORMAT_SECURITY)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat-security")
  endif()
  if(HRVO_COMPILER_SUPPORTS_WERROR_FORMAT_SECURITY)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=format-security")
  endif()
  if(HRVO_COMPILER_SUPPORTS_WNO_UNUSED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused")
  endif()
endif()

if(NOT MSVC)
  check_cxx_linker_flag(-Wl,--as-needed HRVO_LINKER_SUPPORTS__AS_NEEDED)

  if(HRVO_LINKER_SUPPORTS__AS_NEEDED)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed")
    set(CMAKE_SHARED_LINKER_FLAGS
      "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")
  endif()
endif()

option(WARNINGS_AS_ERRORS "Turn compiler warnings into errors" OFF)

if(WARNINGS_AS_ERRORS)
  if(MSVC)
    check_cxx_compiler_flag(/WX HRVO_COMPILER_SUPPORTS_WX)

    if(HRVO_COMPILER_SUPPORTS_WX)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
    endif()
  else()
    check_cxx_compiler_flag(-Werror HRVO_COMPILER_SUPPORTS_WERROR)

    if(HRVO_COMPILER_SUPPORTS_WERROR)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
    endif()
  endif()
endif()

add_subdirectory(src)
add_subdirectory(examples)
add_subdirectory(doc)

install(FILES LICENSE
  COMPONENT runtime
  DESTINATION ${CMAKE_INSTALL_DOCDIR})

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${PROJECT_NAME}Config.cmake.in ${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${HRVO_DIR}
  PATH_VARS HRVO_INCLUDE_DIR HRVO_LIBRARY_DIR)

write_basic_package_version_file(${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  COMPONENT development
  DESTINATION ${HRVO_DIR})

find_package(PkgConfig MODULE)

if(PkgConfig_FOUND)
  configure_file(${PROJECT_NAME}.pc.in ${PROJECT_NAME}.pc @ONLY)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
    COMPONENT development
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
endif()

include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_CONTACT "Jamie Snape")
set(CPACK_PACKAGE_VENDOR "University of North Carolina at Chapel Hill")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_STRIP_FILES ON)

set(CPACK_SOURCE_IGNORE_FILES
  _build/ \\\\.git/ \\\\.gitattributes \\\\.github/ \\\\.gitignore bazel-
  Brewfile)
set(CPACK_SOURCE_STRIP_FILES ON)

set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_DEPENDS)
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE ${HRVO_HOMEPAGE_URL})
set(CPACK_DEBIAN_PACKAGE_SECTION contrib/devel)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

set(CPACK_FREEBSD_PACKAGE_DEPS)
set(CPACK_FREEBSD_PACKAGE_LICENSE APACHE20)
set(CPACK_FREEBSD_PACKAGE_WWW ${HRVO_HOMEPAGE_URL})

set(CPACK_RPM_FILE_NAME RPM-DEFAULT)
set(CPACK_RPM_PACKAGE_AUTOREQPROV ON)
set(CPACK_RPM_PACKAGE_LICENSE "ASL 2.0")
set(CPACK_RPM_PACKAGE_REQUIRES)
set(CPACK_RPM_PACKAGE_URL ${HRVO_HOMEPAGE_URL})

include(CPack)
