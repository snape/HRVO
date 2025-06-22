# -*- mode: cmake; -*-
# vi: set ft=cmake:

#
# src/CMakeLists.txt
# HRVO Library
#
# SPDX-FileCopyrightText: 2009 University of North Carolina at Chapel Hill
# SPDX-License-Identifier: Apache-2.0
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

add_library(${HRVO_LIBRARY})

include(GenerateExportHeader)

generate_export_header(${HRVO_LIBRARY}
  EXPORT_FILE_NAME Export.h)

target_sources(${HRVO_LIBRARY}
  PUBLIC
    FILE_SET HEADERS
    BASE_DIRS
      "${CMAKE_CURRENT_BINARY_DIR}"
      "${CMAKE_CURRENT_SOURCE_DIR}"
    FILES
      "${CMAKE_CURRENT_BINARY_DIR}/Export.h"
      HRVO.h
      Simulator.h
      Vector2.h
    PRIVATE
      Agent.cc
      Agent.h
      Export.cc
      Goal.cc
      Goal.h
      KdTree.cc
      KdTree.h
      Simulator.cc
      Vector2.cc)

set_target_properties(${HRVO_LIBRARY} PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  SOVERSION ${PROJECT_VERSION_MAJOR}
  VERSION ${PROJECT_VERSION}
  VISIBILITY_INLINES_HIDDEN ON)

if(ENABLE_INTERPROCEDURAL_OPTIMIZATION
    AND HRVO_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
  set_target_properties(${HRVO_LIBRARY} PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ON)
endif()

if(WIN32)
  target_compile_definitions(${HRVO_LIBRARY} PUBLIC NOMINMAX)
endif()

if(ENABLE_OPENMP AND OpenMP_FOUND)
  target_link_libraries(${HRVO_LIBRARY} PRIVATE OpenMP::OpenMP_CXX)
endif()

export(TARGETS ${HRVO_LIBRARY} NAMESPACE ${PROJECT_NAME}::
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

install(TARGETS ${HRVO_LIBRARY}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE
    COMPONENT development
    DESTINATION ${HRVO_LIBRARY_DIR}
    FILE_SET HEADERS
    COMPONENT development
    DESTINATION ${HRVO_INCLUDE_DIR}
  LIBRARY
    COMPONENT runtime
    DESTINATION ${HRVO_LIBRARY_DIR}
    NAMELINK_COMPONENT development
  RUNTIME
    COMPONENT runtime
    DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${HRVO_INCLUDE_DIR})

install(EXPORT ${PROJECT_NAME}Targets
  COMPONENT development
  DESTINATION ${HRVO_DIR}
  NAMESPACE ${PROJECT_NAME}::)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.31
    AND ENABLE_CMAKE_EXPERIMENTAL_FEATURES)
  set(CMAKE_EXPERIMENTAL_EXPORT_PACKAGE_INFO
    b80be207-778e-46ba-8080-b23bba22639e)

  install(PACKAGE_INFO ${HRVO_LIBRARY}
    EXPORT ${PROJECT_NAME}Targets
    COMPONENT development
    DESTINATION ${HRVO_CPS_DIR}
    VERSION ${PROJECT_VERSION}
    COMPAT_VERSION ${PROJECT_VERSION_MAJOR}
    VERSION_SCHEMA simple)
endif()
