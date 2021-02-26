# -*- mode: cmake; -*-
# vi: set ft=cmake:

#
# src/CMakeLists.txt
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

set(HRVO_HEADERS
  HRVO.h
  Simulator.h
  Vector2.h)

set(HRVO_SOURCES
  Agent.cpp
  Agent.h
  Definitions.h
  Goal.cpp
  Goal.h
  KdTree.cpp
  KdTree.h
  Simulator.cpp
  Vector2.cpp)

add_library(${HRVO_LIBRARY} ${HRVO_HEADERS} ${HRVO_SOURCES})

include(GenerateExportHeader)

generate_export_header(${HRVO_LIBRARY} EXPORT_FILE_NAME Export.h)

set_target_properties(${HRVO_LIBRARY} PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  PUBLIC_HEADER "${HRVO_HEADERS}"
  SOVERSION ${PROJECT_VERSION_MAJOR}
  VERSION ${PROJECT_VERSION}
  VISIBILITY_INLINES_HIDDEN ON)

if(ENABLE_INTERPROCEDURAL_OPTIMIZATION
    AND RVO_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
  set_target_properties(${HRVO_LIBRARY} PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ON)
endif()

target_include_directories(${HRVO_LIBRARY} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if(WIN32)
  target_compile_definitions(${HRVO_LIBRARY} PUBLIC NOMINMAX)
endif()

export(TARGETS ${HRVO_LIBRARY} NAMESPACE ${PROJECT_NAME}::
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.12)
  set(HRVO_NAMELINK_COMPONENT_ARGUMENT NAMELINK_COMPONENT development)
else()
  set(HRVO_NAMELINK_COMPONENT_ARGUMENT)
endif()

install(TARGETS ${HRVO_LIBRARY}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE
    COMPONENT development
    DESTINATION ${HRVO_LIBRARY_DIR}
  LIBRARY
    COMPONENT runtime
    DESTINATION ${HRVO_LIBRARY_DIR}
    ${HRVO_NAMELINK_COMPONENT_ARGUMENT}
  PUBLIC_HEADER
    COMPONENT development
    DESTINATION ${HRVO_INCLUDE_DIR}
  RUNTIME
    COMPONENT runtime
    DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${HRVO_INCLUDE_DIR})

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/Export.h"
  COMPONENT development
  DESTINATION ${HRVO_INCLUDE_DIR})

install(EXPORT ${PROJECT_NAME}Targets
  COMPONENT development
  DESTINATION ${HRVO_DIR}
  NAMESPACE ${PROJECT_NAME}::)
