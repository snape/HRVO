# -*- mode: cmake; -*-
# vi: set ft=cmake:

#
# examples/CMakeLists.txt
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

if(BUILD_TESTING)
  option(OUTPUT_TIME_AND_POSITIONS "Output time and positions" ON)

  if(OUTPUT_TIME_AND_POSITIONS)
    set(HRVO_EXAMPLES_COMPILE_DEFINITIONS HRVO_OUTPUT_TIME_AND_POSITIONS=1)
  else()
    set(HRVO_EXAMPLES_COMPILE_DEFINITIONS HRVO_OUTPUT_TIME_AND_POSITIONS=0)
  endif()

  if(ENABLE_INTERPROCEDURAL_OPTIMIZATION
      AND HRVO_INTERPROCEDURAL_OPTIMIZATION_SUPPORTED)
    set(HRVO_EXAMPLES_INTERPROCEDURAL_OPTIMIZATION ON)
  else()
    set(HRVO_EXAMPLES_INTERPROCEDURAL_OPTIMIZATION OFF)
  endif()

  add_executable(Circle Circle.cc)
  target_compile_definitions(Circle PRIVATE
    ${HRVO_EXAMPLES_COMPILE_DEFINITIONS})
  target_link_libraries(Circle PRIVATE ${HRVO_LIBRARY})
  if(ENABLE_OPENMP AND OpenMP_FOUND)
    target_link_libraries(Circle PRIVATE OpenMP::OpenMP_CXX)
  endif()
  set_target_properties(Circle PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION ${HRVO_EXAMPLES_INTERPROCEDURAL_OPTIMIZATION})
  add_test(NAME Circle COMMAND Circle)
  set_tests_properties(Circle PROPERTIES
    LABELS medium
    TIMEOUT 60)
endif()
