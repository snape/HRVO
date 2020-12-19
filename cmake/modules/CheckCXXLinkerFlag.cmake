#
# CheckCXXLinkerFlag.cmake
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

include_guard(GLOBAL)

include(CMakeCheckCompilerFlagCommonPatterns)

function(check_cxx_linker_flag _FLAG _VAR)
  if(MSVC)
    _check_cxx_linker_flag(/WX HRVO_LINKER_SUPPORTS_WX)

    if(HRVO_LINKER_SUPPORTS_WX)
      set(_FATAL_WARNINGS_FLAG /WX)
    else()
      set(_FATAL_WARNINGS_FLAG)
    endif()
  else()
    _check_cxx_linker_flag(-Wl,--fatal-warnings
      HRVO_LINKER_SUPPORTS__FATAL_WARNINGS)

    if(HRVO_LINKER_SUPPORTS__FATAL_WARNINGS)
      set(_FATAL_WARNINGS_FLAG -Wl,--fatal-warnings)
    else()
      _check_cxx_linker_flag(-Wl,-fatal_warnings
        HRVO_LINKER_SUPPORTS_FATAL_WARNINGS)

      if(HRVO_LINKER_SUPPORTS_FATAL_WARNINGS)
        set(_FATAL_WARNINGS_FLAG -Wl,-fatal_warnings)
      else()
        set(_FATAL_WARNINGS_FLAG)
      endif()
    endif()
  endif()

  _check_cxx_linker_flag("${_FLAG}" ${_VAR} "${_FATAL_WARNINGS_FLAG}")

  set(${_VAR} "${${_VAR}}" PARENT_SCOPE)
endfunction()

function(_check_cxx_linker_flag _FLAG _VAR)
  if(ARGC GREATER 2)
    list(INSERT _FLAG 0 "${ARGV2}")
  endif()

  if(CMAKE_VERSION VERSION_LESS 3.14)
    set(CMAKE_REQUIRED_LIBRARIES "${_FLAG}")
  else()
    set(CMAKE_REQUIRED_LINK_OPTIONS "${_FLAG}")
  endif()

  if(NOT WIN32)
    set(_LC_VARS LANG LC_ALL LC_MESSAGES)
    foreach(_LC_VAR IN LISTS _LC_VARS)
      set(_SAVED_ENV_${_LC_VAR} "$ENV{${_LC_VAR}}")
      set(ENV{${_LC_VAR}} C)
    endforeach()
  endif()

  check_compiler_flag_common_patterns(_COMMON_PATTERNS)

  include(CheckCXXSourceCompiles)

  check_cxx_source_compiles("int main() { return 0; }" ${_VAR}
    ${_COMMON_PATTERNS})

  if(NOT WIN32)
    foreach(_LC_VAR IN LISTS _LC_VARS)
      set(ENV{${_LC_VAR}} ${_SAVED_ENV_${_LC_VAR}})
    endforeach()
  endif()

  set(${_VAR} "${${_VAR}}" PARENT_SCOPE)
endfunction()