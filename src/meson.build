# -*- mode: meson; -*-
# vi: set ft=meson:

# src/meson.build
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

sources = files(
    'Agent.cc',
    'Goal.cc',
    'KdTree.cc',
    'Simulator.cc',
    'Vector2.cc',
)

export_h = configure_file(
    input: 'Export.h.in',
    output: 'Export.h',
    configuration: {'PREFIX': 'HRVO'},
)

public_headers = files(
    'HRVO.h',
    'Simulator.h',
    'Vector2.h',
)

inc = include_directories('.')

compile_args = []
dep_compile_args = []
default_library = get_option('default_library')

# Keep inline-visibility tightening on the library target only: this is part of
# the exported ABI/symbol policy for HRVO itself, not a project-wide policy that
# needs to affect example executables.
compile_args += cpp_compiler.get_supported_arguments(
    '-fvisibility-inlines-hidden',
)

if host_os == 'windows'
    compile_args += ['-DNOMINMAX']
    dep_compile_args += ['-DNOMINMAX']

    if default_library in ['shared', 'both']
        compile_args += ['-DHRVO_EXPORTS']
        if default_library == 'both'
            dep_compile_args += ['-DHRVO_STATIC_DEFINE']
        endif
    else
        compile_args += ['-DHRVO_STATIC_DEFINE']
        dep_compile_args += ['-DHRVO_STATIC_DEFINE']
    endif
else
    if default_library == 'static'
        compile_args += ['-DHRVO_STATIC_DEFINE']
        dep_compile_args += ['-DHRVO_STATIC_DEFINE']
    endif
endif

lib_hrvo = library(
    'HRVO',
    sources: sources,
    include_directories: inc,
    cpp_args: compile_args,
    gnu_symbol_visibility: 'hidden',
    dependencies: openmp_deps,
    version: meson.project_version(),
    soversion: meson.project_version().split('.')[0],
    install: not meson.is_subproject(),
)

dep_hrvo = declare_dependency(
    link_with: lib_hrvo,
    include_directories: inc,
    compile_args: dep_compile_args,
    dependencies: openmp_deps,
)

meson.override_dependency('HRVO', dep_hrvo)

if not meson.is_subproject()
    install_headers(
        public_headers,
        export_h,
        subdir: 'HRVO',
    )

    pkgconfig = import('pkgconfig')
    pkgconfig.generate(
        lib_hrvo,
        name: 'HRVO',
        description: 'Hybrid Reciprocal Velocity Obstacle',
        url: 'https://gamma.cs.unc.edu/HRVO/',
        version: meson.project_version(),
        filebase: 'HRVO',
        subdirs: 'HRVO',
        extra_cflags: dep_compile_args,
    )
endif
