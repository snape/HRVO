# -*- mode: meson; -*-
# vi: set ft=meson:

#
# examples/meson.build
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

examples_opt = get_option('examples')
if examples_opt.disabled() or (examples_opt.auto() and meson.is_subproject())
    subdir_done()
endif

example_args = [
    '-DHRVO_OUTPUT_TIME_AND_POSITIONS=0',
]

exe = executable(
    'Circle',
    sources: 'Circle.cc',
    cpp_args: example_args,
    dependencies: dep_hrvo,
    install: false,
)

test(
    'Circle',
    exe,
    is_parallel: false,
    suite: 'examples',
    timeout: 60,
)
