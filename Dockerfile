#
# Dockerfile
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

FROM ubuntu:22.04
LABEL org.opencontainers.image.authors="Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha"
LABEL org.opencontainers.image.base.name="docker.io/library/ubuntu:22.04"
LABEL org.opencontainers.image.description="The Hybrid Reciprocal Velocity Obstacle"
LABEL org.opencontainers.image.licenses="Apache-2.0"
LABEL org.opencontainers.image.source="https://github.com/snape/HRVO/"
LABEL org.opencontainers.image.title="HRVO Library"
LABEL org.opencontainers.image.url="https://gamma.cs.unc.edu/HRVO/"
LABEL org.opencontainers.image.vendor="University of North Carolina at Chapel Hill"
ENV LANG C.UTF-8
ENV LOGNAME root
ENV SHELL /bin/bash
ENV USER root
RUN export DEBIAN_FRONTEND=noninteractive \
  && apt-get update -qq \
  && apt-get install --no-install-recommends -o Dpkg::Use-Pty=0 -qy \
    ca-certificates \
    clang \
    clang-tidy \
    cmake-format \
    cppcheck \
    cpplint \
    dirmngr \
    doxygen \
    dpkg \
    dpkg-dev \
    file \
    g++ \
    gdb \
    git \
    gnupg \
    graphviz \
    iwyu \
    lldb \
    make \
    nano \
    netbase \
    ninja-build \
    npm \
    pkgconf \
    python3 \
    reuse \
    strace \
    unzip \
    valgrind \
    wget \
    yamllint \
    zlib1g-dev \
  && wget -qO - https://bazel.build/bazel-release.pub.gpg | gpg --dearmor - \
    > /usr/share/keyrings/bazel-archive-keyring.gpg \
  && echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8' \
    > /etc/apt/sources.list.d/bazel.list \
  && wget -qO - https://apt.kitware.com/keys/kitware-archive-latest.asc | gpg --dearmor - \
    > /usr/share/keyrings/kitware-archive-keyring.gpg \
  && echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu jammy main' \
    > /etc/apt/sources.list.d/kitware.list \
  && apt-get update -qq \
  && apt-get install --no-install-recommends -o Dpkg::Use-Pty=0 -qy \
    bazel \
    cmake \
  && rm -rf /var/lib/apt/lists/* \
  && npm install -g \
    jsonlint \
    markdownlint-cli
