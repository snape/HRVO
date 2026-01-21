<!--
README.md
HRVO Library

SPDX-FileCopyrightText: 2009 University of North Carolina at Chapel Hill
SPDX-License-Identifier: CC-BY-SA-4.0

Creative Commons Attribution-ShareAlike 4.0 International Public License

You are free to:

* Share -- copy and redistribute the material in any medium or format

* ShareAlike -- If you remix, transform, or build upon the material, you must
  distribute your contributions under the same license as the original

* Adapt -- remix, transform, and build upon the material for any purpose, even
  commercially.

The licensor cannot revoke these freedoms as long as you follow the license
terms.

Under the following terms:

* Attribution -- You must give appropriate credit, provide a link to the
  license, and indicate if changes were made. You may do so in any reasonable
  manner, but not in any way that suggests the licensor endorses you or your
  use.

* No additional restrictions -- You may not apply legal terms or technological
  measures that legally restrict others from doing anything the license
  permits.

Notices:

* You do not have to comply with the license for elements of the material in
  the public domain or where your use is permitted by an applicable exception
  or limitation.

* No warranties are given. The license may not give you all of the permissions
  necessary for your intended use. For example, other rights such as publicity,
  privacy, or moral rights may limit how you use the material.

Please send all bug reports to <geom@cs.unc.edu>.

The authors may be contacted via:

Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
Dept. of Computer Science
201 S. Columbia St.
Frederick P. Brooks, Jr. Computer Science Bldg.
Chapel Hill, N.C. 27599-3175
United States of America

<https://gamma.cs.unc.edu/HRVO/>
-->

The Hybrid Reciprocal Velocity Obstacle
=======================================

<https://gamma.cs.unc.edu/HRVO/>

[![DOI](https://zenodo.org/badge/10278421.svg)](https://zenodo.org/badge/latestdoi/10278421)


We present the hybrid reciprocal velocity obstacle (HRVO) for collision-free and
oscillation-free navigation of multiple mobile robots or virtual agents. Each
robot senses its surroundings and acts independently without central
coordination or communication with other robots. Our approach uses both the
current position and the velocity of other robots to compute their future
trajectories in order to avoid collisions. Moreover, our approach is reciprocal
and avoids oscillations by explicitly taking into account that the other robots
also sense their surroundings and change their trajectories accordingly. We
apply hybrid reciprocal velocity obstacles to iRobot Create mobile robots and
demonstrate direct, collision-free, and oscillation-free navigation.

![Build Status](https://github.com/snape/HRVO/actions/workflows/ci.yml/badge.svg?branch=main)

<!-- REUSE-IgnoreStart -->
SPDX-FileCopyrightText: 2009 University of North Carolina at Chapel Hill  
SPDX-License-Identifier: Apache-2.0

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

&nbsp;&nbsp;<https://www.apache.org/licenses/LICENSE-2.0>

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Please send all bug reports to [geom@cs.unc.edu](mailto:geom@cs.unc.edu).

The authors may be contacted via:

Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha  
Dept. of Computer Science  
201 S. Columbia St.  
Frederick P. Brooks, Jr. Computer Science Bldg.  
Chapel Hill, N.C. 27599-3175  
United States of America
<!-- REUSE-IgnoreEnd -->
