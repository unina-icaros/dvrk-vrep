V-REP simulator for the da Vinci Research Kit (dVRK)
====================
This repository has code related to daVinci Research Kit (dVRK) V-REP simulator.
**Under development!!!**

[**Check out our YouTube-Video, showing the V-REP Simulator in action**](https://youtu.be/_flffuIevbw)


# Install
The repository contains both some V-REP scenes and ROS code to control the simulated dvrk-ros
* Download V-REP from the official webpage: http://www.coppeliarobotics.com/
* To use the simulator with ROS download and compile the V-REP ROS interface as described in: https://github.com/CoppeliaRobotics/v_repExtRosInterface
* Compile the catkin workspace folder using catkin build tools, NOT catkin_make. Please don't use catkin_make.

# List of Folders:
* V-REP_scenes **[under development]** 
  * Contains the main dVRK scene (dVRK) and five different applications described in [1]
* V-REP_models **[under development]**
  * Contains some V-REP models of the robots (complete dVRK, PSM, ECM, standard needle driver tool, some phantoms)
* V-REP_catkin_ws **[under development]**
  * Contains the ROS code of some application scenes described in [1]
  
[1]  G.A. Fontanelli, M. Selvaggio, M. Ferro, F. Ficuciello, M. Vendittelli, B. Siciliano, *"A V-REP Simulator for the da Vinci Research Kit Robotic Platform"*, BioRob, 2018
  
## Publication
If you use this code in an academic context, please cite the following:

```
@article{Fontanelli2018
  author = {Fontanelli, Giuseppe Andrea and Selvaggio, Mario and Ferro, Marco and Ficuciello, Fanny and Vendittelli, Marilena and Siciliano, Bruno},
  title = {A V-REP Simulator for the da Vinci Research Kit Robotic Platform},
  conference = {BioRob},
  year = {2018}
}
```


## License

Copyright (C) 2017-2018 Philipp Foehn, Robotics and Perception Group, University of Zurich

The RPG MPC repository provides packages that are intended to be used with [ROS](http://www.ros.org/). 
This code has been tested with ROS kinetic on Ubuntu 16.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.
For a commercial license, please contact [Giuseppe Andrea Fontanelli](giuseppeandrea.fontanelli@unina.it).

```
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
```

This work depends on the ACADO Toolkit, developed by the Optimization in Engineering Center (OPTEC) under supervision of Moritz Diehl. Licensing detail can be found on the [ACADO licensing page](http://acado.github.io/licensing.html). It is released under GNU Lesser General Public License as published by the Free Software Foundation, version 3.
ACADO uses qpOASES by Hans Joachim Ferreau et al., released under GPL v2.1.


