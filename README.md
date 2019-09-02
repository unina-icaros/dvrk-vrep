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
  
[1]  G. A. Fontanelli, M. Selvaggio, M. Ferro, F. Ficuciello, M. Vendittelli and B. Siciliano, "A V-REP Simulator for the da Vinci Research Kit Robotic Platform," 2018 7th IEEE International Conference on Biomedical Robotics and Biomechatronics (Biorob), Enschede, 2018, pp. 1056-1061, doi: 10.1109/BIOROB.2018.8487187, URL: http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8487187&isnumber=8487180
  
## Publication
If you use this code in an academic context, please cite the following:

```
@INPROCEEDINGS{8487187,
author={G. A. Fontanelli and M. Selvaggio and M. Ferro and F. Ficuciello 
and M. Vendittelli and B. Siciliano},
booktitle={2018 7th IEEE International Conference on Biomedical Robotics 
and Biomechatronics (Biorob)},
title={A V-REP Simulator for the da Vinci Research Kit Robotic Platform},
year={2018},
volume={},
number={},
pages={1056-1061},
keywords={Robot sensing systems;Electronic 
countermeasures;Kinematics;Surgery;Manipulators;Solid modeling},
doi={10.1109/BIOROB.2018.8487187},
ISSN={2155-1782},
month={Aug},}

```


## License

Copyright (C) 2017-2018 Giuseppe Andrea Fontanelli, ICAROS center, University of Naples Federico II

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




