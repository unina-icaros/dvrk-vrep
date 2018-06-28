V-REP simulator for the da Vinci Research Kit robotic platform
====================
This repository has code related to daVinci Research Kit (dVRK) V-REP symulator.
**Under development!!!**

# Install
The repository contains both some V-REP scenes and ROS code to control the simulated dvrk-ros
* Download V-REP from the official webpage
* To use the symulator with ROS download and compile the V-REP ROS interface as described in: https://github.com/CoppeliaRobotics/v_repExtRosInterface

# List of Folders:
* V-REP_scenes **[under development]** 
  * Contains the main dVRK scene (dVRK) and five different applications described in @fontanelli2018
* V-REP_models **[under development]**
  * Contains some V-REP models of the robots (complete dVRK, PSM, ECM, standard needle driver tool, some phantoms)
* V-REP_catkin_ws **[under development]**
  * Contains the ROS code of some application scenes described in @fontanelli2018 
  
  
  
---
references:
- id: fontanelli2018
  title: A V-REP Simulator for the da Vinci Research Kit Robotic Platform
  author:
  - family: Fontanelli
    given: Giuseppe Andrea
  - family: Selvaggio
    given: Mario
  - family: Ferro
    given: Marco
  - family: Ficuciello
    given: Fanny
  - family: Vendittelli
    given: Marilena
  - family: Siciliano
    given: Bruno
  container-title: BioRob
  type: article-conference
  issued:
    year: 2018
---
