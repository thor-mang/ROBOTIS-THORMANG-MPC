^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package thormang3_walking_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2017-05-23)
-----------
* updated cmake file for ros install
* Contributors: SCH

0.1.2 (2017-04-24)
-----------
* removed error of negative cut_off_freq
* bug fix : balance param and joint feedback gain update
* bug fix : joint feedback gain update
* when balance turned off, walking can be started
* modified done msg in walking module and action module
* upgrade walking balance control
* - fixed package dependencies
* - optimized cpu usage by spin loop (by astumpf)
* Merge branch 'whole-body' into walking_upgrade
* bug fix : incorrect target force for balance
* Merge branch 'whole-body' into walking_upgrade
* wholebody modified
* - reduce CPU consumption
* wholebody inipose added
* update thormang3_head_control_module : scan lidar using range
* Contributors: Jay Song, Zerom, Kayman

0.1.1 (2016-08-19)
-----------
* modify CMakeLists.txt for sync
* file name is changed
* bug fix
* Contributors: Jay Song

0.1.0 (2016-08-18)
-----------
* first public release for Kinetic
* modified package information for release
* modified thormang3_walking_module/package.xml
* removed some codes that commented out.
* fifth trajectory is applied
* modified File name
  thormang3_online_walking.h
  thormang3_online_walking.cpp
  thormang3_walking_module.h
  thormang3_wakling_module.cpp
* removed walking_module_math & modify thormang3_balance_control
  added set enable function for each balance algorithm
  removed walking module math
* modified target force calculation
  modified thormang3_walking_module/src/robotis_online_walking.cpp
  add target force calculation for fx fy
* added Function - action play for only specified joints
  modified action_module.h
  modified action_module.cpp
  modified message in walking_module.cpp
* StepData can be changed in walking.
  modify thormang3_walking_module/src/wakling_module.cpp
* ROS C++ coding style is applied.
* fixed high CPU consumption due to busy waits
* Contributors: Alexander Stumpf, Jay Song, Zerom, Pyo, SCH
