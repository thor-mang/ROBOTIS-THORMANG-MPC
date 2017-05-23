^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package thormang3_action_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2017-05-23)
-----------
* updated cmake file for ros install
* Contributors: SCH

0.1.2 (2017-04-24)
-----------
* edited action file and dxl_init
* added function for action_editor to action_module
* bug fix : action file load
* edited comment and some hard coded parameter
* modified done msg in walking module and action module
* add done msg : action module
* code revision : action module
* - fixed package dependencies
* - optimized cpu usage by spin loop (by astumpf)
* wholebody modified
* - reduce CPU consumption
* wholebody inipose added
* update thormang3_head_control_module : scan lidar using range
* Contributors: Jay Song, Zerom, Kayman

0.1.1 (2016-08-19)
-----------
* none

0.1.0 (2016-08-18)
-----------
* first public release for Kinetic
* modified package information for release
* action file edited
  added some new actions
* walking Module Update
  divide balance algorithm
  make thormang3_balance_control
* fixed a bug : check enable when action start
* fixed abnormal message.
* added Function - action play for only specified joints
* modified motion_4096.bin
  added stand_motion
  added value for gripper
* ROS C++ coding style is applied
* added thormang3_action_module_msgs package
* fixed a bug of initialize code in action_module.cpp
* Change initialize code in action_module.cpp
* fixed a bug
* modified motion_4096.bin
  added and modify some actions in motion_4096.bin
* added thormang3_action_module
* Contributors: Jay Song, Zerom, Pyo, SCH
