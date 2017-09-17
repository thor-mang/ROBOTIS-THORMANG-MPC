^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package thormang3_feet_ft_module
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.3 (2017-05-23)
-----------
* updated cmake file for ros install
* Contributors: SCH

0.1.2 (2017-04-24)
-----------
* merged develop with feature_walking_upgrade
* - fixed package dependencies
* - optimized cpu usage by spin loop (by astumpf)
* bug fix : calculate ft_scale
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
* bug fixed
  change ft calib command
* modified include guard
* ROS C++ coding style is applied
* file name changed.
* thormang3_feet_ft_module : coding style is applied.
* ROS c++ coding stylie is applying, first thormang3_kinematics_dynamics complete
* fixed a bug at thormang3_walking_module
* fixed high CPU consumption due to busy waits
* added Singleton template
* SensorModule->Process() function argument changed.
* Contributors: Alexander Stumpf, Jay Song, Zerom, Pyo, SCH
