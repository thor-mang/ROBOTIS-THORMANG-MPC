^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package thormang3_mpc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* bug fix
* package name changed
  imu_3dm_gx4 -> thormang3_imu_3dm_gx4
* Contributors: Jay Song

0.1.0 (2016-08-18)
-----------
* first public release for Kinetic
* modified package information for release
* Contributors: Jay Song, Zerom, Pyo, Kayman, SCH
