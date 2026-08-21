^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_policy_inferer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.2.0 (2026-08-21)
------------------
* Merge branch 'cleanup/dependencies' into 'humble-devel'
  Cleanup unused headers and their dependencies
  See merge request control/pal_policy_deployer!27
* Cleanup unused headers and their dependencies
* Contributors: Sai Kishor Kothakota

1.1.3 (2026-07-20)
------------------
* Merge branch 'fix/missing/library' into 'humble-devel'
  Fix the issue of the missing library libonnxruntime.so.1
  See merge request control/pal_policy_deployer!25
* Fix the issue of the missing library libonnxruntime.so.1
* Contributors: Sai Kishor Kothakota

1.1.2 (2026-07-05)
------------------
* Merge branch 'fix/build_warnings' into 'humble-devel'
  Fix build warnings
  See merge request control/pal_policy_deployer!21
* Fix build warnings
* Contributors: Sai Kishor Kothakota

1.1.1 (2026-07-05)
------------------

1.1.0 (2026-07-05)
------------------
* Merge branch 'add/introspection' into 'humble-devel'
  Add projected gravity and orientation normalization
  See merge request control/pal_policy_deployer!17
* Register received cmd_vel as PAL statistics for non-imitation policies
* Add projected gravity and orientation normalization
* Merge branch 'migrate/kangaroo_policy/ros2' into 'humble-devel'
  Migrate kangaroo policy node to ros2
  See merge request control/pal_policy_deployer!14
* Add logic to send zero twist to the policy on cmd_vel timeout
* Initialize elapsed_time to zero
* Use onnxruntime_vendor package instead of onnxruntime-dev dependency
* Change the IMU naming to torso_imu
* Add pid_controller exec dependency
* Reuse the part from the introspection data published from the controller manager
* Add a working version of the joint states and imu to build the observation vector
* Add changes to use the PID controller
* Add first version of the migration
* Contributors: Sai Kishor Kothakota

1.0.0 (2026-05-28)
------------------
* Update Maintainer
* Merge branch 'fix/compilation' into 'humble_devel'
  Fix compilation issue with onnxruntime debian
  See merge request control/pal_policy_deployer!12
* Add build dependency of onnxruntime-dev
* Use PATH_SUFFIXES instead
* Fix compilation errors due to onnxruntime_cxx_api.h
* Merge branch 'sbl/metadatamap' into 'humble_devel'
  Improve ONNX metadata loading
  See merge request control/pal_policy_deployer!3
* remove options from inferer
* namespace
* add error handling
* resource mgmt inferer
* working version fixed non updating output
* allocation inferer
* metamap parameters in inferer
* launch file refactoring
* command in constructor
* Merge branch 'sbl/automation' into 'humble_devel'
  stable ROS2 deployer automated with onnx metadata
  See merge request control/pal_policy_deployer!1
* fix topic names
* split inferer to base and child class
* remove typo
* cmake launch and package xml add pkg onnx
* remove files fix cmake and package xml
* fix topic names
* model path in launch file
* style fixes
* infer obesrvation vector size
* hardcode observation vector dimension
* add post processor
* new layout inferer and preprocessor
* new observation vector after merging palmjlab in main
* updated deployer
* deployer mjlab
* init commit on two targets
* refactor deployer to double target
* latest changes
* fix startup
* add logic to enable staircase ZOH
* change update logic to delay instead of step
* possible controller refactor
* fix timer
* add all joints to interpolator
* add lpf in RL output and spline interpolator for update rates
* add orientation to deployer and interactive marker
* fix initial frame in inferer
* add tf2 for arm
* add correct frame interactive marker and fix cmake
* convert inferer to ros2 and remove jointtrajectorycontroller
* first layout inferer
* latest changes for v36
* Contributors: Louis Le Lay, Sai Kishor Kothakota, sergiacosta, sofieblankers

0.1.0 (2025-10-03)
------------------
* removes 31 and 29 policies
* updates some policies to test
* tests loco v26 and revert back to loco v21
* updates some text
* improves runtime and update readne
* loops what can be
* removes postprocesor
* wip
* Contributors: Louis Le Lay, louislelay

0.0.0 (2025-09-12)
------------------
* adds lasts changes to make cb work
* fixes policy model path finder
* add a few dependencies
* add a few dependencies
* adds wip
* adds all previous work as it is
* creates inference pkg
* Contributors: louislelay
