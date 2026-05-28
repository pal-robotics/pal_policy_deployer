^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_policy_inferer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
