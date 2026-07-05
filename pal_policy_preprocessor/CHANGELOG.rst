^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_policy_preprocessor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.2 (2026-07-05)
------------------

1.1.1 (2026-07-05)
------------------

1.1.0 (2026-07-05)
------------------
* Merge branch 'migrate/kangaroo_policy/ros2' into 'humble-devel'
  Migrate kangaroo policy node to ros2
  See merge request control/pal_policy_deployer!14
* Use onnxruntime_vendor package instead of onnxruntime-dev dependency
* Contributors: Sai Kishor Kothakota

1.0.0 (2026-05-28)
------------------
* Update Maintainer
* Merge branch 'fix/compilation' into 'humble_devel'
  Fix compilation issue with onnxruntime debian
  See merge request control/pal_policy_deployer!12
* Fix build warning
* Add conditioning for USE_SCOPED_HEADER_INSTALL_DIR
* Add build dependency of onnxruntime-dev
* Use PATH_SUFFIXES instead
* Fix compilation errors due to onnxruntime_cxx_api.h
* Merge branch 'sbl/metadatamap' into 'humble_devel'
  Improve ONNX metadata loading
  See merge request control/pal_policy_deployer!3
* utils pkg
* namespace
* cleanup naming and unused options
* add error handling
* add interactive marker node back in
* working version fixed non updating output
* remove get parameter since static
* allocation inferer
* typo
* headers and start postproc
* launch file refactoring
* input naming
* Merge branch 'sbl/automation' into 'humble_devel'
  stable ROS2 deployer automated with onnx metadata
  See merge request control/pal_policy_deployer!1
* add config files
* fix resources and allocation in preprocessor
* cmake launch and package xml add pkg onnx
* remove files fix cmake and package xml
* fix topic names
* model path in launch file
* style fixes
* add post processor
* new layout inferer and preprocessor
* fix cmake
* remove mutex from component
* automation
* initial commit automation
* new initial marker pose
* new observation vector after merging palmjlab in main
* deployer for 16/12 1742 policy
* updated deployer
* deployer mjlab
* init commit on two targets
* refactor deployer to double target
* fix startup
* add lpf in RL output and spline interpolator for update rates
* add orientation to deployer and interactive marker
* fix initial frame in inferer
* add tf2 for arm
* add correct frame interactive marker and fix cmake
* convert preprocessor and interactivemarker to ros2
* convert inferer to ros2 and remove jointtrajectorycontroller
* first layout inferer
* latest changes for v36
* Contributors: Louis Le Lay, Sai Kishor Kothakota, sergiacosta, sofieblankers

0.1.0 (2025-10-03)
------------------
* updates some policies to test
* tests loco v26 and revert back to loco v21
* adds the possibility to control using joystick
* loops what can be
* cleans code a bit
* wip
* adds interactive marker node
* Contributors: Louis Le Lay, louislelay

0.0.0 (2025-09-12)
------------------
* adds lasts changes to make cb work
* fixes policy model path finder
* add a few dependencies
* add a few dependencies
* adds wip
* adds all previous work as it is
* creates pre processor pkg
* Contributors: louislelay
