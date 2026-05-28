^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_policy_deployer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-05-28)
------------------
* Update Maintainer
* Merge branch 'fix/compilation' into 'humble_devel'
  Fix compilation issue with onnxruntime debian
  See merge request control/pal_policy_deployer!12
* Add conditioning for USE_SCOPED_HEADER_INSTALL_DIR
* Merge branch 'sbl/metadatamap' into 'humble_devel'
  Improve ONNX metadata loading
  See merge request control/pal_policy_deployer!3
* add error handling
* update readme
* add interactive marker node back in
* yaml files in launch file
* allocation inferer
* headers and start postproc
* metamap parameters in inferer
* launch file refactoring
* launch opaque
* command in constructor
* Merge branch 'sbl/automation' into 'humble_devel'
  stable ROS2 deployer automated with onnx metadata
  See merge request control/pal_policy_deployer!1
* add config files
* cmake launch and package xml add pkg onnx
* cmake files
* remove files fix cmake and package xml
* change to python launch file
* model path in launch file
* add post processor
* new policies
* new initial marker pose
* new policy
* new policies
* new observation vector after merging palmjlab in main
* deployer for 16/12 1742 policy
* init commit on two targets
* refactor deployer to double target
* fix startup
* add lpf in RL output and spline interpolator for update rates
* add new policies
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
* adds the possibility to control using joystick
* removes postprocesor
* wip
* adds general launch file for kang
* creates launch file
* Contributors: Louis Le Lay, louislelay

0.0.0 (2025-09-12)
------------------
* adds lasts changes to make cb work
* corrects typo
* adds a few launch file
* add a few dependencies
* adds wip
* adds all previous work as it is
* create meta pkg
* Contributors: louislelay
