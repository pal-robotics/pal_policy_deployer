^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_policy_deployer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.1.2 (2026-07-05)
------------------

1.1.1 (2026-07-05)
------------------

1.1.0 (2026-07-05)
------------------
* Change default policy to v315
* Merge branch 'reduce/update_rate' into 'humble-devel'
  Reduce the PID control to 1000 Hz for stability
  See merge request control/pal_policy_deployer!20
* Reduce the PID control to 1000 Hz for stability
* Merge branch 'add/flat_walking_policies' into 'humble-devel'
  Add flat walking policies
  See merge request control/pal_policy_deployer!19
* Add flatç walking policy v534
* Add flat walking policy v315
* Merge branch 'stairs_policies' into 'humble-devel'
  Stairs policies
  See merge request control/pal_policy_deployer!18
* Forward policies
  636 is the best one
  639 goes up quite well deviates to the left
* Backwards policies
  Recommended speeds:
  - 0.4 lin x
  - 0.3 ang z
  - 0.0 lin y
* Merge branch 'migrate/kangaroo_policy/ros2' into 'humble-devel'
  Add nice jumping policy
  See merge request control/pal_policy_deployer!16
* Add nice jumping policy
* Merge branch 'migrate/kangaroo_policy/ros2' into 'humble-devel'
  Migrate kangaroo policy node to ros2
  See merge request control/pal_policy_deployer!14
* Merge branch 'add/type_impedance_params' into 'migrate/kangaroo_policy/ros2'
  LB config and proper launch
  See merge request control/pal_policy_deployer!15
* Robot type launch arg
* Add immitation policy 472
* Add support for multi policies
* Add policy 265 for testing
* Kill the spawner if the policy node dies
* Make sure the model exists before launching the node and the controller
* add arguments for task and the model
* Add launch and config information of the rl controller pid
* Contributors: Sai Kishor Kothakota, Óscar Martínez

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
