# pal_policy_deployer

This repository provides a modular ROS 2 framework for deploying trained machine-learning policies (e.g. ONNX / VLA models) on a robot.
It separates observation construction, model inference, and action post-processing into cleanly defined packages with shared base classes and extensible child implementations.

The framework is designed to be model- and policy-agnostic, thus easy to extend.

The repository consists of four ROS2 (ament) packages:

### Overview

```bash
pal_policy_deployer/
├── pal_policy_deployer
├── pal_policy_inferer
├── pal_policy_postprocessor
└── pal_policy_preprocessor
```

- **pal_policy_deployer** contains launch files and trained models. Installing this package installs the four packages.
- **pal_policy_preprocessor** builds the input(/observation) vector from robot and environment information and
publishes it to the inferer. 
- **pal_policy_inferer** runs policy inference on the observation vector and publishes raw model outputs.
- **pal_policy_postprocessor** converts raw model outputs into controller-compatible commands and sends them to
the robot.

Each package provides a **base class** and one or more **child classes**.  
The base classes are fully generic and reusable: they define the standard data flow (as described above) and
expose the required interfaces.

The child classes implement model- or policy-specific behaviour:
- **pal_policy_inferer child classes** implement concrete model backends (e.g. ONNX runtime). These are
independent of the robot and policy and can be reused across different deployments.
- **pal_policy_preprocessor and pal_policy_postprocessor child classes** are policy- and robot-specific.
They acquire the required observations (e.g. specific joint states), construct the input vector in the expected
order, and convert model outputs into commands compatible with the appropriate robot controllers.

_(Note: currently only the reaching policy for TIAGo Pro, trained using reinforcement learning and exported to ONNX, is implemented.)_

### Configuring a policy
1. **Add trained model** 

Place the model (e.g. ONNX file) you want to deploy in ```pal_policy_deployer/models/``` and define the model path in the corresponding launch file.

2. **Define model metadata**

To enhance reusability and avoid hardcoding policy-specific values in the launch
file, model metadata is defined and extracted at launch time.

Each metadata entry is parsed into separate key–value pairs (strings or lists of
strings) and passed to the nodes as ROS 2 parameters. Typical metadata includes
observation order, included joints, and input/output structure.

These parameters must be retrieved (```this->declare_parameter```) to be used inside the
corresponding child classes.

3. **Configure YAMLs**

Additional policy- or robot-specific information that is not part of the model
metadata is configured via YAML files, mainly for the preprocessor and
postprocessor. Examples include joint limits and excluded joints.

The YAML entries are loaded as ROS 2 parameters and can also be accessed within the child classes.

4. **Implement child classes**

Implement the required child classes for the policy. Existing implementations can be used as reference. 

5. **Launch**

After building and sourcing the workspace, launch the policy, for example: ```ros2 launch pal_policy_deployer reaching_tiago_deployer.launch.py```

### Kangaroo (mjlab) impedance gains

`kang_mjlab_node` writes the active policy's ONNX metadata `joint_stiffness` / `joint_damping`
(ordered like the action joints) to `/rl_impedance_controller` as `gains.<joint>.p` /
`gains.<joint>.d`. This happens at startup, on every `~/switch_policy` request, and when an
imitation policy auto-switches back to policy `[0]`. If the controller is not up yet, the request
is retried every 0.5 s and only the latest policy's gains are sent. Only gains whose value
actually changes are logged, as `gains.<joint>.p: <old> -> <new>`, followed by a
`<n> changed, <n> unchanged, <n> failed` summary. If the metadata has no gains, or the wrong number of them, an
error is logged and the controller gains are left as they are.

### TO DO
- Add locomotion (Kangaroo) policies
- Add different model types 
- Implement the use of references for commands/targets, instead of the interactive marker node. This would be scalable using other references

