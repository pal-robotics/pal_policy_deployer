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

For the Kang mjlab launch files, models live in ```models/<robot_type>/<task>/<task>_<model>_mjlab/```. The launch file resolves this directory from its ```task```/```model``` arguments and passes it to ```kang_mjlab_node``` as ```<policy>.model_path``` (e.g. ```loco.model_path```). The node loads the first ```.onnx``` file in that directory.

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

### Model registry

The shipped policies can be hosted in the [GitLab Model Registry](https://docs.gitlab.com/user/project/ml/model_registry/)
of [control/pal_policy_deployer](https://gitlab.pal-robotics.com/control/pal_policy_deployer/-/ml/models)
instead of git. At CMake configure time, every file listed in the manifest
`pal_policy_deployer/models.yaml` is made available under `share/pal_policy_deployer/models/<relpath>`:

- if `pal_policy_deployer/models/<relpath>` exists locally, it is used and **nothing is downloaded**;
- otherwise it is downloaded from the registry into the build directory, checked against its SHA256, and installed.
  A verified copy is kept, so re-configuring does not download it again.

The install layout is the same either way, so the launch files do not change.

#### One-time project setup

In **Settings > General > Visibility, project features, permissions**, enable **Model registry** and **Package registry**.

#### Staging and stable models

Models are organised by stage, one registry model per robot type and task:

```text
staging/lower_body_loco   315.0.0, 412.0.0, ...   candidates under test, delete freely
stable/lower_body_loco    315.0.0, ...            shipped policies, the only ones in the manifest
```

- The model name is `<stage>/<body>_<task>`. The registry is a flat list of models, so the `/` is part of the name;
  searching for `staging/` in **Deploy > Model registry** lists every staging model.
- The version is the model directory's number as SemVer (`loco_v315_mjlab` → `315.0.0`). GitLab only accepts SemVer,
  so `v315` itself cannot be a version.
- Files are stored flat in a version, under their file name. The manifest maps them back to their path under `models/`.

The commands below use these variables (`GITLAB_TOKEN` needs the `api` scope to push):
```bash
API=https://gitlab.pal-robotics.com/api/v4/projects/control%2Fpal_policy_deployer
MLFLOW=$API/ml/mlflow/api/2.0/mlflow
AUTH="Authorization: Bearer $GITLAB_TOKEN"   # the registry endpoints reject PRIVATE-TOKEN
```

**1. Push a candidate to staging.** Either use the UI (**Deploy > Model registry > Create model**, **Create new
version**, upload files on the **Artifacts** tab) or:
```bash
NAME=staging/lower_body_loco; VERSION=315.0.0; DIR=pal_policy_deployer/models/lower_body/loco/loco_v315_mjlab
curl -sS -H "$AUTH" -H "Content-Type: application/json" -X POST $MLFLOW/registered-models/create \
  -d "{\"name\":\"$NAME\"}"     # only the first time; errors if the model exists
curl -sS -H "$AUTH" -H "Content-Type: application/json" -X POST $MLFLOW/model-versions/create \
  -d "{\"name\":\"$NAME\",\"source\":\"pal_policy_deployer\",\"tags\":[{\"key\":\"gitlab.version\",\"value\":\"$VERSION\"}]}"
# The response's "source" is .../-/ml/models/<model_id>/versions/<version_id>
VERSION_ID=<version_id>
for f in $DIR/*; do
  curl -sS --fail -H "$AUTH" --upload-file "$f" "$API/packages/ml_models/$VERSION_ID/files/$(basename "$f")"
done
```
To test a staging model, put its files in `pal_policy_deployer/models/<relpath>` (download them from the UI, or with
`curl -H "$AUTH" -o <file> $API/packages/ml_models/<version_id>/files/<file>`) and build. A local file always wins.

**2. Promote to stable.** Repeat step 1 with `NAME=stable/lower_body_loco`, the same version and the same files.
This gives a new `<version_id>`. Then add an entry to the manifest `pal_policy_deployer/models.yaml`:
```yaml
lower_body/loco/loco_v315_mjlab:     # model directory below models/
  model: stable/lower_body_loco      # informative only
  version: 315.0.0                   # informative only
  version_id: 3                      # used to download the files
  files:                             # file name: sha256
    2026-07-05_07-57-09_lowerbody_flat_locomotion_..._bad_imu_accel.onnx: 611ebf3e...
    agent.yaml: 9d29b93d...
    env.yaml: 566adf90...
```
To generate the entry from the local files, from the repository root:
```bash
NAME=stable/lower_body_loco; VERSION=315.0.0; VERSION_ID=<stable version_id>
DIR=lower_body/loco/loco_v315_mjlab
(cd pal_policy_deployer/models && \
  printf '\n%s:\n  model: %s\n  version: %s\n  version_id: %s\n  files:\n' $DIR $NAME $VERSION $VERSION_ID && \
  sha256sum $DIR/* | awk '{n=$2; sub(".*/","",n); print "    " n ": " $1}') \
  >> pal_policy_deployer/models.yaml
```
Commit the manifest. Committing the model files to git is optional. Only `stable/` version ids belong in the manifest.

**3. Clean up.** Delete `staging/` models or versions from **Deploy > Model registry** whenever you like: builds
never use them. Delete a `stable/` version only after removing its entry from the manifest.

#### Building

Reading the manifest needs `python3` with PyYAML (`python3-yaml`, declared as a build dependency).
Fetching needs `GITLAB_TOKEN`: a personal, project or group access token with the `read_api` scope
(sent as `Authorization: Bearer`).

```bash
export GITLAB_TOKEN=<token>
colcon build --packages-select pal_policy_deployer
```

If a listed file is missing locally and `GITLAB_TOKEN` is not set, or the download or checksum fails, configure
stops with an error naming the file. For the debian build, set `GITLAB_TOKEN` as a masked CI/CD variable of the
job that runs the package build. To use another registry project, pass
`-DPAL_MODEL_REGISTRY_URL=https://<host>/api/v4/projects/<url-encoded project path>`.

### TO DO
- Add locomotion (Kangaroo) policies
- Add different model types 
- Implement the use of references for commands/targets, instead of the interactive marker node. This would be scalable using other references

