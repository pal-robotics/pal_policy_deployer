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

- A model directory `models/<robot_type>/<task>/<task>_v<N>_mjlab` maps to the registry model
  `<stage>/<robot_type>_<task>`, version `<N>.0.0` (GitLab only accepts SemVer versions). Task names must not contain `_`.
- The registry is a flat list of models, so the `/` is part of the name; searching for `staging/` in
  **Deploy > Model registry** lists every staging model.
- Files are stored flat in a version, under their file name.

#### handle_models.py

`pal_policy_deployer/scripts/handle_models.py` pushes models to and pulls them from the registry
(`scripts/handle_models.py --help` shows the same reference).

- `GITLAB_TOKEN` must be set: `api` scope to push, `read_api` to pull. The script needs `python3` with PyYAML.
- Exactly one of `--push` / `--pull` is required.
- It works on `staging/` by default, and on `stable/` with `--stable`. `--stable` combines with every command below.

**Naming a model.** A model always refers to a directory in the package's `models/` directory, whatever the current
directory. It can be given as:

| Form | Example |
|---|---|
| `<task>_v<N>_mjlab` plus `--robot-type` | `loco_v534_mjlab --robot-type lower_body` |
| `<robot_type>/<task>/<task>_v<N>_mjlab` | `lower_body/loco/loco_v534_mjlab` |
| any path to that directory | `pal_policy_deployer/models/lower_body/loco/loco_v534_mjlab` |

If `--robot-type` is given together with a path, the two must agree. Directories that do not follow this naming
(e.g. `tiagopro`) cannot be handled by the script.

**Commands** (run from `pal_policy_deployer/`; add `--stable` to any of them to use `stable/` instead of `staging/`):

| Command | What it does |
|---|---|
| `scripts/handle_models.py --push <model>` | Push one model. Creates the registry model and version if needed and uploads every file. Skipped if the version already exists upstream. |
| `scripts/handle_models.py --push <model> --override` | Like `--push`, but if the version exists, re-uploads the files whose SHA256 differs from upstream. Does nothing if all files are identical. |
| `scripts/handle_models.py --push --robot-type <T>` | Push every local model of robot type `<T>`, skipping versions that exist upstream. Directories that do not follow the naming are listed as ignored. |
| `scripts/handle_models.py --push --robot-type <T> --override` | Same for every local model of `<T>`, re-uploading changed files of existing versions. |
| `scripts/handle_models.py --pull <model>` | Download one version into its model directory. Files that already match locally are skipped. Stops before downloading anything if a local file differs from upstream. |
| `scripts/handle_models.py --pull --all --robot-type <T>` | Pull every upstream version of `<T>` whose model directory does not exist locally. |

For example:
```bash
scripts/handle_models.py --push loco_v534_mjlab --robot-type lower_body            # → staging/lower_body_loco 534.0.0
scripts/handle_models.py --push lower_body/loco/loco_v534_mjlab --stable           # → stable/lower_body_loco 534.0.0
scripts/handle_models.py --pull --all --robot-type full_body_4dof --stable         # ← every missing stable full_body_4dof model
```

**Details:**
- `--push --stable` updates `pal_policy_deployer/models.yaml` automatically, adding or replacing the model's entry:
  | Upstream `stable/` version | `models.yaml` |
  |---|---|
  | new: created and uploaded | entry added |
  | exists, `--override` re-uploaded files, or all files already identical | entry added or replaced with the new checksums |
  | exists and identical to the local files (without `--override`) | entry added if missing, nothing uploaded |
  | exists with different files (without `--override`) | left alone, with a warning |

  The file is rewritten sorted by directory. The comment block at the top is kept; comments inside entries are not.
  Re-running `--push --robot-type <T> --stable` therefore rebuilds the entries of all of `<T>`'s stable models without
  uploading anything. Staging pushes and pulls never touch `models.yaml`. To drop a model from the build, delete its
  entry by hand. Review the change with `git diff` before committing it.
- Every downloaded file is checked against the SHA256 the registry reports. A mismatch stops the pull and the file is
  not saved.
- `--override` never deletes anything upstream. GitLab keeps the previous copy of each re-uploaded file and serves the
  newest; old copies can be removed under **Deploy > Package registry**. A file that exists upstream but not locally
  is left in place, with a warning.
- `--pull --all` skips a model if its directory exists locally, even if it is incomplete; complete it with
  `--pull <model>`. Upstream versions that are not `<N>.0.0` are skipped, since they map to no `_v<N>_` directory.
  `--robot-type full_body` does not pick up `full_body_4dof` models.
- If a registry error stops `--push --robot-type <T>`, the models pushed before it stay pushed. Running the same
  command again skips them and continues with the rest.

**Invalid combinations**, rejected before anything is sent to the registry:

| Command | Error |
|---|---|
| neither `--push` nor `--pull`, or both | one of them is required |
| `--push` without a model and without `--robot-type` | `--push needs a model or --robot-type` |
| `--pull` without a model and without `--all`, or with both | `--pull needs either a model or --all` |
| `--all` without `--robot-type` | `--all needs --robot-type` |
| `--all` with `--push` | `--all only works with --pull` |
| `--override` with `--pull` | `--override only works with --push` |
| `<task>_v<N>_mjlab` without `--robot-type` | `pass --robot-type` |
| a path whose robot type differs from `--robot-type` | `... is a lower_body model, but --robot-type is full_body_4dof` |

The script also exits with an error when `GITLAB_TOKEN` is missing, when the version is not found upstream (`--pull`),
and on any registry error.

#### Workflow

1. **Push a candidate to staging:** `scripts/handle_models.py --push loco_v534_mjlab --robot-type lower_body`.
   Others can test it with `--pull`: a local model directory always wins over the manifest in the build.
2. **Promote it to stable** once it is good enough:
   ```bash
   scripts/handle_models.py --pull loco_v534_mjlab --robot-type lower_body            # if not local yet
   scripts/handle_models.py --push loco_v534_mjlab --robot-type lower_body --stable   # also updates models.yaml
   ```
   Commit the updated `pal_policy_deployer/models.yaml`. Committing the model files to git is optional. Only `stable/`
   versions belong in the manifest. An entry looks like this:
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
3. **Clean up:** delete `staging/` models or versions from **Deploy > Model registry** whenever you like: builds never
   use them. Delete a `stable/` version only after removing its entry from the manifest. Overriding a `stable/` version
   (`--push --stable --override`) changes its checksums; the script updates the entry, so commit `models.yaml` with it.

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

