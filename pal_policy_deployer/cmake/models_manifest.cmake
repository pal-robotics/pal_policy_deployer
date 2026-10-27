# Policy models shipped with pal_policy_deployer that are hosted in the GitLab Model Registry.
# One line per file: pal_policy_model_file(<version_id> <relpath below models/> <sha256>)
# Files that exist locally in models/ are used as is. See the README "Model registry" section
# for how to push a model and generate these lines.
#
# Example:
# pal_policy_model_file(42 lower_body/loco/loco_v315_mjlab/env.yaml 3b1f...c9)
pal_policy_model_file(3 lower_body/loco/loco_v315_mjlab/2026-07-05_07-57-09_lowerbody_flat_locomotion_new_model_action_rate_0_2_with_better_imu_and_bad_imu_accel.onnx 611ebf3e6362b41a04c67782992c5e087256d6e4459953a054f71c3f7a55a5b3)
pal_policy_model_file(3 lower_body/loco/loco_v315_mjlab/agent.yaml 9d29b93dece252b4d97fba5ff6429c528df7b459977676322f5048c30d55a22e)
pal_policy_model_file(3 lower_body/loco/loco_v315_mjlab/env.yaml 566adf90c52b8da9cb5876987402b04f6507112d451978d6ae4c01768f53807f)
