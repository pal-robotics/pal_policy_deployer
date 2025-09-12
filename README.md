# pal_policy_deployer

v0.0.0

RL nodes for

- kang reach
- kang_balance
- (wip) kang_loco

TODO

- ~~Add interactive marker node.~~
- Create three launch files for reach, loco, and balance (sim2sim).
- Add an option to run sim2real in each.
- Then merge them into a single, parameterized launch file.
- Split the current inference node into two nodes: inference and post-processor.
- Improve how joint states are fed into the observation vector.
- Create a base Preprocessor class and derive others from it.
- Standardize on one inference node (not three per use case).
- Modularize the post-processor.
