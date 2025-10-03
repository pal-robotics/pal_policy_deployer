# pal_policy_deployer

**WIP:** Currently, only 3 policies are available, and only for Kang.

To run:

```bash
roslaunch pal_policy_deployer kang_rl_deployer.launch policy:=(loco|reach|balance)
```

- **pal_policy_deployer**: Package containing the models and launch files. Installing it will also install the two other packages.
- **pal_policy_inferer**: Package containing the nodes for running policy inference and sending position references.
- **pal_policy_preprocessor**: Package containing the nodes for generating observation inputs and publishing them to a topic.

### Available RL Nodes

- Kang Reach
- Kang Balance
- (WIP) Kang Loco → validated only in sim2sim

### TODO

P0
- Retrain balance to have the correct offset
- Find a way to make obs size dynamic 
- Make balance and loco inferer node the same

P1
- Retrain reach, balance and loco to use the arms as well
- Clean reach inferer and preprocessor node
- Make reach inferer node the same as the other as well
- Find a way to make the preprocessor be the same 
