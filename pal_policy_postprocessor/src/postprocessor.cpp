// pub_q_desired_right_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
//   "pal_impedance_control_right/q_desired", 1);
// pub_q_desired_left_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
//   "pal_impedance_control_left/q_desired", 1);
// pub_last_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
//   "/ik_policy/last_action", 1);

// sub_obs_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
//   "/ik_policy/obs", 1,
//   std::bind(&TiagoPolicyInfererIK::obsCallback, this, std::placeholders::_1));

// constexpr float action_scale = 0.5f;

// // === ACTION LAYOUT (network) ===
// // y_f[0]      -> torso
// // y_f[1..7]   -> left arm  (7 joints)
// // y_f[8..14]  -> right arm (7 joints)

// // ---- Default positions (baseline) ----
// const double torso_default = 0.1;
// const std::vector<double> left_default = {0.358, -1.827, 0.470, -2.341, 0.000, -1.201, 0.000};
// const std::vector<double> right_default = {-0.358, -1.827, -0.470, -2.341, 0.000, -1.201, 0.000};

// // Torso command (not published in this file, but kept for completeness/debug)
// double torso_cmd = torso_default + action_scale * static_cast<double>(y_f[0]);
// (void)torso_cmd;

// std::vector<double> left_cmd(7);
// std::vector<double> right_cmd(7);

// // Left arm: y_f[1..7] as DELTA around default
// for (size_t i = 0; i < 7; ++i) {
//   left_cmd[i] = left_default[i] + action_scale * static_cast<double>(y_f[1 + i]);
// }

// // Right arm: y_f[8..14] as DELTA around default
// for (size_t i = 0; i < 7; ++i) {
//   right_cmd[i] = right_default[i] + action_scale * static_cast<double>(y_f[8 + i]);
// }

// // Joint limits
// const std::vector<double> min_val = {
//   -4.71239, -2.44346, -2.61799, -2.44346, -1.5708, -1.88496, -2.61799,
// };

// const std::vector<double> max_val = {
//   0.523599, 1.13446, 2.61799, 1.13446, 3.66519, 3.00197, 2.61799,
// };

// std::vector<double> clamped_left(7);
// std::vector<double> clamped_right(7);

// for (size_t i = 0; i < 7; ++i) {
//   clamped_left[i] = clamp(left_cmd[i], min_val[i], max_val[i]);
//   clamped_right[i] = clamp(right_cmd[i], min_val[i], max_val[i]);
// }

// // Publish desired joint positions for both arms
// std_msgs::msg::Float64MultiArray qmsg_left;
// qmsg_left.data = clamped_left;     // publish clamped absolute targets
// pub_q_desired_left_->publish(qmsg_left);

// std_msgs::msg::Float64MultiArray qmsg_right;
// qmsg_right.data = clamped_right;   // publish clamped absolute targets
// pub_q_desired_right_->publish(qmsg_right);

// // === Publish last actions: torso (1) + left (7) + right (7) = 15 ===
// // last.data layout:
// //   [0]      = torso delta (scaled)
// //   [1..7]   = left arm deltas (scaled)
// //   [8..14]  = right arm deltas (scaled)
// std_msgs::msg::Float64MultiArray last;
// last.data.resize(15);

// // Torso delta (scaled)
// last.data[0] = static_cast<double>(y_f[0]);

// // Left deltas (scaled)
// for (size_t i = 0; i < 7; ++i) {
//   last.data[1 + i] = static_cast<double>(y_f[1 + i]);
// }

// // Right deltas (scaled)
// for (size_t i = 0; i < 7; ++i) {
//   last.data[8 + i] = static_cast<double>(y_f[8 + i]);
// }
