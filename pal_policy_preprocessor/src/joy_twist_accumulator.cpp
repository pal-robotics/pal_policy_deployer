#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>

struct Limits {
  double min_x = -0.3, max_x = 1.0;
  double min_y = -0.3, max_y = 0.3;
  double min_wz = -0.5, max_wz = 0.5;
};

class JoyTwistAccumulator {
public:
  JoyTwistAccumulator(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {
    // Params (indices follow typical Xbox/Logitech mappings; adjust if needed)
    pnh_.param<std::string>("joy_topic", joy_topic_, "/joy");
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    pnh_.param<int>("axis_x_index", axis_x_idx_, 5);     // axis 5 -> linear.x
    pnh_.param<int>("axis_y_index", axis_y_idx_, 4);     // axis 4 -> linear.y
    pnh_.param<int>("btn_wz_dec_index", btn_wz_dec_idx_, 0); // button 0 -> -0.1
    pnh_.param<int>("btn_wz_inc_index", btn_wz_inc_idx_, 2); // button 2 -> +0.1
    pnh_.param<double>("step_linear", step_lin_, 0.1);
    pnh_.param<double>("step_angular", step_wz_, 0.1);
    pnh_.param<double>("edge_threshold", edge_thresh_, 0.9); // consider ±1 when |axis|>=0.9

    // Limits (defaults already set in struct)
    pnh_.param("min_x", limits_.min_x, limits_.min_x);
    pnh_.param("max_x", limits_.max_x, limits_.max_x);
    pnh_.param("min_y", limits_.min_y, limits_.min_y);
    pnh_.param("max_y", limits_.max_y, limits_.max_y);
    pnh_.param("min_wz", limits_.min_wz, limits_.min_wz);
    pnh_.param("max_wz", limits_.max_wz, limits_.max_wz);

    pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    sub_ = nh_.subscribe(joy_topic_, 10, &JoyTwistAccumulator::joyCB, this);

    // Initial values are zero by default.
    twist_.linear.x = 0.0;
    twist_.linear.y = 0.0;
    twist_.angular.z = 0.0;

    prev_axis_x_sign_ = 0;
    prev_axis_y_sign_ = 0;
    prev_btn_wz_dec_ = false;
    prev_btn_wz_inc_ = false;

    ROS_INFO_STREAM("[joy_twist_accumulator] Subscribing to " << joy_topic_
                    << ", publishing Twist to " << cmd_vel_topic_);
  }

private:
  static int axisToSign(double val, double thresh) {
    if (val >= thresh) return +1;
    if (val <= -thresh) return -1;
    return 0;
  }

  static double clamp(double v, double vmin, double vmax) {
    return std::max(vmin, std::min(v, vmax));
  }

  void joyCB(const sensor_msgs::Joy::ConstPtr& msg) {
    // Defensive checks for array sizes
    if (axis_x_idx_ < 0 || axis_x_idx_ >= static_cast<int>(msg->axes.size()) ||
        axis_y_idx_ < 0 || axis_y_idx_ >= static_cast<int>(msg->axes.size()) ||
        btn_wz_dec_idx_ < 0 || btn_wz_dec_idx_ >= static_cast<int>(msg->buttons.size()) ||
        btn_wz_inc_idx_ < 0 || btn_wz_inc_idx_ >= static_cast<int>(msg->buttons.size())) {
      ROS_WARN_THROTTLE(2.0, "Joy message doesn't match expected axis/button indices.");
      return;
    }

    // --- linear.x via axis 5 ---
    int axis_x_sign = axisToSign(msg->axes[axis_x_idx_], edge_thresh_);
    if (axis_x_sign != 0 && axis_x_sign != prev_axis_x_sign_) {
      twist_.linear.x += axis_x_sign * step_lin_;
      twist_.linear.x = clamp(twist_.linear.x, limits_.min_x, limits_.max_x);
    }
    prev_axis_x_sign_ = axis_x_sign;

    // --- linear.y via axis 4 ---
    int axis_y_sign = axisToSign(msg->axes[axis_y_idx_], edge_thresh_);
    if (axis_y_sign != 0 && axis_y_sign != prev_axis_y_sign_) {
      twist_.linear.y += axis_y_sign * step_lin_;
      twist_.linear.y = clamp(twist_.linear.y, limits_.min_y, limits_.max_y);
    }
    prev_axis_y_sign_ = axis_y_sign;

    // --- angular.z via buttons ---
    bool btn_dec = (msg->buttons[btn_wz_dec_idx_] != 0);
    bool btn_inc = (msg->buttons[btn_wz_inc_idx_] != 0);

    // Rising edges only
    if (btn_dec && !prev_btn_wz_dec_) {
      twist_.angular.z -= step_wz_;
      twist_.angular.z = clamp(twist_.angular.z, limits_.min_wz, limits_.max_wz);
    }
    if (btn_inc && !prev_btn_wz_inc_) {
      twist_.angular.z += step_wz_;
      twist_.angular.z = clamp(twist_.angular.z, limits_.min_wz, limits_.max_wz);
    }
    prev_btn_wz_dec_ = btn_dec;
    prev_btn_wz_inc_ = btn_inc;

    pub_.publish(twist_);

    ROS_INFO_STREAM_THROTTLE(0.5,
      "Velocity command: Vx: " << twist_.linear.x
      << ", Vy: " << twist_.linear.y
      << ", wz: " << twist_.angular.z);
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string joy_topic_, cmd_vel_topic_;
  int axis_x_idx_, axis_y_idx_;
  int btn_wz_dec_idx_, btn_wz_inc_idx_;
  double step_lin_, step_wz_, edge_thresh_;
  Limits limits_;
  geometry_msgs::Twist twist_;

  // Edge detection state
  int prev_axis_x_sign_;
  int prev_axis_y_sign_;
  bool prev_btn_wz_dec_;
  bool prev_btn_wz_inc_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_twist_accumulator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  JoyTwistAccumulator node(nh, pnh);
  ros::spin();
  return 0;
}
