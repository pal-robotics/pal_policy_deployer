#ifndef PAL_POLICY_INFERER__HERMITE_INTERPOLATOR_HPP_
#define PAL_POLICY_INFERER__HERMITE_INTERPOLATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <math.h>
#include <boost/math/interpolators/pchip.hpp>

#include <deque>
#include <mutex>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

using boost::math::interpolators::pchip;


class HermiteInterpolator : public rclcpp::Node
{
public:
  HermiteInterpolator();
  ~HermiteInterpolator() override;

private:
  struct Sample
  {
    double time;   // when we received it
    double value;  // the actual setpoint
  };

  std::deque<Sample> buffer_;
  std::mutex mutex_;
  std::unique_ptr<boost::math::interpolators::pchip<std::vector<double>>> current_pchip_;

  double t_min_{0.0};
  double t_max_{0.0};

  void onNewSample(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publishInterpolated();

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_inferer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_interpolator_;
  rclcpp::TimerBase::SharedPtr timer_;

};

#endif  // PAL_POLICY_INFERER__HERMITE_INTERPOLATOR_HPP_
