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
    std::vector<double> values;  // the actual setpoint
  };

  std::deque<Sample> buffer_;

  double last_seg_start_time_{0.0};
  double last_seg_end_time_{0.0};
  int last_seg_step_{0};
  static constexpr int UPSAMPLE_FACTOR = 20;

  std::mutex mutex_;
  std::vector<std::unique_ptr<boost::math::interpolators::pchip<std::vector<double>>>>
  current_pchip_;


  void onNewSample(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void publishInterpolated();

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_inferer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_interpolator_;
  rclcpp::TimerBase::SharedPtr timer_;

};

#endif  // PAL_POLICY_INFERER__HERMITE_INTERPOLATOR_HPP_
