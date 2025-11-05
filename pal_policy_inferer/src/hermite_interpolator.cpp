#include "pal_policy_inferer/hermite_interpolator.hpp"


HermiteInterpolator::HermiteInterpolator()
: rclcpp::Node("hermite_interpolator")
{
  // Subscriber to the inferer output
  sub_inferer_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/q_des_raw", 10,
    std::bind(&HermiteInterpolator::onNewSample, this, std::placeholders::_1));
  // Publisher for the interpolated output
  pub_interpolator_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "pal_impedance_control_right/q_desired", 1);

  timer_ = create_wall_timer(
    1ms, std::bind(&HermiteInterpolator::publishInterpolated, this));

  RCLCPP_INFO(get_logger(), "DEBUG1: HermiteInterpolator node started");

}


void HermiteInterpolator::onNewSample(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // Perform interpolation and publish results
  std::lock_guard<std::mutex> lock(mutex_);
  double now = this->get_clock()->now().seconds();

  // Keep a rolling buffer of ~10 samples (~0.2 s at 50 Hz)
  std::vector<double> values(msg->data.begin(), msg->data.end());
  buffer_.push_back({now, std::move(values)});
  if (buffer_.size() > 10) {
    buffer_.pop_front();
  }

  if (buffer_.size() < 5) {
    return;
  }

  std::vector<double> times;
  times.reserve(buffer_.size());

  for (const auto & s : buffer_) {
    times.push_back(s.time);
  }

  // (re)build PCHIP interpolator
  std::vector<std::unique_ptr<boost::math::interpolators::pchip<std::vector<double>>>> interps;
  interps.reserve(buffer_.front().values.size());

  for (std::size_t j = 0; j < buffer_.front().values.size(); ++j) {
    // collect dimension j values
    std::vector<double> values_j;
    values_j.reserve(buffer_.size());
    for (const auto & s : buffer_) {
      values_j.push_back(s.values[j]);
    }


    std::vector<double> times_j = times;

    auto interp_j = boost::math::interpolators::pchip(std::move(times_j), std::move(values_j));
    interps.push_back(
      std::make_unique<boost::math::interpolators::pchip<std::vector<double>>>(
        std::move(interp_j)));
  }

  current_pchip_ = std::move(interps);
  t_min_ = buffer_.front().time;
  t_max_ = buffer_.back().time;
}

void HermiteInterpolator::publishInterpolated()
{
  std::lock_guard<std::mutex> lock(mutex_);
  double now = this->get_clock()->now().seconds();

  if (current_pchip_.empty()) {
    return;
  }

  if (now < t_min_ || now > t_max_) {
    return;
  }

  std_msgs::msg::Float64MultiArray output_msg;
  output_msg.data.reserve(current_pchip_.size());

  for (auto & interp : current_pchip_) {
    output_msg.data.push_back((*interp)(now));
  }

  pub_interpolator_->publish(output_msg);

}

HermiteInterpolator::~HermiteInterpolator()
{

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HermiteInterpolator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
