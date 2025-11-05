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
  buffer_.push_back({now, msg->data[0]});
  if (buffer_.size() > 10) {
    buffer_.pop_front();
  }

  {
    std::ostringstream oss;
    oss << "Buffer (" << buffer_.size() << "): ";
    std::size_t idx = 0;
    for (const auto & s : buffer_) {
      oss << "[" << idx++ << "] t=" << s.time << " v=" << s.value << "  ";
    }
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  if (buffer_.size() < 5) {
    return;
  }

  std::vector<double> times;
  std::vector<double> values;
  times.reserve(buffer_.size());
  values.reserve(buffer_.size());

  for (const auto & s : buffer_) {
    times.push_back(s.time);
    values.push_back(s.value);
  }

  // (re)build PCHIP interpolator
  auto interp = pchip(std::move(times), std::move(values));
  current_pchip_ = std::make_unique<boost::math::interpolators::pchip<std::vector<double>>>(
    std::move(interp));


  t_min_ = buffer_.front().time;
  t_max_ = buffer_.back().time;
}

void HermiteInterpolator::publishInterpolated()
{
  std::lock_guard<std::mutex> lock(mutex_);
  double now = this->get_clock()->now().seconds();

  if (!current_pchip_) {
    return;
  }

  if (now < t_min_ || now > t_max_) {
    return;
  }

  double interpolated_value = (*current_pchip_)(now);

  std_msgs::msg::Float64MultiArray output_msg;
  output_msg.data.push_back(interpolated_value);
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
