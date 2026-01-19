#pragma once

#include <chrono>
#include <map>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class Preprocessor : public rclcpp::Node
{
public:
  explicit Preprocessor(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~Preprocessor() = default;

protected:
  std::string resolveModelPath();
  void buildMetadataMap(const std::string & onnx_model_path);
  const std::map<std::string, std::string> & metadataMap() const {return metadata_map_;}

  void run(std::chrono::milliseconds period = std::chrono::milliseconds(20));
  virtual std::vector<double> buildObservationVector() const = 0;
  void publishObservation(const std::vector<double> & obs);

  static std::vector<std::string> splitString(const std::string & str, char delimiter);
  static std::vector<double> parseDoubles(const std::string & str);
  static bool isExcludedJoint(
    const std::string & joint_name,
    const std::vector<std::string> & excluded);

  Ort::Env env_;

private:
  void onTimer();

  std::map<std::string, std::string> metadata_map_;
  std::string model_path_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_obs_;
  rclcpp::TimerBase::SharedPtr timer_;
};
