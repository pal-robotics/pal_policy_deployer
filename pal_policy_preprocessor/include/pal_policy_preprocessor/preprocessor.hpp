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
  void buildMetadataMap(const std::string & onnx_model_path);
  const std::map<std::string, std::string> & metadataMap() const {return metadata_map_;}
  void run(std::chrono::milliseconds period = std::chrono::milliseconds(20));
  virtual void buildInputVector(std::vector<double> * obs) const = 0;


  // TO DO: move to utils?
  static std::vector<std::string> splitString(const std::string & str, char delimiter);
  static std::vector<double> parseDoubles(const std::string & str);
  static bool isExcludedJoint(
    const std::string & joint_name,
    const std::vector<std::string> & excluded);

  Ort::Env env_;

  std::vector<double> obs_;

private:
  void publishInput(const std::vector<double> & obs);
  void onTimer();

  std::map<std::string, std::string> metadata_map_;
  std::string model_path_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_obs_;
  rclcpp::TimerBase::SharedPtr timer_;
};
