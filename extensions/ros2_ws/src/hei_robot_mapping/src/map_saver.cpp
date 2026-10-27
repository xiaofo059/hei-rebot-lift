#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rtabmap_msgs/srv/get_map.hpp>
#include "hei_robot_mapping/map_snapshot.hpp"

using namespace std::chrono_literals;

template<typename Future>
auto receive(
  const rclcpp::Node::SharedPtr & node, Future & future,
  std::chrono::nanoseconds timeout, const std::string & operation)
{
  if (rclcpp::spin_until_future_complete(node, future, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error(operation + " timed out or was interrupted; no map was saved");
  }
  return future.get();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_saver");
  int code = 0;
  try {
    const auto name = node->declare_parameter<std::string>("map_name", "");
    const auto directory = node->declare_parameter<std::string>("output_dir",
        "/home/user/hei-rebot-lift/extensions/ros2_ws/src/hei_robot_mapping/map");
    const auto remote_node = node->declare_parameter<std::string>("rtabmap_node", "/rtabmap/rtabmap");
    const auto service = node->declare_parameter<std::string>("map_service", remote_node + "/get_map_data");
    const auto wait_seconds = node->declare_parameter<double>("wait_timeout_sec", 10.0);
    const auto request_seconds = node->declare_parameter<double>("request_timeout_sec", 120.0);
    if (!std::isfinite(wait_seconds) || !std::isfinite(request_seconds) ||
      wait_seconds <= 0 || request_seconds <= 0)
    {
      throw std::invalid_argument("Timeout parameters must be finite and positive");
    }
    const auto destination = hei_robot_mapping::snapshot_path(directory, name);
    const auto wait = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(wait_seconds));
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(request_seconds));
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node, remote_node);
    auto client = node->create_client<rtabmap_msgs::srv::GetMap>(service);
    RCLCPP_INFO(node->get_logger(), "Waiting for mapper %s and %s", remote_node.c_str(), service.c_str());
    if (!parameters_client->wait_for_service(wait) || !client->wait_for_service(wait)) {
      throw std::runtime_error("Mapper services unavailable. Start mapping first and use the same ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY");
    }
    auto list_future = parameters_client->list_parameters({}, 0);
    const auto listed = receive(node, list_future, timeout, "Listing mapper parameters");
    const auto supported = rtabmap::Parameters::getDefaultParameters();
    std::vector<std::string> names;
    for (const auto & item : listed.names) {if (supported.count(item)) {names.push_back(item);}}
    if (names.empty()) {throw std::runtime_error("Selected node has no RTAB-Map parameters");}
    auto params_future = parameters_client->get_parameters(names);
    rtabmap::ParametersMap parameters;
    for (const auto & parameter : receive(node, params_future, timeout, "Reading mapper parameters")) {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        throw std::runtime_error("Unexpected RTAB-Map parameter type: " + parameter.get_name());
      }
      parameters[parameter.get_name()] = parameter.as_string();
    }
    if (!parameters.count("Reg/Strategy") || parameters.at("Reg/Strategy") != "1") {
      throw std::runtime_error("This saver is for the current laser-only mapper (Reg/Strategy=1)");
    }
    auto request = std::make_shared<rtabmap_msgs::srv::GetMap::Request>();
    request->global_map = true;
    request->optimized = true;
    request->graph_only = false;
    RCLCPP_INFO(node->get_logger(), "Reading map snapshot; keep mapping running until save completes");
    auto future = client->async_send_request(request);
    const auto response = receive(node, future, timeout, "Reading map snapshot");
    const auto result = hei_robot_mapping::save_snapshot(response->data, parameters, destination);
    RCLCPP_INFO(node->get_logger(), "Saved %s (%zu nodes, %zu scans, %zu directed constraints). Mapping remains running.",
      result.path.c_str(), result.nodes, result.scans, result.directed_links);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Map save failed: %s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
