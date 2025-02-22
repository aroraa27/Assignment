#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>

class TrajectoryReaderPublisher : public rclcpp::Node
{
public:
  TrajectoryReaderPublisher() 
  : Node("trajectory_reader_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Publish static transform between "map" and "odom" directly in this node
    publishStaticTransform();

    // Declare and get the parameter for the CSV file.
    this->declare_parameter<std::string>("trajectory_file", "trajectory.csv");
    this->get_parameter("trajectory_file", trajectory_file_);

    // Read the trajectory file.
    if (!readTrajectoryFile(trajectory_file_)) {
      RCLCPP_ERROR(this->get_logger(), "Unable to read trajectory file: %s", trajectory_file_.c_str());
    }

    // Create publisher for the MarkerArray.
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("read_trajectory_markers", 10);

    // Set up a timer to publish the markers every second.
    marker_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TrajectoryReaderPublisher::publishMarkers, this));

    RCLCPP_INFO(this->get_logger(), "Trajectory Reader Node started; reading file: %s", trajectory_file_.c_str());
  }

private:
  // Function to publish a static transform between "map" and "odom"
  void publishStaticTransform()
  {
    // Create a static transform broadcaster.
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = "map";  // Parent frame
    static_transform.child_frame_id = "odom";   // Child frame

    // For identity transform (i.e. no offset) set translation to 0
    static_transform.transform.translation.x = 0.0;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.0;
    // Identity rotation
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;

    static_broadcaster_->sendTransform(static_transform);
    RCLCPP_INFO(this->get_logger(), "Static transform from 'map' to 'odom' published.");
  }

  // Reads the CSV file 
  bool readTrajectoryFile(const std::string & filename)
  {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
      return false;
    }
    std::string line;
    // Read the first line and check if it's a header
    if (std::getline(infile, line)) {
      if (line.find("timestamp") != std::string::npos) {
        RCLCPP_INFO(this->get_logger(), "Header detected in CSV; skipping first line.");
      } else {
        processLine(line);
      }
    }
    // Process remaining lines
    while (std::getline(infile, line)) {
      if (line.empty()) {
        continue;
      }
      processLine(line);
    }
    infile.close();
    RCLCPP_INFO(this->get_logger(), "Read %zu points from trajectory file.", points_.size());
    return true;
  }

  // Helper function 
  void processLine(const std::string & line)
  {
    std::istringstream ss(line);
    std::string token;
    // Parse CSV values
    std::getline(ss, token, ',');
    double timestamp = std::stod(token);
    std::getline(ss, token, ',');
    double x = std::stod(token);
    std::getline(ss, token, ',');
    double y = std::stod(token);
    std::getline(ss, token, ',');
    double theta = std::stod(token);

    // Create a PointStamped message; points are in the "map" frame
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = "map";
    // Convert timestamp to nanoseconds
    point.header.stamp = rclcpp::Time(static_cast<uint64_t>(timestamp * 1e9));
    // Apply offset of -5.5 on x and y to shift from turtlesim's coordinate space to RViz space
    point.point.x = x - 5.5;
    point.point.y = y - 5.5;
    point.point.z = 0.0;

    points_.push_back(point);
  }

  // Transform points from "map" to "odom" and publishe them as a MarkerArray
  void publishMarkers()
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_.lookupTransform("odom", "map", this->now(), std::chrono::milliseconds(100));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";  
    marker.ns = "read_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;

    // Set marker color (green)
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Transform each point from "map" to "odom" and add it to the marker
    for (const auto & point_in_map : points_) {
      geometry_msgs::msg::PointStamped point_in_odom;
      try {
        tf2::doTransform(point_in_map, point_in_odom, transformStamped);
        marker.points.push_back(point_in_odom.point);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Failed to transform point: %s", ex.what());
      }
    }

    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
  }

  // Member variables
  std::string trajectory_file_;
  std::vector<geometry_msgs::msg::PointStamped> points_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // TF2 objects 
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryReaderPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
