#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "amr_custom_msgs/srv/save_trajectory.hpp"  // Custom service header
#include <deque>
#include <mutex>
#include <fstream>
#include <vector>
#include <algorithm>

// Structure to store a pose along with its timestamp
struct StampedPose
{
  rclcpp::Time stamp;
  turtlesim::msg::Pose pose;
};

class TrajectoryPublisherSaver : public rclcpp::Node
{
public:
  TrajectoryPublisherSaver() : Node("trajectory_publisher_saver")
  {
    // Subscribe to turtle1 pose topic
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&TrajectoryPublisherSaver::pose_callback, this, std::placeholders::_1));

    // Publisher for MarkerArray messages for visualization in RViz
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 10);

    // Timer to publish markers 
    marker_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TrajectoryPublisherSaver::publish_markers, this));

    // Create the service to save trajectory data.
    service_ = this->create_service<amr_custom_msgs::srv::SaveTrajectory>(
      "save_trajectory",
      std::bind(&TrajectoryPublisherSaver::save_trajectory_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Trajectory node started. Collecting poses, publishing markers, and offering save service.");
  }

private:
  // Callback to capture and store the pose data with a timestamp.
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    rclcpp::Time current_time = this->now();
    StampedPose stamped_pose;
    stamped_pose.stamp = current_time;
    stamped_pose.pose = *msg;

    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      trajectory_buffer_.push_back(stamped_pose);
    }
  }

  // Periodically publish a MarkerArray that visualizes the trajectory
  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;

    // Set up the marker header
    marker.header.frame_id = "map";  // Using "map" as our frame
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Set the line width.
    marker.scale.x = 0.05;
    
    // Set the marker color (red)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      // Convert each buffered pose into a point in the marker
      for (const auto & stamped_pose : trajectory_buffer_) {
        geometry_msgs::msg::Point pt;
        pt.x = stamped_pose.pose.x - 5.5;
        pt.y = stamped_pose.pose.y - 5.5;
        pt.z = 0.0;  // turtlesim is 2D, set z = 0
        marker.points.push_back(pt);
      }
    }

    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
  }

  // Service callback to save trajectory data.
  void save_trajectory_callback(
    const std::shared_ptr<amr_custom_msgs::srv::SaveTrajectory::Request> request,
    std::shared_ptr<amr_custom_msgs::srv::SaveTrajectory::Response> response)
  {
    rclcpp::Time current_time = this->now();
    std::vector<StampedPose> filtered_poses;

    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      // Iterate from the newest to the oldest and collect poses within the duratio
      for (auto rit = trajectory_buffer_.rbegin(); rit != trajectory_buffer_.rend(); ++rit) {
        double dt = (current_time - rit->stamp).seconds();
        if (dt <= request->duration) {
          // Insert at beginning to maintain chronological order
          filtered_poses.insert(filtered_poses.begin(), *rit);
        } else {
          break;
        }
      }
    }

    // Open file for writing
    std::ofstream outfile(request->filename);
    if (!outfile.is_open()) {
      response->success = false;
      response->message = "Failed to open file: " + request->filename;
      return;
    }

    // Write CSV header
    outfile << "timestamp,x,y,theta\n";
    for (const auto & pose : filtered_poses) {
      outfile << pose.stamp.seconds() << ","
              << pose.pose.x << ","
              << pose.pose.y << ","
              << pose.pose.theta << "\n";
    }
    outfile.close();

    response->success = true;
    response->message = "Trajectory saved successfully with " + std::to_string(filtered_poses.size()) + " points.";
    RCLCPP_INFO(this->get_logger(), "Saved trajectory with %zu points.", filtered_poses.size());
  }

  // Buffer for storing poses
  std::deque<StampedPose> trajectory_buffer_;
  std::mutex buffer_mutex_;

  // ROS interfaces
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  rclcpp::Service<amr_custom_msgs::srv::SaveTrajectory>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPublisherSaver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
