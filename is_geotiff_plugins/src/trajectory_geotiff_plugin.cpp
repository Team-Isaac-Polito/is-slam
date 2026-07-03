//=================================================================================================
// Copyright (c) 2012, Gregor Gebhardt, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

// Modifications for ROS2 by Team ISAAC, Politecnico di Torino, 2026

#include <is_geotiff/map_writer_interface.h>
#include <is_geotiff/map_writer_plugin_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <memory>
#include <string>
#include <vector>

namespace is_geotiff_plugins
{

using namespace is_geotiff;

class TrajectoryMapWriter : public MapWriterPluginInterface
{
public:
  TrajectoryMapWriter();
  ~TrajectoryMapWriter() = default;

  void initialize(const std::string& name) override;
  void draw(MapWriterInterface *interface) override;

protected:
  rclcpp::Node::SharedPtr node_;
  bool initialized_;
  std::string trajectory_topic_;
  int path_color_r_;
  int path_color_g_;
  int path_color_b_;
};

TrajectoryMapWriter::TrajectoryMapWriter()
    : initialized_(false)
{}

void TrajectoryMapWriter::initialize(const std::string& name)
{
  (void)name;

  node_ = std::make_shared<rclcpp::Node>("trajectory_map_writer");

  node_->declare_parameter<std::string>("trajectory_topic", "trajectory");
  node_->declare_parameter<int>("path_color_r", 120);
  node_->declare_parameter<int>("path_color_g", 0);
  node_->declare_parameter<int>("path_color_b", 240);

  node_->get_parameter("trajectory_topic", trajectory_topic_);
  node_->get_parameter("path_color_r", path_color_r_);
  node_->get_parameter("path_color_g", path_color_g_);
  node_->get_parameter("path_color_b", path_color_b_);

  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "Successfully initialized is_geotiff MapWriter plugin %s.", name.c_str());
}

void TrajectoryMapWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;

    nav_msgs::msg::Path trajectory_msg;
    const bool received_trajectory = rclcpp::wait_for_message<nav_msgs::msg::Path>(
      trajectory_msg, node_, trajectory_topic_, std::chrono::seconds(4));
    if (!received_trajectory) {
      RCLCPP_ERROR(node_->get_logger(), "Cannot draw trajectory, topic %s unavailable", trajectory_topic_.c_str());
      return;
    }

    const auto& traj_vector = trajectory_msg.poses;

    size_t size = traj_vector.size();

    std::vector<Eigen::Vector2f> pointVec;
    pointVec.resize(size);

    for (size_t i = 0; i < size; ++i){
      const geometry_msgs::msg::PoseStamped& pose (traj_vector[i]);

      pointVec[i] = Eigen::Vector2f(pose.pose.position.x, pose.pose.position.y);
    }

    if (size > 0){
      //Eigen::Vector3f startVec(pose_vector[0].x,pose_vector[0].y,pose_vector[0].z);
      Eigen::Vector3f startVec(pointVec[0].x(),pointVec[0].y(),0.0f);
      interface->drawPath(startVec, pointVec, path_color_r_, path_color_g_, path_color_b_);
    }
}

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(is_geotiff_plugins::TrajectoryMapWriter, is_geotiff::MapWriterPluginInterface)
