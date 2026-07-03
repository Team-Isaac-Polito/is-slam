//=================================================================================================
// Copyright (c) 2026, Team ISAAC, Politecnico di Torino

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

#include <is_geotiff/map_writer_interface.h>
#include <is_geotiff/map_writer_plugin_interface.h>

#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

namespace is_geotiff_plugins
{

  using namespace is_geotiff;

  class MarkerMapWriter : public MapWriterPluginInterface
  {
  public:
    MarkerMapWriter();
    ~MarkerMapWriter() = default;

    void initialize(const std::string &name) override;
    void draw(MapWriterInterface *interface) override;

  protected:
    rclcpp::Node::SharedPtr node_;
    bool initialized_;
    std::string marker_topic_;
  };

  MarkerMapWriter::MarkerMapWriter()
      : initialized_(false)
  {
  }

  void MarkerMapWriter::initialize(const std::string &name)
  {
    (void)name;

    node_ = std::make_shared<rclcpp::Node>("marker_map_writer");

    node_->declare_parameter<std::string>("marker_topic", "/detection/markers");

    node_->get_parameter("marker_topic", marker_topic_);

    initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "Successfully initialized is_geotiff MapWriter plugin %s.", name.c_str());
  }

  void MarkerMapWriter::draw(MapWriterInterface *interface)
  {
    if (!initialized_)
      return;

    visualization_msgs::msg::Marker marker_msg;
    const bool received_marker = rclcpp::wait_for_message<visualization_msgs::msg::Marker>(
        marker_msg, node_, marker_topic_, std::chrono::seconds(4));
    if (!received_marker)
    {
      RCLCPP_ERROR(node_->get_logger(), "Cannot draw marker, topic %s unavailable", marker_topic_.c_str());
      return;
    }

    for (const auto &marker : marker_msg.markers)
    {
      Eigen::Vector2f coords(marker.pose.position.x, marker.pose.position.y);

      MapWriterInterface::Color color(static_cast<unsigned int>(marker.color.r * 255),
                                      static_cast<unsigned int>(marker.color.g * 255),
                                      static_cast<unsigned int>(marker.color.b * 255));

      Shape shape;
      switch (marker.type)
      {
      case visualization_msgs::msg::Marker::CUBE:
        shape = SHAPE_DIAMOND;
        break;
      case visualization_msgs::msg::Marker::SPHERE:
        shape = SHAPE_CIRCLE;
        break;
      default:
        shape = SHAPE_CIRCLE; // Default shape
        break;
      }

      std::string text = marker.text.substr(0, 2);
      


      interface->drawObjectOfInterest(coords, text, color, shape);
    }
  }

} // namespace

// register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(is_geotiff_plugins::MarkerMapWriter, is_geotiff::MapWriterPluginInterface)
