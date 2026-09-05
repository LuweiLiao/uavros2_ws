/*
 * Copyright 2026
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_TRAJECTORY_VISUAL_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_TRAJECTORY_VISUAL_PLUGIN_H

#include <deque>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/DynamicLines.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {

static constexpr double kDefaultTrajectoryMinDistance = 0.05;
static constexpr int kDefaultTrajectoryMaxPoints = 2000;
static constexpr double kDefaultTrajectoryZOffset = 0.0;

class GazeboTrajectoryVisualPlugin : public VisualPlugin {
 public:
  GazeboTrajectoryVisualPlugin();
  ~GazeboTrajectoryVisualPlugin() override;

  void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override;

 private:
  void OnUpdate();
  bool EnsureLine();
  void RebuildLine();

  rendering::VisualPtr parent_visual_;
  rendering::VisualPtr world_visual_;
  rendering::ScenePtr scene_;
  rendering::DynamicLines* line_;
  event::ConnectionPtr update_connection_;

  std::deque<ignition::math::Vector3d> points_;
  ignition::math::Vector3d last_point_;
  ignition::math::Color color_;

  double min_distance_;
  double z_offset_;
  int max_points_;
  bool has_last_point_;
};

}  // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_GAZEBO_TRAJECTORY_VISUAL_PLUGIN_H
