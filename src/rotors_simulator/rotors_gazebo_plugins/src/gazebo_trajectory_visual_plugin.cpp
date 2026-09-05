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

#include "rotors_gazebo_plugins/gazebo_trajectory_visual_plugin.h"

#include <boost/bind.hpp>

namespace gazebo {

GazeboTrajectoryVisualPlugin::GazeboTrajectoryVisualPlugin()
    : VisualPlugin(),
      line_(nullptr),
      color_(1.0, 0.2, 0.1, 1.0),
      min_distance_(kDefaultTrajectoryMinDistance),
      z_offset_(kDefaultTrajectoryZOffset),
      max_points_(kDefaultTrajectoryMaxPoints),
      has_last_point_(false) {}

GazeboTrajectoryVisualPlugin::~GazeboTrajectoryVisualPlugin() {
  if (this->update_connection_) {
    this->update_connection_.reset();
  }

  if (this->world_visual_ && this->line_) {
    this->world_visual_->DeleteDynamicLine(this->line_);
    this->line_ = nullptr;
  }
}

void GazeboTrajectoryVisualPlugin::Load(rendering::VisualPtr _visual,
                                        sdf::ElementPtr _sdf) {
  this->parent_visual_ = _visual;
  if (!this->parent_visual_) {
    gzerr << "[gazebo_trajectory_visual_plugin] Failed to load parent visual.\n";
    return;
  }

  this->scene_ = this->parent_visual_->GetScene();

  if (_sdf->HasElement("minDistance")) {
    this->min_distance_ = _sdf->Get<double>("minDistance");
  }
  if (_sdf->HasElement("maxPoints")) {
    this->max_points_ = _sdf->Get<int>("maxPoints");
  }
  if (_sdf->HasElement("zOffset")) {
    this->z_offset_ = _sdf->Get<double>("zOffset");
  }
  if (_sdf->HasElement("red")) {
    this->color_.R(_sdf->Get<double>("red"));
  }
  if (_sdf->HasElement("green")) {
    this->color_.G(_sdf->Get<double>("green"));
  }
  if (_sdf->HasElement("blue")) {
    this->color_.B(_sdf->Get<double>("blue"));
  }
  if (_sdf->HasElement("alpha")) {
    this->color_.A(_sdf->Get<double>("alpha"));
  }

  if (this->max_points_ < 2) {
    this->max_points_ = 2;
  }

  this->update_connection_ = event::Events::ConnectPreRender(
      boost::bind(&GazeboTrajectoryVisualPlugin::OnUpdate, this));
}

bool GazeboTrajectoryVisualPlugin::EnsureLine() {
  if (this->line_) {
    return true;
  }

  if (!this->scene_) {
    this->scene_ = this->parent_visual_->GetScene();
    if (!this->scene_) {
      return false;
    }
  }

  if (!this->world_visual_) {
    this->world_visual_ = this->scene_->WorldVisual();
    if (!this->world_visual_) {
      return false;
    }
  }

  this->line_ =
      this->world_visual_->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
  if (!this->line_) {
    gzerr << "[gazebo_trajectory_visual_plugin] Failed to create dynamic line.\n";
    return false;
  }

  this->line_->setMaterial("Gazebo/Red");
  this->line_->setVisibilityFlags(GZ_VISIBILITY_GUI);
  this->line_->Update();
  return true;
}

void GazeboTrajectoryVisualPlugin::RebuildLine() {
  if (!this->line_) {
    return;
  }

  this->line_->Clear();

  if (this->points_.empty()) {
    this->line_->Update();
    return;
  }

  for (const auto& point : this->points_) {
    this->line_->AddPoint(point, this->color_);
  }

  if (this->points_.size() == 1u) {
    this->line_->AddPoint(this->points_.front(), this->color_);
  }

  this->line_->Update();
}

void GazeboTrajectoryVisualPlugin::OnUpdate() {
  if (!this->parent_visual_ || !this->EnsureLine()) {
    return;
  }

  ignition::math::Vector3d current_point =
      this->parent_visual_->WorldPose().Pos() +
      ignition::math::Vector3d(0.0, 0.0, this->z_offset_);

  if (!this->has_last_point_ ||
      current_point.Distance(this->last_point_) >= this->min_distance_) {
    this->points_.push_back(current_point);
    this->last_point_ = current_point;
    this->has_last_point_ = true;

    while (static_cast<int>(this->points_.size()) > this->max_points_) {
      this->points_.pop_front();
    }

    this->RebuildLine();
  }
}

GZ_REGISTER_VISUAL_PLUGIN(GazeboTrajectoryVisualPlugin)

}  // namespace gazebo
