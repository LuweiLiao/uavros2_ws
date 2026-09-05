#include "rotors_model.hh"

#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

namespace gazebo {

GazeboMotorModel::~GazeboMotorModel() { }

void GazeboMotorModel::InitializeParams()
{
    if (publish_speed_) {
        turning_velocity_msg_.set_data(joint_->GetVelocity(0));
        motor_velocity_pub_->Publish(turning_velocity_msg_);
    }
    if (publish_position_) {
        position_msg_.set_data(joint_->Position(0));
        motor_position_pub_->Publish(position_msg_);
    }
    if (publish_force_) {
        force_msg_.set_data(joint_->GetForce(0));
        motor_force_pub_->Publish(force_msg_);
    }
}

}