#pragma once

// SYSTEM
#include <stdio.h>

// 3RD PARTY

#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/default_topics.h> // This comes from the mav_comm repo

#include "Float32.pb.h" 

static constexpr char MOTOR_MEASUREMENT[] = "motor_speed";
static constexpr char MOTOR_POSITION_MEASUREMENT[] = "motor_position";
static constexpr char MOTOR_FORCE_MEASUREMENT[] = "motor_force";
static constexpr char COMMAND_ACTUATORS[] = "command/motor_speed";

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
} // namespace turning_direction

enum class MotorType { kVelocity, kPosition, kForce };

namespace gazebo {

// Changed name from speed to input for more generality. TODO(kajabo): integrate general actuator command.
typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed> GzCommandMotorInputMsgPtr;

// Set the max_force_ to the max double value. The limitations get handled by the FirstOrderFilter.
static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
static constexpr double kDefaultMotorConstant = 8.54858e-06;
static constexpr double kDefaultMomentConstant = 0.016;
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
static constexpr double kDefaultMaxRotVelocity = 838.0;
static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;

class GazeboMotorModel : public ModelPlugin {
public:
    GazeboMotorModel().
        : ModelPlugin()
        , command_sub_topic_(COMMAND_ACTUATORS)
        , motor_speed_pub_topic_(MOTOR_MEASUREMENT)
        , motor_position_pub_topic_(MOTOR_POSITION_MEASUREMENT)
        , motor_force_pub_topic_(MOTOR_FORCE_MEASUREMENT)
    {
    }

    virtual ~GazeboMotorModel();

    virtual void InitializeParams();
    virtual void Publish();

protected:
    virtual void UpdateForcesAndMoments();
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo& /*_info*/);

private:
    std::string command_sub_topic_;
    std::string motor_speed_pub_topic_;
    std::string motor_position_pub_topic_;
    std::string motor_force_pub_topic_;

    std::string joint_name_;
    std::string link_name_;
    std::string namespace_;

    bool publish_speed_;
    bool publish_position_;
    bool publish_force_;

    int motor_number_;
    int turning_direction_;
    MotorType motor_type_;

    double max_force_;
    double max_rot_velocity_;
    double moment_constant_;
    double motor_constant_;
    double ref_motor_input_;
    double rolling_moment_coefficient_;
    double rotor_drag_coefficient_;
    double rotor_velocity_slowdown_sim_;
    double time_constant_down_;
    double time_constant_up_;

    common::PID pids_;

    gazebo::transport::NodePtr node_handle_;

    gazebo::transport::PublisherPtr motor_velocity_pub_;

    gazebo::transport::PublisherPtr motor_position_pub_;

    gazebo::transport::PublisherPtr motor_force_pub_;

    gazebo::transport::SubscriberPtr command_sub_;

    gazebo::transport::SubscriberPtr wind_speed_sub_;

    physics::ModelPtr model_;
    physics::JointPtr joint_;
    physics::LinkPtr link_;

    event::ConnectionPtr updateConnection_;

    boost::thread callback_queue_thread_;

    void QueueThread();

    gz_std_msgs::Float32 turning_velocity_msg_;
    gz_std_msgs::Float32 position_msg_;
    gz_std_msgs::Float32 force_msg_;
}

}