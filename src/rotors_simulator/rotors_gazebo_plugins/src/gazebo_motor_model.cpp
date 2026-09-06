/*
 * Gazebo Sim port of the ROS 1 RotorS GazeboMotorModel.
 *
 * The force, drag, reaction-torque, direction and first-order motor model are
 * intentionally kept equivalent to the original implementation.  Only the
 * Gazebo Classic transport / physics calls are replaced with Gazebo Sim API
 * calls.
 */

#include "rotors_gazebo_plugins/gazebo_motor_model.h"
#include "CommandMotorSpeed.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"
#include "Float32.pb.h"
#include "WindSpeed.pb.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gz/math/PID.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

namespace
{

enum class MotorType
{
  kVelocity,
  kPosition,
  kForce
};

constexpr double kDefaultMotorConstant = 8.54858e-6;
constexpr double kDefaultMomentConstant = 0.016;
constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
constexpr double kDefaultMaxRotVelocity = 838.0;
constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;
constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

std::string Trim(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
    return {};
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string NamespacedTopic(const std::string &robotNamespace,
                            const std::string &topic)
{
  // ROS 1 concatenates namespace_ + "/" + topic, even if the SDF topic
  // starts with '/'. Collapse redundant separators for Gazebo Sim transport.
  std::string result{"/"};
  for (const char c : robotNamespace + "/" + topic)
    if (c != '/' || result.back() != '/')
      result += c;
  if (result.size() > 1 && result.back() == '/')
    result.pop_back();
  return result;
}

double NormalizeAngle(double input)
{
  // Keep the ROS 1 [0, 2*pi) wrap, including its boundary tolerance.
  double wrapped = std::copysign(std::fmod(std::abs(input), 2*M_PI), input);
  if (std::abs(wrapped - 2*M_PI) < 1e-8)
    wrapped = 0;
  if (wrapped < 0)
    wrapped += 2*M_PI;
  return wrapped;
}

std::string LastScopePart(const std::string &name)
{
  const std::size_t separator = name.rfind("::");
  if (separator == std::string::npos)
    return name;
  return name.substr(separator + 2);
}

gz::sim::Entity FindEntity(
    const std::string &_name,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::Entity _relative,
    gz::sim::ComponentTypeId _type)
{
  auto matches = gz::sim::entitiesFromScopedName(_name, _ecm, _relative);
  if (matches.empty())
    matches = gz::sim::entitiesFromScopedName(_name, _ecm);
  for (const auto entity : matches)
  {
    if (_ecm.EntityHasComponentType(entity, _type))
      return entity;
  }
  return gz::sim::kNullEntity;
}

}  // namespace

namespace rotors_gazebo_plugins
{

class GazeboMotorModel::Private
{
public:
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity joint{gz::sim::kNullEntity};
  gz::sim::Entity rotorLink{gz::sim::kNullEntity};
  // ROS 1 obtains this link from link_->GetParentJointsLinks().at(0).
  // Keep the same relationship instead of selecting a vehicle-specific link
  // name.
  gz::sim::Entity parentLink{gz::sim::kNullEntity};

  std::string jointName;
  std::string linkName;
  std::string parentLinkName;
  std::string commandTopic{"command/motor_speed"};
  std::string motorSpeedTopic{"motor_speed"};
  std::string windTopic;
  std::string positionTopic;
  std::string forceTopic;
  gz::math::PID positionPid;

  int motorNumber{0};
  double turningDirection{-1.0};
  MotorType motorType{MotorType::kVelocity};
  double maxForce{std::numeric_limits<double>::max()};
  double maxRotVelocity{kDefaultMaxRotVelocity};
  double momentConstant{kDefaultMomentConstant};
  double motorConstant{kDefaultMotorConstant};
  double rollingMomentCoefficient{kDefaultRollingMomentCoefficient};
  double rotorDragCoefficient{kDefaultRotorDragCoefficient};
  double rotorVelocitySlowdownSim{kDefaultRotorVelocitySlowdownSim};
  double timeConstantDown{kDefaultTimeConstantDown};
  double timeConstantUp{kDefaultTimeConstantUp};

  double referenceInput{0.0};
  double filteredInput{0.0};
  double previousSimTime{0.0};
  gz::math::Vector3d windSpeedWorld{0, 0, 0};
  gz::math::Vector3d jointAxis{0, 0, 1};
  std::string jointAxisExpressedIn;
  bool jointAxisResolved{false};

  bool resolved{false};
  bool warnedMissingCommand{false};
  std::mutex mutex;

  gz::transport::Node node;
  gz::transport::Node::Subscriber commandSubscriber;
  gz::transport::Node::Subscriber windSubscriber;
  gz::transport::Node::Publisher speedPublisher;
  gz::transport::Node::Publisher positionPublisher;
  gz::transport::Node::Publisher forcePublisher;
  gz::transport::Node::Publisher legacySpeedPublisher;
  gz::transport::Node::Publisher legacyPositionPublisher;
  gz::transport::Node::Publisher legacyForcePublisher;
  gz::transport::Node::Publisher connectGazeboToRos;
  gz::transport::Node::Publisher connectRosToGazebo;
  std::vector<gz_std_msgs::ConnectGazeboToRosTopic> gazeboRequests;
  std::vector<gz_std_msgs::ConnectRosToGazeboTopic> rosRequests;
  double nextRegistrationTime{-1.0};
  bool rosTopicsCreated{false};

  void OnLegacyCommand(const gz_mav_msgs::CommandMotorSpeed &message)
  {
    gz::msgs::Actuators native;
    for (float value : message.motor_speed()) native.add_velocity(value);
    OnCommand(native);
  }

  void OnLegacyWind(const gz_mav_msgs::WindSpeed &message)
  {
    OnWind(message.velocity());
  }

  void CreateRosTopics(const gz::sim::EntityComponentManager &ecm)
  {
    if (rosTopicsCreated)
      return;
    const auto world = gz::sim::worldEntity(modelEntity, ecm);
    const auto worldName = ecm.Component<gz::sim::components::Name>(world);
    if (!worldName)
      return;

    // As in ROS 1 CreatePubsAndSubs, initialize on update, when the model
    // hierarchy is available. A missing parent during Configure is not a
    // permanent decision to omit the original ROS routes.
    const std::string worldPrefix = "/gazebo/" + worldName->Data();
    connectGazeboToRos = node.Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
        worldPrefix + "/connect_gazebo_to_ros_subtopic");
    connectRosToGazebo = node.Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
        worldPrefix + "/connect_ros_to_gazebo_subtopic");
    const auto addMeasurement = [&](const std::string &topic,
                                   gz::transport::Node::Publisher &publisher)
    {
      publisher = node.Advertise<gz_std_msgs::Float32>(worldPrefix + topic);
      gz_std_msgs::ConnectGazeboToRosTopic request;
      request.set_gazebo_topic("~" + topic); request.set_ros_topic(topic);
      request.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
      gazeboRequests.push_back(request);
    };
    if (speedPublisher)
      addMeasurement(motorSpeedTopic, legacySpeedPublisher);
    if (positionPublisher)
      addMeasurement(positionTopic, legacyPositionPublisher);
    if (forcePublisher)
      addMeasurement(forceTopic, legacyForcePublisher);
    for (const auto &entry : std::vector<std::pair<std::string,
         gz_std_msgs::ConnectRosToGazeboTopic::MsgType>>{
         {commandTopic, gz_std_msgs::ConnectRosToGazeboTopic::COMMAND_MOTOR_SPEED},
         {windTopic, gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED}})
    {
      gz_std_msgs::ConnectRosToGazeboTopic request;
      request.set_gazebo_topic("~" + entry.first); request.set_ros_topic(entry.first);
      request.set_msgtype(entry.second); rosRequests.push_back(request);
    }
    (void)node.Subscribe(worldPrefix + commandTopic, &Private::OnLegacyCommand, this);
    (void)node.Subscribe(worldPrefix + windTopic, &Private::OnLegacyWind, this);
    rosTopicsCreated = true;
  }

  void RegisterRosTopics(double simTime)
  {
    if (simTime >= nextRegistrationTime || simTime < previousSimTime)
    {
      // Gazebo Sim has no Classic Publish(message, blocking=true). Wait
      // non-blockingly for the world bridge and repeat idempotent requests
      // so a late-loaded/reloaded bridge also receives the original routes.
      if (connectGazeboToRos && connectGazeboToRos.HasConnections())
        for (const auto &request : gazeboRequests) connectGazeboToRos.Publish(request);
      if (connectRosToGazebo && connectRosToGazebo.HasConnections())
        for (const auto &request : rosRequests) connectRosToGazebo.Publish(request);
      nextRegistrationTime = simTime + 1.0;
    }
  }

  void OnCommand(const gz::msgs::Actuators &_message)
  {
    double value = 0.0;
    bool present = false;
    if (motorNumber >= 0 && motorNumber < _message.velocity_size())
    {
      value = _message.velocity(motorNumber);
      present = true;
    }

    if (!present)
    {
      if (!warnedMissingCommand)
      {
        gzerr << "[rotors_gazebo_motor_model] command message has no motor "
              << motorNumber << " entry\n";
        warnedMissingCommand = true;
      }
      return;
    }

    std::lock_guard<std::mutex> lock(mutex);
    switch (motorType)
    {
      case MotorType::kVelocity:
        referenceInput = std::min(value, maxRotVelocity);
        break;
      case MotorType::kForce:
        referenceInput = std::min(value, maxForce);
        break;
      case MotorType::kPosition:
        referenceInput = value;
        break;
    }
  }

  void OnWind(const gz::msgs::Vector3d &_message)
  {
    std::lock_guard<std::mutex> lock(mutex);
    windSpeedWorld.Set(_message.x(), _message.y(), _message.z());
  }
};

/////////////////////////////////////////////////
GazeboMotorModel::GazeboMotorModel()
  : dataPtr(new Private)
{
}

/////////////////////////////////////////////////
GazeboMotorModel::~GazeboMotorModel() = default;

/////////////////////////////////////////////////
void GazeboMotorModel::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  dataPtr->modelEntity = _entity;
  if (!_ecm.EntityHasComponentType(_entity,
                                  gz::sim::components::Model::typeId))
  {
    gzerr << "[rotors_gazebo_motor_model] plugin must be attached to a model\n";
    return;
  }

  dataPtr->jointName = Trim(_sdf->Get("jointName", std::string{}).first);
  dataPtr->linkName = Trim(_sdf->Get("linkName", std::string{}).first);
  dataPtr->motorNumber = _sdf->Get("motorNumber", 0).first;
  dataPtr->motorConstant = _sdf->Get(
      "motorConstant", dataPtr->motorConstant).first;
  dataPtr->momentConstant = _sdf->Get(
      "momentConstant", dataPtr->momentConstant).first;
  dataPtr->rotorDragCoefficient = _sdf->Get(
      "rotorDragCoefficient", dataPtr->rotorDragCoefficient).first;
  dataPtr->rollingMomentCoefficient = _sdf->Get(
      "rollingMomentCoefficient", dataPtr->rollingMomentCoefficient).first;
  dataPtr->maxRotVelocity = _sdf->Get(
      "maxRotVelocity", dataPtr->maxRotVelocity).first;
  dataPtr->timeConstantUp = _sdf->Get(
      "timeConstantUp", dataPtr->timeConstantUp).first;
  dataPtr->timeConstantDown = _sdf->Get(
      "timeConstantDown", dataPtr->timeConstantDown).first;
  dataPtr->rotorVelocitySlowdownSim = _sdf->Get(
      "rotorVelocitySlowdownSim", dataPtr->rotorVelocitySlowdownSim).first;
  const std::string robotNamespace = _sdf->Get(
      "robotNamespace", std::string{}).first;
  dataPtr->commandTopic = NamespacedTopic(robotNamespace, _sdf->Get(
      "commandSubTopic", dataPtr->commandTopic).first);
  dataPtr->motorSpeedTopic = NamespacedTopic(robotNamespace, _sdf->Get(
      "motorSpeedPubTopic", dataPtr->motorSpeedTopic).first);

  const std::string direction = _sdf->Get(
      "turningDirection", std::string("cw")).first;
  if (direction == "ccw")
    dataPtr->turningDirection = 1.0;
  else if (direction == "cw")
    dataPtr->turningDirection = -1.0;
  else
    gzerr << "[rotors_gazebo_motor_model] turningDirection must be cw or ccw\n";

  const std::string type = _sdf->Get(
      "motorType", std::string("velocity")).first;
  if (type == "position")
    dataPtr->motorType = MotorType::kPosition;
  else if (type == "force")
    dataPtr->motorType = MotorType::kForce;
  else
    dataPtr->motorType = MotorType::kVelocity;

  // Original joint_control_pid keys and zero defaults. Gazebo Math PID is
  // the Gazebo Sim successor to Classic common::PID (same error sign).
  dataPtr->positionPid.Init(0, 0, 0, 0, 0, 0, 0);
  if (dataPtr->motorType == MotorType::kPosition)
  {
    if (_sdf->HasElement("joint_control_pid"))
    {
      const auto pid = _sdf->GetElementImpl("joint_control_pid");
      dataPtr->positionPid.Init(
          pid->Get("p", 0.0).first, pid->Get("i", 0.0).first,
          pid->Get("d", 0.0).first, pid->Get("iMax", 0.0).first,
          pid->Get("iMin", 0.0).first, pid->Get("cmdMax", 0.0).first,
          pid->Get("cmdMin", 0.0).first);
    }
    else
      gzerr << "[rotors_gazebo_motor_model] PID values not found, "
               "setting all values to zero\n";
  }

  if (dataPtr->rotorVelocitySlowdownSim == 0.0)
    dataPtr->rotorVelocitySlowdownSim = 1.0;

  if (!dataPtr->node.Subscribe(
      dataPtr->commandTopic, &Private::OnCommand, dataPtr.get()))
  {
    gzerr << "[rotors_gazebo_motor_model] failed to subscribe to "
          << dataPtr->commandTopic << "\n";
  }

  // ROS 1 always subscribes, defaulting to <robotNamespace>/wind_speed.
  // Vector3d carries the legacy WindSpeed.velocity field at this boundary.
  dataPtr->windTopic = NamespacedTopic(robotNamespace, _sdf->Get(
      "windSpeedSubTopic", std::string("wind_speed")).first);
  (void)dataPtr->node.Subscribe(dataPtr->windTopic, &Private::OnWind, dataPtr.get());

  if (!dataPtr->motorSpeedTopic.empty())
  {
    dataPtr->speedPublisher = dataPtr->node.Advertise<gz::msgs::Float>(
        dataPtr->motorSpeedTopic);
  }
  // As in ROS 1 Load(), these measurements are opt-in SDF topics. Keep
  // them in the original motor plugin; ROS forwarding belongs to the
  // original GazeboRosInterfacePlugin, not a new telemetry plugin.
  if (_sdf->HasElement("motorPositionPubTopic"))
  {
    dataPtr->positionTopic = NamespacedTopic(robotNamespace,
        _sdf->Get<std::string>("motorPositionPubTopic"));
    dataPtr->positionPublisher = dataPtr->node.Advertise<gz::msgs::Float>(dataPtr->positionTopic);
  }
  if (_sdf->HasElement("motorForcePubTopic"))
  {
    dataPtr->forceTopic = NamespacedTopic(robotNamespace,
        _sdf->Get<std::string>("motorForcePubTopic"));
    dataPtr->forcePublisher = dataPtr->node.Advertise<gz::msgs::Float>(dataPtr->forceTopic);
  }

  // Nested RotorS links and top-level joints may be created after Configure.
  // ResolveEntities() retries from PreUpdate until the complete hierarchy is
  // available.
  (void)_ecm;
}

/////////////////////////////////////////////////
void GazeboMotorModel::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!dataPtr->resolved)
  {
    dataPtr->joint = FindEntity(
        dataPtr->jointName, _ecm, dataPtr->modelEntity,
        gz::sim::components::Joint::typeId);
    dataPtr->rotorLink = FindEntity(
        dataPtr->linkName, _ecm, dataPtr->modelEntity,
        gz::sim::components::Link::typeId);

    if (dataPtr->joint == gz::sim::kNullEntity ||
        dataPtr->rotorLink == gz::sim::kNullEntity)
    {
      return;
    }

    // ROS 1 uses link_->GetParentJointsLinks().at(0) for the reaction
    // torque. Resolve the parent from the joint so nested RotorS models keep
    // their original topology instead of relying on a hard-coded link name.
    const gz::sim::Joint joint(dataPtr->joint);
    const auto parentName = joint.ParentLinkName(_ecm);
    if (!parentName)
    {
      gzerr << "[rotors_gazebo_motor_model] joint ["
            << dataPtr->jointName << "] has no parent link\n";
      return;
    }
    dataPtr->parentLinkName = Trim(*parentName);
    dataPtr->parentLink = FindEntity(
        dataPtr->parentLinkName, _ecm, dataPtr->modelEntity,
        gz::sim::components::Link::typeId);
    if (dataPtr->parentLink == gz::sim::kNullEntity)
    {
      dataPtr->parentLink = FindEntity(
          LastScopePart(dataPtr->parentLinkName), _ecm, dataPtr->modelEntity,
          gz::sim::components::Link::typeId);
    }

    const auto axes = joint.Axis(_ecm);
    if (axes && !axes->empty())
    {
      dataPtr->jointAxis = axes->front().Xyz();
      if (dataPtr->jointAxis.Length() > 1e-9)
      {
        dataPtr->jointAxis.Normalize();
        dataPtr->jointAxisExpressedIn = axes->front().XyzExpressedIn();
        dataPtr->jointAxisResolved = true;
      }
    }

    if (dataPtr->parentLink == gz::sim::kNullEntity)
    {
      gzerr << "[rotors_gazebo_motor_model] cannot resolve parent link ["
            << dataPtr->parentLinkName << "] for joint ["
            << dataPtr->jointName << "]\n";
      return;
    }

    // Gazebo Sim does not populate JointVelocity by default.  The ROS 1
    // plugin always read the physics joint velocity on every update, so make
    // that check explicit before using the component below.  Without this
    // call the command still rotates the propeller, but the measured velocity
    // remains unavailable and the migrated thrust equation evaluates to zero.
    gz::sim::Joint(dataPtr->joint).EnableVelocityCheck(_ecm);
    if (dataPtr->motorType == MotorType::kPosition || dataPtr->positionPublisher)
      gz::sim::Joint(dataPtr->joint).EnablePositionCheck(_ecm);

    gz::sim::Link(dataPtr->rotorLink).EnableVelocityChecks(_ecm);
    gz::sim::Link(dataPtr->parentLink).EnableVelocityChecks(_ecm);
    dataPtr->resolved = true;

    gzmsg << "[rotors_gazebo_motor_model] motor " << dataPtr->motorNumber
          << " resolved joint " << dataPtr->jointName << " and link "
          << dataPtr->linkName << "; parent link "
          << dataPtr->parentLinkName << "\n";
  }

  const double simTime = std::chrono::duration<double>(_info.simTime).count();
  dataPtr->CreateRosTopics(_ecm);
  dataPtr->RegisterRosTopics(simTime);
  double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt <= 0.0)
    dt = simTime - dataPtr->previousSimTime;
  dataPtr->previousSimTime = simTime;
  if (dt <= 0.0)
    return;

  double reference;
  gz::math::Vector3d wind;
  {
    std::lock_guard<std::mutex> lock(dataPtr->mutex);
    reference = dataPtr->referenceInput;
    wind = dataPtr->windSpeedWorld;
  }

  auto *velocityComponent = _ecm.Component<
      gz::sim::components::JointVelocity>(dataPtr->joint);
  const double motorVelocity = velocityComponent &&
      !velocityComponent->Data().empty() ? velocityComponent->Data()[0] : 0.0;
  const double realMotorVelocity = motorVelocity *
      dataPtr->rotorVelocitySlowdownSim;

  if (dataPtr->motorType == MotorType::kVelocity)
  {
    // Exact discrete first-order filter used by RotorS' original
    // FirstOrderFilter class (different rise and fall time constants).
    const double tau = reference > dataPtr->filteredInput ?
        dataPtr->timeConstantUp : dataPtr->timeConstantDown;
    const double alpha = tau > 0.0 ? std::exp(-dt / tau) : 0.0;
    dataPtr->filteredInput = alpha * dataPtr->filteredInput +
        (1.0 - alpha) * reference;

    // Classic SetVelocity changes the joint state directly. A velocity
    // command instead asks the physics engine to apply a tracking torque,
    // which adds dynamics absent from the original RotorS implementation.
    gz::sim::Joint(dataPtr->joint).ResetVelocity(_ecm,
        {dataPtr->turningDirection * dataPtr->filteredInput /
         dataPtr->rotorVelocitySlowdownSim});
  }
  else
  {
    double force = reference;
    if (dataPtr->motorType == MotorType::kPosition)
    {
      const auto position = gz::sim::Joint(dataPtr->joint).Position(_ecm);
      if (!position || position->empty())
        return;  // Wait for physics to provide the requested joint state.
      double error = NormalizeAngle(position->front()) - NormalizeAngle(reference);
      if (error > M_PI)
        error -= 2*M_PI;
      if (error < -M_PI)
        error += 2*M_PI;
      if (std::abs(error - M_PI) < 1e-8)
        error = -M_PI;
      force = dataPtr->positionPid.Update(error,
          std::chrono::duration<double>(dt));
    }
    // Classic Joint::SetForce calls CheckAndTruncateForce before saving
    // the effort returned by GetForce. Preserve that behavior explicitly;
    // JointForceCmd must not report an unclamped reference as applied effort.
    const auto axes = gz::sim::Joint(dataPtr->joint).Axis(_ecm);
    if (axes && !axes->empty())
    {
      const double velocityLimit = axes->front().MaxVelocity();
      if (velocityLimit >= 0.0)
      {
        if ((motorVelocity > velocityLimit && force > 0.0) ||
            (motorVelocity < -velocityLimit && force < 0.0))
          force = 0.0;
      }
      const double effortLimit = axes->front().Effort();
      if (effortLimit >= 0.0)
        force = std::clamp(force, -effortLimit, effortLimit);
    }
    auto *forceCommand = _ecm.Component<
        gz::sim::components::JointForceCmd>(dataPtr->joint);
    if (forceCommand == nullptr)
    {
      forceCommand = _ecm.CreateComponent(
          dataPtr->joint, gz::sim::components::JointForceCmd({force}));
    }
    if (forceCommand != nullptr)
    {
      if (forceCommand->Data().empty())
        forceCommand->Data().push_back(force);
      else
        forceCommand->Data()[0] = force;
    }
  }

  // All three original modes publish measured joint speed, but only velocity
  // mode applies propeller aerodynamics. A force/position actuator is not a
  // propeller even when its measured joint velocity is nonzero.
  if (dataPtr->speedPublisher)
  {
    gz::msgs::Float speed;
    speed.set_data(static_cast<float>(motorVelocity));
    dataPtr->speedPublisher.Publish(speed);
    if (dataPtr->legacySpeedPublisher)
    {
      gz_std_msgs::Float32 legacy; legacy.set_data(speed.data());
      dataPtr->legacySpeedPublisher.Publish(legacy);
    }
  }
  if (dataPtr->positionPublisher)
  {
    const auto position = gz::sim::Joint(dataPtr->joint).Position(_ecm);
    if (position && !position->empty())
    {
      gz::msgs::Float message;
      // Original Publish() reads Position(0), not the wrapped PID error or
      // command reference. Multi-turn position and its sign are preserved.
      message.set_data(static_cast<float>(position->front()));
      dataPtr->positionPublisher.Publish(message);
      if (dataPtr->legacyPositionPublisher)
      {
        gz_std_msgs::Float32 legacy; legacy.set_data(message.data());
        dataPtr->legacyPositionPublisher.Publish(legacy);
      }
    }
  }
  if (dataPtr->forcePublisher)
  {
    const auto *force = _ecm.Component<gz::sim::components::JointForceCmd>(
        dataPtr->joint);
    gz::msgs::Float message;
    // Classic ODEJoint::GetForce reports saved SetForce effort, not the
    // joint's transmitted wrench. With no SetForce command it is zero.
    message.set_data(force && !force->Data().empty() ?
        static_cast<float>(force->Data().front()) : 0.0f);
    dataPtr->forcePublisher.Publish(message);
    if (dataPtr->legacyForcePublisher)
    {
      gz_std_msgs::Float32 legacy; legacy.set_data(message.data());
      dataPtr->legacyForcePublisher.Publish(legacy);
    }
  }
  if (dataPtr->motorType != MotorType::kVelocity)
    return;

  // The following equations are the ROS 1 RotorS implementation verbatim in
  // physical meaning: thrust is quadratic in the real rotor speed, drag is
  // perpendicular to the rotor axis, and reaction torque alternates by
  // turning direction.
  const int velocitySign = (realMotorVelocity > 0.0) -
      (realMotorVelocity < 0.0);
  const double thrust = dataPtr->turningDirection * velocitySign *
      realMotorVelocity * realMotorVelocity * dataPtr->motorConstant;

  const auto rotorPose = gz::sim::Link(dataPtr->rotorLink).WorldPose(_ecm);
  const auto parentPose = gz::sim::Link(dataPtr->parentLink).WorldPose(_ecm);
  const auto rotorVelocity = gz::sim::Link(dataPtr->rotorLink)
      .WorldLinearVelocity(_ecm);
  if (!rotorPose || !parentPose || !rotorVelocity)
    return;

  // sdformat represents the legacy <use_parent_model_frame> axis as
  // xyz expressed in __model__. Resolve that frame explicitly; retain the
  // rotor-link frame as a fallback for older/non-model axis declarations.
  gz::math::Vector3d axis;
  if (dataPtr->jointAxisResolved &&
      dataPtr->jointAxisExpressedIn == "__model__")
  {
    const auto *modelPose = _ecm.Component<gz::sim::components::WorldPose>(
        dataPtr->modelEntity);
    if (modelPose != nullptr)
      axis = modelPose->Data().Rot().RotateVector(dataPtr->jointAxis);
  }
  if (axis.Length() < 1e-9)
  {
    axis = rotorPose->Rot().RotateVector(
        dataPtr->jointAxisResolved ? dataPtr->jointAxis :
        gz::math::Vector3d(0, 0, 1));
  }
  axis.Normalize();
  const gz::math::Vector3d relativeWind = *rotorVelocity - wind;
  const gz::math::Vector3d perpendicular = relativeWind -
      axis * relativeWind.Dot(axis);
  const gz::math::Vector3d localThrust(0, 0, thrust);
  const gz::math::Vector3d forceWorld = rotorPose->Rot().RotateVector(
      localThrust);
  const gz::math::Vector3d airDrag = -std::abs(realMotorVelocity) *
      dataPtr->rotorDragCoefficient * perpendicular;

  gz::sim::Link rotor(dataPtr->rotorLink);
  rotor.AddWorldForce(_ecm, forceWorld + airDrag);

  const gz::math::Vector3d localReaction(0, 0,
      -dataPtr->turningDirection * thrust * dataPtr->momentConstant);
  const gz::math::Vector3d reactionWorld = rotorPose->Rot().RotateVector(
      localReaction);
  const gz::math::Vector3d rolling = -std::abs(realMotorVelocity) *
      dataPtr->rollingMomentCoefficient * perpendicular;

  gz::sim::Link base(dataPtr->parentLink);
  base.AddWorldWrench(_ecm, gz::math::Vector3d(0, 0, 0),
                      reactionWorld + rolling);

}

}  // namespace rotors_gazebo_plugins

GZ_ADD_PLUGIN(rotors_gazebo_plugins::GazeboMotorModel, gz::sim::System,
  rotors_gazebo_plugins::GazeboMotorModel::ISystemConfigure,
  rotors_gazebo_plugins::GazeboMotorModel::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(rotors_gazebo_plugins::GazeboMotorModel,
                    "GazeboMotorModel",
                    // Gazebo Classic treated the SDF name as an instance
                    // name.  Gazebo Sim resolves it as a plugin alias, so
                    // retain the original model instance names here rather
                    // than changing the model.sdf contract.
                    "tsduav_quad_prop_0_plugin",
                    "tsduav_quad_prop_1_plugin",
                    "tsduav_quad_prop_2_plugin",
                    "tsduav_quad_prop_3_plugin",
                    "tilt_quadcopter_tilt_front_0_plugin",
                    "tilt_quadcopter_tilt_front_1_plugin",
                    "tilt_quadcopter_tilt_front_2_plugin",
                    "tilt_quadcopter_tilt_front_3_plugin",
                    "prop_0_plugin", "prop_1_plugin", "prop_2_plugin", "prop_3_plugin",
                    "prop_4_plugin", "prop_5_plugin", "prop_6_plugin", "prop_7_plugin",
                    "scorpio_coxa_rf_plugin", "scorpio_femur_rf_plugin", "scorpio_tibia_rf_plugin",
                    "scorpio_coxa_rb_plugin", "scorpio_femur_rb_plugin", "scorpio_tibia_rb_plugin",
                    "scorpio_coxa_lb_plugin", "scorpio_femur_lb_plugin", "scorpio_tibia_lb_plugin",
                    "scorpio_coxa_lf_plugin", "scorpio_femur_lf_plugin", "scorpio_tibia_lf_plugin",
                    "scorpio_coxa_rm_plugin", "scorpio_femur_rm_plugin", "scorpio_tibia_rm_plugin",
                    "scorpio_coxa_lm_plugin", "scorpio_femur_lm_plugin", "scorpio_tibia_lm_plugin",
                    "scorpio_prop_0_plugin", "scorpio_prop_1_plugin", "scorpio_prop_2_plugin",
                    "scorpio_tilt_0_plugin", "scorpio_tilt_1_plugin", "scorpio_tilt_2_plugin")
