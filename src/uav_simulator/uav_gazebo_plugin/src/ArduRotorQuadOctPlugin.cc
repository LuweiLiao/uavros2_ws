/*
 * Gazebo Sim loader port for the ROS 1 ArduRotorQuadOctPlugin target.
 *
 * The ROS 1 source at this exact path intentionally includes
 * ArduRotorNormPlugin.hh and registers the ArduRotorNormPlugin class.  Its
 * only behavioral difference from the normal target is the fixed eight-slot
 * actuator publication; the ROS 2 ArduRotorNormPlugin already preserves that
 * behavior whenever the original SDF supplies motor_num=8.  Keep the
 * original library filename and expose the original class through a loader
 * alias rather than inventing a second dynamics implementation.
 */

#include "ArduRotorNormPlugin.hh"

#include <gz/plugin/Register.hh>

// The SDFs and ROS 1 build use libArduRotorQuadOctPlugin.so.  Registering the
// historical target name as an alias preserves that contract while sharing
// the already-ported ArduRotorNormPlugin implementation.
GZ_ADD_PLUGIN_ALIAS(gazebo::ArduRotorNormPlugin, "ArduRotorQuadOctPlugin")
