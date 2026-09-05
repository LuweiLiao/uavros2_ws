"""Bag analysis helpers shared by the original RotorS evaluation scripts.

The ROS 1 implementation used :mod:`rosbag` and ``tf`` directly.  ROS 2 keeps
the public evaluation scripts and data containers unchanged, while this module
adapts bag access to ``rosbag2_py`` and performs the small quaternion
conversion locally.  Keeping the adapter here avoids introducing a second
package or changing the original command-line tools.
"""

from __future__ import division

import copy
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot
import numpy
from scipy import signal
from geometry_msgs.msg import PoseStamped, Quaternion, Point

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:  # Keep utility imports usable outside a sourced ROS 2 shell.
    rosbag2_py = None
    deserialize_message = None
    get_message = None


class _RosTime(object):
    """Small ROS-time compatible wrapper for a rosbag2 nanosecond timestamp."""

    __slots__ = ('nanoseconds',)

    def __init__(self, nanoseconds):
        self.nanoseconds = int(nanoseconds)

    def to_sec(self):
        return self.nanoseconds * 1.0e-9


def _stamp_to_sec(stamp):
    """Return seconds for ROS 1 or ROS 2 time messages."""
    if stamp is None:
        return 0.0
    if hasattr(stamp, 'to_sec'):
        return float(stamp.to_sec())
    sec = getattr(stamp, 'sec', getattr(stamp, 'secs', 0))
    nanosec = getattr(stamp, 'nanosec', getattr(stamp, 'nsec', 0))
    return float(sec) + float(nanosec) * 1.0e-9


def _euler_from_quaternion(quaternion):
    """Convert an ``(x, y, z, w)`` quaternion to roll/pitch/yaw.

    This is the same fixed-axis convention used by
    ``tf.transformations.euler_from_quaternion`` and removes the ROS 1-only
    ``tf`` import.
    """
    x, y, z, w = quaternion
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class _Rosbag2Reader(object):
    """Expose the ROS 1 ``read_messages`` shape over a ROS 2 bag."""

    def __init__(self, bag_path_name):
        if rosbag2_py is None or deserialize_message is None or get_message is None:
            raise RuntimeError(
                'ROS 2 bag support is unavailable; source /opt/ros/jazzy/setup.bash')

        self.path = Path(bag_path_name).expanduser()
        if not self.path.exists():
            raise IOError('Bag path does not exist: %s' % self.path)
        if self.path.suffix.lower() == '.bag':
            raise ValueError(
                'ROS 1 .bag files are not readable by rosbag2_py. Convert the '
                'fixture to a ROS 2 bag first; the original .bag is retained '
                'unchanged for migration parity.')

        if self.path.suffix.lower() == '.db3':
            storage_id = 'sqlite3'
        elif self.path.suffix.lower() == '.mcap':
            storage_id = 'mcap'
        else:
            # A directory containing metadata.yaml is the normal rosbag2 URI;
            # an empty id lets rosbag2 detect sqlite3 or MCAP from metadata.
            storage_id = ''

        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(
            rosbag2_py.StorageOptions(uri=str(self.path), storage_id=storage_id),
            rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'))
        self.topic_types = {
            item.name: item.type
            for item in self.reader.get_all_topics_and_types()
        }
        self._message_classes = {}

    def read_messages(self, topics=None):
        topic_filter = set(topics or [])
        if topic_filter:
            self.reader.set_filter(
                rosbag2_py.StorageFilter(topics=sorted(topic_filter)))

        try:
            while self.reader.has_next():
                topic, serialized, timestamp = self.reader.read_next()
                if topic not in self.topic_types:
                    continue
                type_name = self.topic_types[topic]
                message_class = self._message_classes.get(type_name)
                if message_class is None:
                    message_class = get_message(type_name)
                    self._message_classes[type_name] = message_class
                message = deserialize_message(serialized, message_class)
                yield topic, message, _RosTime(timestamp)
        finally:
            if topic_filter:
                self.reader.reset_filter()


__author__ = "Fadri Furrer, Michael Burri, Markus Achtelik"
__copyright__ = ("Copyright 2015, Fadri Furrer & Michael Burri & "
                 "Markus Achtelik, ASL, ETH Zurich, Switzerland")
__credits__ = ["Fadri Furrer", "Michael Burri", "Markus Achtelik"]
__license__ = "ASL 2.0"
__version__ = "0.1"
__maintainer__ = "Fadri Furrer"
__email__ = "fadri.furrer@mavt.ethz.ch"
__status__ = "Development"


class BaseWithTime(object):

    """Base Class for all objects with time and bag_time."""

    def __init__(self):
        self.time = numpy.array([])
        self.bag_time = numpy.array([])

    def append_times(self, msg_time, bag_time):
        """Append the msg_time and the bag_time."""
        self.time = numpy.append(self.time, msg_time)
        self.bag_time = numpy.append(self.bag_time, bag_time)

    def slice(self, start_time, end_time):
        """Copy the object and slice it between start_time and end_time."""
        start_index = self.get_next_index(start_time)
        end_index = self.get_next_index(end_time)
        copied_obj = copy.deepcopy(self)
        for key in self.__dict__:
            copied_obj.__setattr__(
                key, copied_obj.__getattribute__(key)[start_index:end_index])
        return copied_obj

    def get_next_index(self, time):
        """Get the index of the next value after time."""
        try:
            index = next(idx for idx, val in enumerate(self.time) if val > time)
        except StopIteration:
            index = len(self.time)
        except Exception as exc:
            raise exc
        return index


class ArrayWithTime(BaseWithTime):

    """This class stores arrays or lists and its corresponding times."""

    def __init__(self):
        self.data = numpy.array([])
        BaseWithTime.__init__(self)

    def append_array(self, array):
        """Append an array or list to the data array."""
        try:
            self.data = numpy.append(self.data, [numpy.array(array)], axis=0)
        except ValueError:
            # If the array is empty create one with the first array in it.
            if self.data.size == 0:
                self.data = numpy.array([numpy.array(array)])
            else:
                raise ValueError


class QuatWithTime(BaseWithTime):

    """This class stores quaternions and its corresponding times."""

    def __init__(self):
        self.w = numpy.array([])
        self.x = numpy.array([])
        self.y = numpy.array([])
        self.z = numpy.array([])
        BaseWithTime.__init__(self)

    def append_quaternion(self, quaternion_msg):
        """Append the w, x, y, z components from a quaternion to its arrays."""
        self.w = numpy.append(self.w, quaternion_msg.w)
        self.x = numpy.append(self.x, quaternion_msg.x)
        self.y = numpy.append(self.y, quaternion_msg.y)
        self.z = numpy.append(self.z, quaternion_msg.z)


class XYZWithTime(BaseWithTime):

    """This class stores x,y,z and its corresponding times."""

    def __init__(self):
        self.x = numpy.array([])
        self.y = numpy.array([])
        self.z = numpy.array([])
        BaseWithTime.__init__(self)

    def append_point(self, point_msg):
        """Append the x, y, z components from a point to its arrays."""
        self.x = numpy.append(self.x, point_msg.x)
        self.y = numpy.append(self.y, point_msg.y)
        self.z = numpy.append(self.z, point_msg.z)

    def resample(self, sampling_times):

        [self.x, self.time] = signal.resample(
            self.x,
            len(sampling_times),
            sampling_times)
        [self.y, self.time] = signal.resample(
            self.y,
            len(sampling_times),
            sampling_times)
        [self.z, self.time] = signal.resample(
            self.z,
            len(sampling_times),
            sampling_times)


class WrenchWithTime(BaseWithTime):

    """
    This class stores x,y,z-forces and torques with its corresponding times.
    """

    def __init__(self):
        self.force_x = numpy.array([])
        self.force_y = numpy.array([])
        self.force_z = numpy.array([])
        self.torque_x = numpy.array([])
        self.torque_y = numpy.array([])
        self.torque_z = numpy.array([])
        BaseWithTime.__init__(self)

    def append_wrench(self, wrench_msg):
        """
        Append the x, y, z components for force and torque to their arrays.
        """
        self.force_x = numpy.append(self.force_x, wrench_msg.wrench.force.x)
        self.force_y = numpy.append(self.force_y, wrench_msg.wrench.force.y)
        self.force_z = numpy.append(self.force_z, wrench_msg.wrench.force.z)
        self.torque_x = numpy.append(self.torque_x, wrench_msg.wrench.torque.x)
        self.torque_y = numpy.append(self.torque_y, wrench_msg.wrench.torque.y)
        self.torque_z = numpy.append(self.torque_z, wrench_msg.wrench.torque.z)


class WaypointWithTime(XYZWithTime):

    """This class stores waypoints or and its corresponding times."""

    def __init__(self):
        self.yaw = numpy.array([])
        XYZWithTime.__init__(self)
        self.empty = True

    def append_waypoint(self, waypoint_msg, msg_time, bag_time):
        """Append data from a waypoint to its arrays."""
        position = waypoint_msg.points[0].transforms[0].translation
        rotation = waypoint_msg.points[0].transforms[0].rotation
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        euler = _euler_from_quaternion(quaternion)
        point_msg = Point(x=position.x, y=position.y, z=position.z)
        yaw = euler[2]
        # Check if the waypoint is different from the last one.
        try:
            different_waypoint = (self.yaw[-1] != yaw or
                                  self.x[-1] != point_msg.x or
                                  self.y[-1] != point_msg.y or
                                  self.z[-1] != point_msg.z)
        except IndexError:
            different_waypoint = True
        # Append new waypoint.
        if (different_waypoint):
            self.append_point(point_msg)
            self.yaw = numpy.append(self.yaw, yaw)
            self.append_times(msg_time, bag_time)


class RPYWithTime(BaseWithTime):

    """This class stores roll, pitch, yaw and its corresponding times."""

    def __init__(self):
        self.roll = numpy.array([])
        self.pitch = numpy.array([])
        self.yaw = numpy.array([])
        BaseWithTime.__init__(self)

    def append_quaternion(self, quaternion_msg):
        """Append a roll, pitch, and yaw to each array from a quaternion."""
        quaternion = (
            quaternion_msg.x,
            quaternion_msg.y,
            quaternion_msg.z,
            quaternion_msg.w)
        euler = _euler_from_quaternion(quaternion)
        self.roll = numpy.append(self.roll, euler[0] * 180 / math.pi)
        self.pitch = numpy.append(self.pitch, euler[1] * 180 / math.pi)
        self.yaw = numpy.append(self.yaw, euler[2] * 180 / math.pi)


class AnalyzeBag(object):

    """This class can be used to plot data and compare data."""

    def __init__(self, bag_path_name, save_plots, prefix=None):
        self.bag = _Rosbag2Reader(bag_path_name)
        self.topics = []
        self.pose_topics = []
        self.twist_topics = []
        self.imu_topics = []
        self.motor_velocity_topics = []
        self.waypoint_topics = []
        self.wrench_topics = []
        self.pos = []
        self.quat = []
        self.rpy = []
        self.pqr = []
        self.motor_vel = []
        self.acc = []
        self.ang_vel = []
        self.waypoint = []
        self.wrench = []
        self.save_plots = save_plots
        self.bag_time_start = None
        self.bag_time_end = None
        self.prefix = prefix

    def add_pose_topic(self, topic):
        """Add a pose topic that should be analyzed."""
        self.pos.append(XYZWithTime())
        self.quat.append(QuatWithTime())
        self.rpy.append(RPYWithTime())
        self.pose_topics.append(topic)
        self.topics.append(topic)

    def add_twist_topic(self, topic):
        """Add a twist topic that should be analyzed."""
        self.pqr.append(XYZWithTime())
        self.twist_topics.append(topic)
        self.topics.append(topic)

    def add_imu_topic(self, topic):
        """Add an imu topic that should be analyzed."""
        self.acc.append(XYZWithTime())
        self.ang_vel.append(XYZWithTime())
        self.imu_topics.append(topic)
        self.topics.append(topic)

    def add_motor_velocity_topic(self, topic):
        """Add a motor velocity topic."""
        self.motor_vel.append(ArrayWithTime())
        self.motor_velocity_topics.append(topic)
        self.topics.append(topic)

    def add_waypoint_topic(self, topic):
        """Add a waypoint topic that should be analyzed."""
        self.waypoint.append(WaypointWithTime())
        self.waypoint_topics.append(topic)
        self.topics.append(topic)

    def add_wrench_topic(self, topic):
        """Add a topic of the wrench."""
        self.wrench.append(WrenchWithTime())
        self.wrench_topics.append(topic)
        self.topics.append(topic)

    def extract_messages(self):
        """Run through the bag file and assign the msgs to its attributes."""
        for topic, msg, bag_time in self.bag.read_messages(topics=self.topics):
            if self.bag_time_start is None:
                self.bag_time_start = bag_time
            self.extract_pose_topics(topic, msg, bag_time)
            self.extract_imu_topics(topic, msg, bag_time)
            self.extract_twist_topics(topic, msg, bag_time)
            self.extract_motor_velocity_topics(topic, msg, bag_time)
            self.extract_waypoint_topics(topic, msg, bag_time)
            self.extract_wrench_topics(topic, msg, bag_time)
            self.bag_time_end = bag_time

    def extract_pose_topics(self, topic, msg, bag_time):
        """Append the pose topic msg content to the rpy and pos attributes."""
        msg_time = _stamp_to_sec(msg.header.stamp)

        pose_msg = PoseStamped()
        pose_msg.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
        for index, pose_topic in enumerate(self.pose_topics):
            if topic != pose_topic:
                continue
            elif hasattr(msg, 'pose') and hasattr(msg.pose, 'position'):
                self.pos[index].append_point(msg.pose.position)
                self.pos[index].append_times(msg_time, bag_time)
                self.quat[index].append_quaternion(msg.pose.orientation)
                self.quat[index].append_times(msg_time, bag_time)
                self.rpy[index].append_quaternion(msg.pose.orientation)
                self.rpy[index].append_times(msg_time, bag_time)
            elif hasattr(msg, 'transform') and hasattr(msg.transform, 'translation'):
                self.pos[index].append_point(msg.transform.translation)
                self.pos[index].append_times(msg_time, bag_time)
                self.quat[index].append_quaternion(msg.transform.rotation)
                self.quat[index].append_times(msg_time, bag_time)
                self.rpy[index].append_quaternion(msg.transform.rotation)
                self.rpy[index].append_times(msg_time, bag_time)
            elif hasattr(msg, 'point'):
                self.pos[index].append_point(msg.point)
                self.pos[index].append_times(msg_time, bag_time)
                self.quat[index].append_quaternion(pose_msg.pose.orientation)
                self.quat[index].append_times(msg_time, bag_time)
                self.rpy[index].append_quaternion(pose_msg.pose.orientation)
                self.rpy[index].append_times(msg_time, bag_time)
            else:
                print("Got unknown type: %s" % type(msg))

    def extract_imu_topics(self, topic, msg, bag_time):
        """Append the imu topic msg content to the acc attributes."""
        msg_time = _stamp_to_sec(msg.header.stamp)
        for index, imu_topic in enumerate(self.imu_topics):
            if topic == imu_topic:
                self.acc[index].x.append(msg.linear_acceleration.x)
                self.acc[index].y.append(msg.linear_acceleration.y)
                self.acc[index].z.append(msg.linear_acceleration.z)
                self.acc[index].append_times(msg_time, bag_time)

    def extract_twist_topics(self, topic, msg, bag_time):
        """Append the twist topic msg content to the pqr attributes."""
        msg_time = _stamp_to_sec(msg.header.stamp)
        for index, twist_topic in enumerate(self.twist_topics):
            if topic == twist_topic:
                self.pqr[index].append_point(msg.twist.angular)
                self.pqr[index].append_times(msg_time, bag_time)

    def extract_motor_velocity_topics(self, topic, msg, bag_time):
        """
        Append the motor velocity topic msg content to the motor_vel
        attributes.
        """
        msg_time = _stamp_to_sec(msg.header.stamp)
        for index, motor_velocity_topic in enumerate(self.motor_velocity_topics):
            if topic == motor_velocity_topic:
                # ROS 1 RotorS used mav_msgs/MotorSpeed.motor_speed, while
                # ROS 2 mav_msgs/Actuators calls the equivalent field
                # angular_velocities.  Accept both without changing the
                # public topic or evaluator data structure.
                motor_speed = getattr(msg, 'motor_speed', None)
                if motor_speed is None:
                    motor_speed = getattr(msg, 'angular_velocities', None)
                if motor_speed is None:
                    raise AttributeError(
                        'Motor topic %s has no motor_speed or '
                        'angular_velocities field (type %s)' %
                        (topic, type(msg)))
                self.motor_vel[index].append_array(motor_speed)
                self.motor_vel[index].append_times(msg_time, bag_time)

    def extract_waypoint_topics(self, topic, msg, bag_time):
        """Append the waypoint topic msg to the waypoint attributes."""
        msg_time = _stamp_to_sec(msg.header.stamp)
        for index, waypoint_topic in enumerate(self.waypoint_topics):
            if topic == waypoint_topic:
                self.waypoint[index].append_waypoint(msg, msg_time, bag_time)

    def extract_wrench_topics(self, topic, msg, bag_time):
        """Append the wrench topic msg to the wrench attributes."""
        msg_time = _stamp_to_sec(msg.header.stamp)
        for index, wrench_topic in enumerate(self.wrench_topics):
            if topic == wrench_topic:
                self.wrench[index].append_wrench(msg)
                self.wrench[index].append_times(msg_time, bag_time)

    def plot_positions(self, start_time=None, end_time=None,
                       settling_time=None, x_range=None, y_range=None,
                       plot_suffix=None):
        """Plot all position lists."""
        fig = pyplot.figure()
        fig.suptitle("Position")
        a_x = fig.add_subplot(111)
        for index, pos in enumerate(self.pos):
            # fig.suptitle(self.pose_topics[index])
            a_x.plot(pos.time, pos.x, 'b', label='x')
            a_x.plot(pos.time, pos.y, 'r', label='y')
            a_x.plot(pos.time, pos.z, 'g', label='z')

        if not y_range:
            y_max = max([max(pos.x), max(pos.y), max(pos.z)])
            y_min = min([min(pos.x), min(pos.y), min(pos.z)])
        else:
            y_max = y_range[1]
            y_min = y_range[0]
        y_center = (y_max + y_min)/2.0

        if start_time:
            a_x.axvline(x=start_time, color='c')
            pyplot.text(start_time, y_center, 'start evaluation', rotation=90,
                        color='c')
        if end_time:
            a_x.axvline(x=end_time, color='m')
            pyplot.text(end_time, y_max, 'end evaluation', rotation=90,
                        color='m')
        if settling_time:
            a_x.axvline(x=settling_time, color='k')
            pyplot.text(settling_time, y_max, 'settled, start RMS eval',
                        rotation=90, color='k')

        pyplot.xlabel('time [s]')
        pyplot.ylabel('position [m]')
        # Shrink current axis's height by 10% on the bottom
        box = a_x.get_position()
        a_x.set_position([box.x0, box.y0 + box.height * 0.2,
                         box.width, box.height * 0.8])

        # Put a legend below current axis
        a_x.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15),
                   fancybox=True, shadow=True, ncol=5)
        pyplot.xlim(x_range)
        pyplot.ylim(y_range)
        pyplot.grid(visible=True, which='both')

        if self.save_plots:
            file_name = self.prefix + '_pos' if self.prefix else 'pos'
            if plot_suffix:
                file_name += '_' + str(plot_suffix)
            file_name += '.png'
            pyplot.savefig(file_name)

    def plot_position_error(self, set_point, settling_radius=None,
                            start_time=None, end_time=None, settling_time=None,
                            x_range=None, y_range=None, plot_suffix=None):
        """Plot all position errors lists."""
        fig = pyplot.figure()
        fig.suptitle("Position error")
        a_x = fig.add_subplot(111)
        pos_errors = numpy.array([])

        for index, pos in enumerate(self.pos):
            x_error = (pos.x - set_point.x[0])
            y_error = (pos.y - set_point.y[0])
            z_error = (pos.z - set_point.z[0])

            pos_errors = numpy.append(pos_errors,
                                      (x_error ** 2 + y_error ** 2
                                       + z_error ** 2) ** 0.5)
            a_x.plot(pos.time, pos_errors, 'b', label='pos_error')

        if not y_range:
            y_max = max(pos_errors)
            y_min = 0
        else:
            y_max = y_range[1]
            y_min = y_range[0]
        y_center = (y_max + y_min)/2.0

        if settling_radius:
            a_x.axhline(y=settling_radius, color='r', xmin=0, xmax=1)
            pyplot.text(start_time, settling_radius, 'settling radius',
                        color='r')
        if start_time:
            a_x.axvline(x=start_time, color='c')
            pyplot.text(start_time, y_center, 'start evaluation', rotation=90,
                        color='c')
        if end_time:
            a_x.axvline(x=end_time, color='m')
            pyplot.text(end_time, y_max, 'end evaluation', rotation=90,
                        color='m')
        if settling_time:
            a_x.axvline(x=settling_time, color='k')
            pyplot.text(settling_time, y_max, 'settled, start RMS eval',
                        rotation=90, color='k')

        pyplot.xlabel('time [s]')
        pyplot.ylabel('position error [m]')
        # Shrink current axis's height by 10% on the bottom
        box = a_x.get_position()
        a_x.set_position([box.x0, box.y0 + box.height * 0.2,
                         box.width, box.height * 0.8])

        # Put a legend below current axis
        a_x.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15),
                   fancybox=True, shadow=True, ncol=5)
        pyplot.xlim(x_range)
        pyplot.ylim(y_range)
        pyplot.grid(visible=True, which='both')

        if self.save_plots:
            file_name = self.prefix + '_pos_error' if self.prefix else 'pos_error'
            if plot_suffix:
                file_name += '_' + str(plot_suffix)
            file_name += '.png'
            pyplot.savefig(file_name)

        # pyplot.show()

    def plot_angular_velocities(self, start_time=None, end_time=None,
                                settling_time=None, x_range=None, y_range=None,
                                plot_suffix=None):
        """Plot all angular_velocity lists."""
        fig = pyplot.figure()
        fig.suptitle("Angular Velocity")
        a_x = fig.add_subplot(111)
        for index, pqr in enumerate(self.pqr):
            a_x.plot(pqr.time, pqr.x, 'b', label='x')
            a_x.plot(pqr.time, pqr.y, 'r', label='y')
            a_x.plot(pqr.time, pqr.z, 'g', label='z')

        if not y_range:
            y_max = max([max(pqr.x), max(pqr.y), max(pqr.z)])
            y_min = min([min(pqr.x), min(pqr.y), min(pqr.z)])
        else:
            y_max = y_range[1]
            y_min = y_range[0]
        y_center = (y_max + y_min)/2.0

        if start_time:
            a_x.axvline(x=start_time, color='c')
            pyplot.text(start_time, y_center, 'start evaluation', rotation=90,
                        color='c')
        if end_time:
            a_x.axvline(x=end_time, color='m')
            pyplot.text(end_time, y_max, 'end evaluation', rotation=90,
                        color='m')
        if settling_time:
            a_x.axvline(x=settling_time, color='k')
            pyplot.text(settling_time, y_max, 'settled, start RMS eval',
                        rotation=90, color='k')

        pyplot.xlabel('time [s]')
        pyplot.ylabel('angular velocity [rad/s]')
        # Shrink current axis's height by 10% on the bottom
        box = a_x.get_position()
        a_x.set_position([box.x0, box.y0 + box.height * 0.2,
                         box.width, box.height * 0.8])

        # Put a legend below current axis
        a_x.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15),
                   fancybox=True, shadow=True, ncol=5)
        pyplot.xlim(x_range)
        pyplot.ylim(y_range)
        pyplot.grid(visible=True, which='both')

        if self.save_plots:
            file_name = self.prefix + '_angular_velocity' if self.prefix else 'angular_velocity'
            if plot_suffix:
                file_name += '_' + str(plot_suffix)
            file_name += '.png'
            pyplot.savefig(file_name)

        # pyplot.show()

    def plot_3d_trajectories(self):
        fig = pyplot.figure()
        a_x = fig.add_subplot(111, projection='3d')
        for index, pos in enumerate(self.pos):
            a_x.plot(pos.x, pos.y, pos.z, label=self.pose_topics[index])
        # pyplot.show()

    def plot_rpys(self, start_time=None, end_time=None, settling_time=None,
                  plot_suffix=None):
        """Plot rpy lists."""
        fig = pyplot.figure()
        fig.suptitle("RPY")
        a_x = fig.add_subplot(111)
        for index, rpy in enumerate(self.rpy):
            a_x.plot(rpy.time, rpy.roll, 'b',
                     label='roll' + self.pose_topics[index])
            a_x.plot(rpy.time, rpy.pitch, 'r',
                     label='pitch' + self.pose_topics[index])
            a_x.plot(rpy.time, rpy.yaw, 'g',
                     label='yaw' + self.pose_topics[index])

        if start_time:
            a_x.axvline(x=start_time, color='c')
        if end_time:
            a_x.axvline(x=end_time, color='m')
        if settling_time:
            a_x.axvline(x=settling_time, color='k')

        pyplot.xlabel('time [s]')
        pyplot.ylabel('angle [deg]')
        pyplot.legend()

        if self.save_plots:
            file_name = self.prefix + '_rpy' if self.prefix else 'rpy'
            if plot_suffix:
                file_name += '_' + str(plot_suffix)
            file_name += '.png'
            pyplot.savefig(file_name)

        # pyplot.show()

    def plot_accelerations(self, plot_suffix=None):
        """Plot all acceleration lists."""
        fig = pyplot.figure()
        fig.suptitle("Accelerations")
        a_x = fig.add_subplot(111)
        for index, acc in enumerate(self.acc):
            a_x.plot(acc.time, acc.x, 'b', label='x' + self.imu_topics[index])
            a_x.plot(acc.time, acc.y, 'r', label='y' + self.imu_topics[index])
            a_x.plot(acc.time, acc.z, 'g', label='z' + self.imu_topics[index])
        pyplot.xlabel('time [s]')
        pyplot.ylabel('acceleration [m/s^2]')
        pyplot.legend()
        if self.save_plots:
            file_name = self.prefix + '_acc' if self.prefix else 'acc'
            if plot_suffix:
                file_name += '_' + str(plot_suffix)
            file_name += '.png'
            pyplot.savefig(file_name)
        # pyplot.show()

    def compare_positions(self, pose_indeces):
        """
        Compare the position lists of pose_topics.

        Args:
           pose_indeces (list): Indeces of pose_topics for comparison.
        """
        if len(pose_indeces) < 2:
            print("At least two pose_indeces need to be provided to make a "
                  "comparison")
        for index, pose_index in enumerate(pose_indeces):
            for pose_index_cmp in pose_indeces[index+1:]:
                compare_two_xyz(self.pos[pose_index],
                                self.pos[pose_index_cmp])

    def get_collisions(self, start_time=None, end_time=None):
        """Get the collision times."""
        collision_times = []

        for collision in self.wrench:
            for t in collision.time:
                if t >= (start_time or 0) and t <= (end_time or
                                                    max(collision.time)):
                    collision_times.append(t)
        return collision_times


def compare_two_xyz(xyz_one, xyz_two):
    # TODO(ff): Implement some position comparison
    pass


def xyz_rms_error(set_point, input_series):
    """
    Calculate the RMS error between a set_point and the points in input_series.

    Args:
        set_point: XYZWithTime of the reference point
        input_series (list): XYZWithTime of the points that should be evaluated

    Returns:
        rms: Float of the RMS Error of the XYZWithTime input_series
    """
    sum_of_squares = 0.0
    elems = len(input_series.x)
    for index in range(elems):
        x_error = (input_series.x[index] - set_point.x)
        y_error = (input_series.y[index] - set_point.y)
        z_error = (input_series.z[index] - set_point.z)
        sum_of_squares += (1.0 / elems *
                           (x_error ** 2 + y_error ** 2 + z_error ** 2))
    rms = sum_of_squares ** 0.5
    return rms


def settling_time(set_point, input_series, bounding_radius, min_time):
    """
    Calculate the settling time of input_series to be bounded for min_time.

    Args:
        set_point: XYZWithTime of the reference point
        input_series (list): XYZWithTime of the points that should be evaluated
        bounding_radius: Float of the radius of the bounding sphere
        min_time: Float of the time, the input_series has to stay within bounds

    Returns:
        settling_time: Float of the time for input_series to be settled
    """
    bounded_time = False
    elems = len(input_series.x)
    for index in range(elems):
        current_time = input_series.time[index]
        x_error = (input_series.x[index] - set_point.x)
        y_error = (input_series.y[index] - set_point.y)
        z_error = (input_series.z[index] - set_point.z)
        error_radius = (x_error ** 2 + y_error ** 2 + z_error ** 2) ** 0.5
        if error_radius <= bounding_radius and not bounded_time:
            bounded_time = current_time
        elif (error_radius <= bounding_radius and bounded_time and
              current_time - bounded_time >= min_time):
            return bounded_time - input_series.time[0]
        elif (not bounded_time or error_radius <= bounding_radius and
              bounded_time):
            continue
        else:
            bounded_time = False
    return None


def work(input_series, work_constant):
    """
    Calculate the work of input_series.

    Args:
        input_series (list): ArrayWithTime of the motor velocities
        work_constant: Float of the (rotor_constant * moment_constant) constant

    Returns:
        total_work: Float of the work done in input_series
    """
    total_work = 0

    elems = len(input_series.time)
    for index in range(elems - 1):
        current_time = input_series.time[index]
        next_time = input_series.time[index + 1]
        delta_time = next_time - current_time
        for value in input_series.data[index]:
            total_work += abs(value) ** 3 * work_constant * delta_time
    return total_work


def get_errors_pos_only(xyz_one, quat_one, xyz_two, restart_distance,
                        alignment_length, t_vs=numpy.eye(4), time_offset=0):
    # TODO(ff): Implement this function
    xyz_two_time_shifted = copy.deepcopy(xyz_two)
    xyz_two_time_shifted.time -= time_offset
    xyz_two_time_shifted.resample(xyz_one.time)
    xyz_one_time_shifted = xyz_one
    xyz_one_time_shifted.time -= time_offset
    xyz_one_time_shifted.resample(xyz_two.time)
    xyz_two = xyz_two_time_shifted


def create_topic_list(topics_string):
    if topics_string and ',' in topics_string:
        return topics_string.split(',')
    elif topics_string:
        return [topics_string]
    else:
        return []


def create_set_point(x, y, z):
    """Create a XYZWithTime set point."""
    point_msg = Point(x=x, y=y, z=z)
    set_point = XYZWithTime()
    set_point.append_point(point_msg)
    return set_point
