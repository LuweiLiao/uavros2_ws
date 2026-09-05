#!/usr/bin/env python3

import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QFormLayout, QWidget
from rqt_gui_py.plugin import Plugin


class HilPlugin(Plugin):
    # MAV mode flags
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64
    MAV_MODE_FLAG_HIL_ENABLED = 32
    MAV_MODE_FLAG_STABILIZE_ENABLED = 16
    MAV_MODE_FLAG_GUIDED_ENABLED = 8
    MAV_MODE_FLAG_AUTO_ENABLED = 4
    MAV_MODE_FLAG_TEST_ENABLED = 2
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

    # MAV state dictionary
    mav_state = {
        0: 'Uninitialized',
        1: 'Booting up',
        2: 'Calibrating',
        3: 'Standby',
        4: 'Active',
        5: 'Critical',
        6: 'Emergency',
        7: 'Poweroff',
    }

    # Constants
    STR_ON = 'ON'
    STR_OFF = 'OFF'
    STR_UNKNOWN = 'N/A'

    STR_MAVROS_ARM_SERVICE_NAME = '/mavros/cmd/arming'
    STR_MAVROS_COMMAND_LONG_SERVICE_NAME = '/mavros/cmd/command'
    STR_MAVROS_SET_MODE_SERVICE_NAME = '/mavros/set_mode'

    STR_SYS_STATUS_SUB_TOPIC = '/mavros/state'

    TIMEOUT_HIL_HEARTBEAT = 2.0

    def __init__(self, context):
        super(HilPlugin, self).__init__(context)
        self.setObjectName('HilPlugin')

        # rqt_gui_py creates and spins one shared ROS 2 node for all Python
        # plugins.  Use that node rather than creating a second context.
        self.node = context.node

        self._widget = QWidget()
        ui_file = os.path.join(
            get_package_share_directory('rqt_rotors'),
            'resource',
            'HilPlugin.ui',
        )
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('HilPluginUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() +
                (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        # Set the initial parameters of UI elements.
        self._widget.button_set_hil_mode.setEnabled(False)
        self._widget.button_arm.setEnabled(False)
        self._widget.button_reboot_autopilot.setEnabled(False)
        self._widget.text_state.setText(self.STR_UNKNOWN)
        self.clear_mav_mode()

        # Initialize class variables.
        self.last_heartbeat_time = time.time()
        self.mav_mode = 65
        self.mav_status = 255
        self.armed = False
        self.connected = False
        self.guided = False
        self.hil_enabled = False

        # ROS callbacks run on rqt_gui_py's executor thread.  Keep their
        # message snapshot separate and update Qt widgets on the Qt thread.
        self._state_lock = threading.Lock()
        self._latest_state = None
        self._ui_timer = QTimer(self._widget)
        self._ui_timer.timeout.connect(self._update_ui_from_state)
        self._ui_timer.start(100)

        # Set the functions that are called when signals are emitted.
        self._widget.button_set_hil_mode.pressed.connect(
            self.on_set_hil_mode_button_pressed)
        self._widget.button_arm.pressed.connect(
            self.on_arm_button_pressed)
        self._widget.button_reboot_autopilot.pressed.connect(
            self.on_reboot_autopilot_button_pressed)

        # Create ROS 2 service clients.  The public service names are
        # unchanged from the ROS 1 plugin.
        self.arm = self.node.create_client(
            CommandBool, self.STR_MAVROS_ARM_SERVICE_NAME)
        self.send_command_long = self.node.create_client(
            CommandLong, self.STR_MAVROS_COMMAND_LONG_SERVICE_NAME)
        self.set_mode = self.node.create_client(
            SetMode, self.STR_MAVROS_SET_MODE_SERVICE_NAME)

        # Initialize the ROS 2 subscriber.  rqt_gui_py supplies the executor
        # that dispatches this callback.
        self.sys_status_sub = self.node.create_subscription(
            State,
            self.STR_SYS_STATUS_SUB_TOPIC,
            self.sys_status_callback,
            1,
        )

    def _call_service(self, client, request):
        """Send a non-blocking ROS 2 service request when the server exists."""
        if not client.service_is_ready():
            self.node.get_logger().warning(
                'HIL plugin service is not available: %s' %
                client.srv_name)
            return None
        try:
            return client.call_async(request)
        except RuntimeError as exc:
            self.node.get_logger().warning(
                'HIL plugin service request failed: %s' % exc)
            return None

    def _request_hil_mode(self):
        new_mode = self.mav_mode | self.MAV_MODE_FLAG_HIL_ENABLED
        self.hil_enabled = True
        self.mav_mode = new_mode

        request = SetMode.Request()
        request.base_mode = new_mode
        request.custom_mode = ''
        self._call_service(self.set_mode, request)
        self._widget.text_mode_hil.setText(
            self.mav_mode_text(self.hil_enabled))

    def on_set_hil_mode_button_pressed(self):
        self._request_hil_mode()

    def on_arm_button_pressed(self):
        request = CommandBool.Request()
        request.value = True
        self._call_service(self.arm, request)

    def on_reboot_autopilot_button_pressed(self):
        request = CommandLong.Request()
        request.broadcast = False
        request.command = 246
        request.confirmation = 1
        request.param1 = 1.0
        request.param2 = 1.0
        request.param3 = 0.0
        request.param4 = 0.0
        request.param5 = 0.0
        request.param6 = 0.0
        request.param7 = 0.0
        self._call_service(self.send_command_long, request)

    def sys_status_callback(self, msg):
        """Store the state message; Qt updates happen in the UI timer."""
        with self._state_lock:
            self._latest_state = {
                'connected': bool(msg.connected),
                'armed': bool(msg.armed),
                'guided': bool(msg.guided),
                'manual_input': bool(msg.manual_input),
                'mode': msg.mode,
                'system_status': int(msg.system_status),
            }

    def _update_ui_from_state(self):
        with self._state_lock:
            state = self._latest_state
        if state is None:
            return

        if not self.connected and state['connected']:
            self._widget.button_set_hil_mode.setEnabled(True)
            self._widget.button_arm.setEnabled(True)
            self._widget.button_reboot_autopilot.setEnabled(True)
            self.connected = True
            self.last_heartbeat_time = time.time()
            self.armed = state['armed']
            self.guided = state['guided']
            self._widget.text_mode_safety_armed.setText(
                self.mav_mode_text(self.armed))
            self._widget.text_mode_guided.setText(
                self.mav_mode_text(self.guided))
            self._widget.text_state.setText(
                self.mav_state.get(state['system_status'], self.STR_UNKNOWN))
            return

        if ((time.time() - self.last_heartbeat_time) >=
                self.TIMEOUT_HIL_HEARTBEAT and self.hil_enabled):
            self._request_hil_mode()

        if self.armed != state['armed']:
            self.armed = state['armed']
            self._widget.text_mode_safety_armed.setText(
                self.mav_mode_text(self.armed))
            self._widget.button_arm.setEnabled(not self.armed)
            # Keep the original mode-bit update contract.
            self.mav_mode = self.mav_mode | self.MAV_MODE_FLAG_SAFETY_ARMED

        if self.guided != state['guided']:
            self.guided = state['guided']
            self._widget.text_mode_guided.setText(
                self.mav_mode_text(self.guided))

        self._widget.text_state.setText(
            self.mav_state.get(state['system_status'], self.STR_UNKNOWN))
        self.last_heartbeat_time = time.time()

    def clear_mav_mode(self):
        count = self._widget.mav_mode_layout.rowCount()
        for i in range(count):
            item = self._widget.mav_mode_layout.itemAt(
                i, QFormLayout.FieldRole)
            if item is not None and item.widget() is not None:
                item.widget().setText(self.STR_UNKNOWN)

    def mav_mode_text(self, mode_enabled):
        return self.STR_ON if mode_enabled else self.STR_OFF

    def shutdown_plugin(self):
        if self._ui_timer is not None:
            self._ui_timer.stop()
            self._ui_timer.deleteLater()
            self._ui_timer = None

        if self.sys_status_sub is not None:
            self.node.destroy_subscription(self.sys_status_sub)
            self.sys_status_sub = None

        for client_name in ('arm', 'send_command_long', 'set_mode'):
            client = getattr(self, client_name, None)
            if client is not None:
                self.node.destroy_client(client)
                setattr(self, client_name, None)
