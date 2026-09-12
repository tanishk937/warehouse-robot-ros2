#!/usr/bin/env python3
# Copyright 2026 Tanishk Patidar
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Configuration-driven sensor fault-injection topic gate.

Config-driven topic-gate fault injector (see docs/AUDIT.md #9 /
Section 17 of the upgrade brief: "do not require manually killing
random processes for every demonstration").

For each configured sensor, this node subscribes to its raw Gazebo
topic (e.g. /scan_raw) and republishes onto the public topic that the
rest of the system actually watches (e.g. /scan). While a fault is
injected for that sensor, republishing simply stops -- every consumer
(sensor_heartbeat_monitor, safety_controller, localization_monitor)
experiences this exactly like a real sensor failure: the topic goes
silent and their existing, independent staleness-detection logic
handles it. No duplicate "is this a fault or a real failure" branching
exists anywhere downstream, which is deliberate -- a fault-injector
that consumers can tell apart from a real failure would not actually
be testing the real failure path.

Adding a new gateable sensor requires only a new entry in
config/fault_injector.yaml, following the same
config-driven-scalability pattern as sensor_heartbeat_monitor.

Publishes:
    <public_topic> for each configured sensor (message type per config)
Subscribes:
    <raw_topic> for each configured sensor (message type per config)
Services:
    /fault/set_sensor_fault   (warehouse_robot_msgs/srv/SetSensorFault)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from warehouse_robot_msgs.srv import SetSensorFault


class _GatedSensor:

    def __init__(self, name, raw_topic, public_topic, msg_type_str, qos_kind):
        self.name = name
        self.raw_topic = raw_topic
        self.public_topic = public_topic
        self.msg_type_str = msg_type_str
        self.qos_kind = qos_kind
        self.fault_enabled = False
        self.publisher = None
        self.subscription = None
        self.messages_dropped = 0
        self.messages_passed = 0


class SensorFaultInjector(Node):

    def __init__(self):
        super().__init__('sensor_fault_injector')

        self.declare_parameter('sensor_names', [''])
        sensor_names = self.get_parameter('sensor_names').get_parameter_value().string_array_value
        sensor_names = [n for n in sensor_names if n]

        self._gates = {}

        for name in sensor_names:
            self.declare_parameter(f'{name}.raw_topic', '')
            self.declare_parameter(f'{name}.public_topic', '')
            self.declare_parameter(f'{name}.msg_type', 'sensor_msgs/msg/LaserScan')
            self.declare_parameter(f'{name}.qos', 'best_effort')

            raw_topic = self.get_parameter(f'{name}.raw_topic').value
            public_topic = self.get_parameter(f'{name}.public_topic').value
            msg_type_str = self.get_parameter(f'{name}.msg_type').value
            qos_kind = self.get_parameter(f'{name}.qos').value

            if not raw_topic or not public_topic:
                self.get_logger().warn(
                    f"Fault gate '{name}' missing raw_topic/public_topic, skipping."
                )
                continue

            try:
                msg_class = get_message(msg_type_str)
            except (ValueError, ImportError, AttributeError) as exc:
                self.get_logger().error(
                    f"Fault gate '{name}': could not resolve message type "
                    f"'{msg_type_str}' ({exc}), skipping."
                )
                continue

            if qos_kind == 'reliable':
                qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                )
            else:
                qos = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                )

            gate = _GatedSensor(name, raw_topic, public_topic, msg_type_str, qos_kind)
            gate.publisher = self.create_publisher(msg_class, public_topic, qos)

            def _make_callback(gate_ref):
                def _cb(msg):
                    if gate_ref.fault_enabled:
                        gate_ref.messages_dropped += 1
                        return
                    gate_ref.messages_passed += 1
                    gate_ref.publisher.publish(msg)
                return _cb

            gate.subscription = self.create_subscription(
                msg_class, raw_topic, _make_callback(gate), qos
            )
            self._gates[name] = gate
            self.get_logger().info(f"Gating sensor '{name}': {raw_topic} -> {public_topic}")

        self.create_service(SetSensorFault, '/fault/set_sensor_fault', self._on_set_fault)

    def _on_set_fault(self, request: SetSensorFault.Request, response: SetSensorFault.Response):
        gate = self._gates.get(request.sensor_name)
        if gate is None:
            response.success = False
            response.message = (
                f"Unknown sensor '{request.sensor_name}'. Configured: {list(self._gates.keys())}"
            )
            return response

        gate.fault_enabled = request.fault_enabled
        level = (
            'INJECTED (topic gated off)'
            if request.fault_enabled
            else 'CLEARED (passthrough resumed)'
        )
        self.get_logger().warn(f"[FAULT] Sensor '{gate.name}' fault {level}")
        response.success = True
        response.message = f"Sensor '{gate.name}' fault_enabled={request.fault_enabled}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SensorFaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
