#!/usr/bin/env python3
# Copyright 2026 Tanishk Patidar
#
# This source code is provided for viewing, evaluation, educational,
# and portfolio purposes. All rights reserved.
#
# See the repository LICENSE file for terms governing copying,
# modification, distribution, and commercial use.

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
