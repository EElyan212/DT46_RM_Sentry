"""Exercise the merged serial protocol without opening hardware or ROS nodes."""

import struct
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from builtin_interfaces.msg import Time
from rm_interfaces.msg import GimbalControl, Senddata

from rm_application.modules.crc import crc16_modbus, verify_crc16_check_sum
from rm_application.serial_node import SerialNode


def _receive_frame(sequence, valid_crc=True):
    payload = struct.pack(
        '<BBBfffffHHHHHBBfBBBBff',
        0xA5, 1, 0,
        0.1, 0.2, 0.3, 0.4, 0.5,
        321, 222, 111, 120, 99,
        4, 1, 28.0,
        2, 1, 3, sequence, 0.1, 0.2,
    )
    checksum = crc16_modbus(payload)
    if not valid_crc:
        checksum ^= 1
    return payload + struct.pack('<H', checksum)


@pytest.mark.parametrize('mode, decision_count', [('mock', 0), ('serial', 2)])
def test_receive_keeps_input_mode_and_publishes_each_event_once(mode, decision_count):
    """New protocol support must preserve the existing decision input switch."""
    frames = [_receive_frame(7), _receive_frame(8)]
    clock = Mock()
    clock.now.return_value.to_msg.return_value = Time()
    node = SimpleNamespace(
        serial=Mock(), send_datas=Senddata(),
        serial_receive_header=0xA5,
        decision_input_mode=mode, pub_rpy=True, pub_serial_debug=True,
        get_logger=Mock(return_value=Mock()),
        get_clock=Mock(return_value=clock),
        pub_uart_receive_decision=Mock(), pub_uart_receive_imu=Mock(),
        pub_uart_receive_odo=Mock(), pub_armor_hit=Mock(),
        pub_serial_debug_publisher=Mock(), reopen_port=Mock(),
    )
    node.serial.read.side_effect = [chunk for frame in frames for chunk in (frame[:1], frame[1:])]
    with patch('rm_application.serial_node.rclpy.ok', side_effect=[True, True, False]):
        SerialNode.receive_data(node)

    assert node.pub_uart_receive_decision.publish.call_count == decision_count
    assert node.pub_uart_receive_odo.publish.call_count == 2
    assert node.pub_uart_receive_imu.publish.call_count == 2
    node.pub_armor_hit.publish.assert_called_once()
    assert node.pub_armor_hit.publish.call_args.args[0].armor_hit_seq == 8
    assert node.pub_serial_debug_publisher.publish.call_count == 2
    debug = node.pub_serial_debug_publisher.publish.call_args.args[0]
    assert debug.self_heavy_hp == 222
    assert debug.crc_ok is True
    if decision_count:
        decision = node.pub_uart_receive_decision.publish.call_args.args[0]
        assert decision.self_sentry_hp == 321
        assert decision.self_heavy_hp == 222
    node.reopen_port.assert_not_called()


def test_receive_discards_bad_crc():
    """A corrupted packet must not reach navigation or decision consumers."""
    frame = _receive_frame(7, valid_crc=False)
    node = SimpleNamespace(
        serial=Mock(), serial_receive_header=0xA5, pub_serial_debug=False,
        get_logger=Mock(return_value=Mock()), pub_uart_receive_decision=Mock(),
        pub_uart_receive_odo=Mock(), reopen_port=Mock(),
    )
    node.serial.read.side_effect = [frame[:1], frame[1:]]
    with patch('rm_application.serial_node.rclpy.ok', side_effect=[True, False]):
        SerialNode.receive_data(node)
    node.pub_uart_receive_decision.publish.assert_not_called()
    node.pub_uart_receive_odo.publish.assert_not_called()
    node.reopen_port.assert_not_called()


@pytest.mark.parametrize('last_gimbal_time, expected_fire, expected_rate', [
    (99.95, 1, 1.25),
    (99.84, 0, 0.0),
    (0.0, 0, 0.0),
])
def test_send_new_fields_and_expire_stale_gimbal_data(last_gimbal_time, expected_fire, expected_rate):
    """Check the on-wire layout and timeout while preserving chassis commands."""
    node = SimpleNamespace(
        lock=Lock(), is_reconnecting=False, serial=Mock(),
        send_datas=Senddata(
            linear_velocity_x=0.5, linear_velocity_y=-0.25,
            gimbal_mode=2, yaw=3.0, pitch=4.0, can_fire=1,
            yaw_rate=1.25, cliss_mode=0, gimbal_speed=10,
        ),
        last_gimbal_time=last_gimbal_time, serial_send_header=0x5A,
        get_logger=Mock(return_value=Mock()),
    )
    with patch('rm_application.serial_node.time.monotonic', return_value=100.0):
        SerialNode.Send(node)
    node.serial.write.assert_called_once()
    packet = node.serial.write.call_args.args[0]
    assert len(packet) == 39  # 37-byte payload and two-byte CRC.
    assert verify_crc16_check_sum(packet)
    assert struct.unpack('<Bffiffifii', packet[:-2]) == (
        0x5A, 0.5, -0.25, 2, 3.0, 4.0,
        expected_fire, expected_rate, 0, 10,
    )
    assert node.send_datas.can_fire == 1
    assert node.send_datas.yaw_rate == 1.25


def test_gimbal_callback_records_feedforward_and_freshness():
    """A fresh visual command supplies both feedforward and its receive time."""
    node = SimpleNamespace(lock=Lock(), send_datas=Senddata(), last_gimbal_time=0.0)
    msg = GimbalControl(yaw=3.0, pitch=4.0, can_fire=1, yaw_rate=1.25)
    with patch('rm_application.serial_node.time.monotonic', return_value=100.0):
        SerialNode.gimbal_control_callback(node, msg)
    assert node.send_datas.yaw_rate == 1.25
    assert node.send_datas.yaw == 3.0
    assert node.send_datas.pitch == 4.0
    assert node.send_datas.can_fire == 1
    assert node.last_gimbal_time == 100.0


def test_modbus_crc_reference_vector():
    """Use the standard MODBUS check vector rather than only round-trip CRCs."""
    assert crc16_modbus(b'123456789') == 0x4B37
