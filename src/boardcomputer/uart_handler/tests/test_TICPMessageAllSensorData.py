from unittest import TestCase

from uart_handler.ticp_message import TICPMessageAllSensorData
from uart_handler.ticp_message import _TICPMessageDecoder


class TestTICPMessageAllSensorData(TestCase):

    def test_from_sensor_values(self):
        msg = TICPMessageAllSensorData.from_sensor_values(255, 0, 32767, 0, 32767, 127, 255)

        expected_payload = _TICPMessageDecoder.convert_from_tof_to_byte(255)
        expected_payload += _TICPMessageDecoder.convert_from_tof_to_byte(0)
        expected_payload += _TICPMessageDecoder.convert_from_accel_to_bytes(32767)
        expected_payload += _TICPMessageDecoder.convert_from_accel_to_bytes(0)
        expected_payload += _TICPMessageDecoder.convert_from_speed_to_bytes(32767)
        expected_payload += _TICPMessageDecoder.convert_from_servo_to_bytes(127)
        expected_payload += _TICPMessageDecoder.convert_from_fsm_to_bytes(255)

        self.assertEqual(expected_payload, msg._payload)

    def test_tof1(self):
        msg = TICPMessageAllSensorData.from_sensor_values(255, 0, 32767, 0, 32767, 127, 255)
        self.assertEqual(255, msg.tof1)

    def test_tof2(self):
        msg = TICPMessageAllSensorData.from_sensor_values(255, 0, 32767, 0, 32767, 127, 255)
        self.assertEqual(0, msg.tof2)

    def test_accel_x(self):
        msg = TICPMessageAllSensorData.from_sensor_values(tof1_data=10,
                                                          tof2_data=20,
                                                          accelx_data=30,
                                                          accely_data=40,
                                                          speed_data=50,
                                                          servo1_data=60,
                                                          fsm_state=70)
        self.assertEqual(30, msg.accel_x)

    def test_accel_y(self):
        msg = TICPMessageAllSensorData.from_sensor_values(tof1_data=10,
                                                          tof2_data=20,
                                                          accelx_data=30,
                                                          accely_data=40,
                                                          speed_data=50,
                                                          servo1_data=60,
                                                          fsm_state=70)
        self.assertEqual(40, msg.accel_y)

    def test_speed(self):
        msg = TICPMessageAllSensorData.from_sensor_values(tof1_data=10,
                                                          tof2_data=20,
                                                          accelx_data=30,
                                                          accely_data=40,
                                                          speed_data=50,
                                                          servo1_data=60,
                                                          fsm_state=70)
        self.assertEqual(50, msg.speed)

    def test_servo(self):
        msg = TICPMessageAllSensorData.from_sensor_values(tof1_data=10,
                                                          tof2_data=20,
                                                          accelx_data=30,
                                                          accely_data=40,
                                                          speed_data=50,
                                                          servo1_data=60,
                                                          fsm_state=70)
        self.assertEqual(60, msg.servo)

    def test_fsm_state(self):
        msg = TICPMessageAllSensorData.from_sensor_values(tof1_data=10,
                                                          tof2_data=20,
                                                          accelx_data=30,
                                                          accely_data=40,
                                                          speed_data=50,
                                                          servo1_data=60,
                                                          fsm_state=70)
        self.assertEqual(70, msg.fsm_state)

