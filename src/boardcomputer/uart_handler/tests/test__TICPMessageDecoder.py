from unittest import TestCase

from uart_handler.ticp_message import _TICPMessageDecoder


class Test_TICPMessageDecoder(TestCase):

    def test_convert_from_tof_to_byte(self):
        tof = 255
        expected_result = bytes.fromhex("FF")

        result = _TICPMessageDecoder.convert_from_tof_to_byte(tof)
        self.assertEqual(expected_result, result)

    def test_convert_from_accel_to_bytes_UpperLimit(self):
        value = 32767
        expected_result = bytes.fromhex("FF7F")

        result = _TICPMessageDecoder.convert_from_accel_to_bytes(value)
        self.assertEqual(expected_result, result)

    def test_convert_from_accel_to_bytes_LowerLimit(self):
        value = -32768
        expected_result = bytes.fromhex("0080")

        result = _TICPMessageDecoder.convert_from_accel_to_bytes(value)
        self.assertEqual(expected_result, result)

    def test_convert_from_speed_to_bytes_UpperLimit(self):
        value = 32767
        expected_result = bytes.fromhex("FF7F")

        result = _TICPMessageDecoder.convert_from_speed_to_bytes(value)
        self.assertEqual(expected_result, result)

    def test_convert_from_speed_to_bytes_LowerLimit(self):
        value = 32767
        expected_result = bytes.fromhex("FF7F")

        result = _TICPMessageDecoder.convert_from_speed_to_bytes(value)
        self.assertEqual(expected_result, result)

    def test_convert_from_servo_to_bytes_UpperLimit(self):
        value = 127
        expectet_result = bytes.fromhex("7F")

        result = _TICPMessageDecoder.convert_from_servo_to_bytes(value)
        self.assertEqual(expectet_result, result)

    def test_convert_from_servo_to_bytes_LowerLimit(self):
        value = -128
        expected_result = bytes.fromhex("80")

        result = _TICPMessageDecoder.convert_from_servo_to_bytes(value)
        self.assertEqual(expected_result, result)

    def test_convert_from_fsm_to_bytes(self):
        fsm = 255
        expected_result = bytes.fromhex("FF")

        result = _TICPMessageDecoder.convert_from_fsm_to_bytes(fsm)
        self.assertEqual(expected_result, result)

    def test_convert_byte_to_tof_data(self):
        value = bytes.fromhex("FF")
        expected_result = 255

        result = _TICPMessageDecoder.convert_byte_to_tof_data(value)
        self.assertEqual(expected_result, result)

    def test_convert_from_bytes_to_accel(self):
        value = bytes.fromhex("FF7F")
        expected_result = 32767

        result = _TICPMessageDecoder.convert_from_bytes_to_accel(value)
        self.assertEqual(expected_result, result)

    def test_convert_from_bytes_to_speed(self):
        value = bytes.fromhex("FF7F")
        expected_result = 32767

        result = _TICPMessageDecoder.convert_from_bytes_to_speed(value)
        self.assertEqual(expected_result, result)

    def test_convert_from_bytes_to_servo(self):
        value = bytes.fromhex("7F")
        expected_result = 127

        result = _TICPMessageDecoder.convert_from_bytes_to_servo(value)
        self.assertEqual(expected_result, result)
