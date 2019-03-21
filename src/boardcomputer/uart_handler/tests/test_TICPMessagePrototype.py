from unittest import TestCase

from uart_handler.ticp_common import TICPMessageType
from uart_handler.ticp_message import TICPMessagePrototype


class TestTICPMessagePrototype(TestCase):
    def test_command(self):
        msg = TICPMessagePrototype(TICPMessageType.RES_ALL_SENSOR_DATA_MSG,
                                   b'\xff\x00\xff\x7f\x00\x00\xff\x7f\x7f\xff')
        self.assertEqual(msg.payload, b'\xff\x00\xff\x7f\x00\x00\xff\x7f\x7f\xff')

    def test_payload_size(self):
        msg = TICPMessagePrototype(TICPMessageType.RES_ALL_SENSOR_DATA_MSG,
                                   b'\xff\x00\xff\x7f\x00\x00\xff\x7f\x7f\xff')
        self.assertEqual(10, msg.payload_size)
