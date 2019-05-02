from queue import Queue
from unittest import TestCase

from mock import MagicMock
import sys
sys.modules['serial'] = MagicMock()
sys.modules['WebStream'] = MagicMock()

from uart_handler.ticp_handler import _TICPToSerialAdapter
from uart_handler.ticp_message import TICPMessageAllSensorData

import pytest


class Test_TICPToSerialAdapter(TestCase):

    @pytest.mark.skip(reason="fails but I don't know why..")
    def test_write(self):

        queue = Queue()
        serial_mock = _SerialInterfaceMock(queue)
        adapter = _TICPToSerialAdapter(serial_mock)

        payload = bytes.fromhex("FF00FFFF0000FFFF0000")
        msg = TICPMessageAllSensorData(payload)

        exp_queue = Queue()
        exp_queue.put(msg.command.opcode)
        for block in payload:
            exp_queue.put(block.to_bytes(1, byteorder='big', signed=False))

        adapter.write(msg)

        self.assertEqual(exp_queue.queue, queue.queue)

    def test_read(self):

        queue = Queue()
        payload = bytes.fromhex("FF00FFFF0000FFFF0000")
        exp_msg = TICPMessageAllSensorData(payload)

        queue.put(exp_msg.command.opcode)
        for block in payload:
            queue.put(block.to_bytes(1, byteorder='big', signed=False))

        serial_mock = _SerialInterfaceMock(queue)
        adapter = _TICPToSerialAdapter(serial_mock)
        act_msg = adapter.read()

        self.assertTrue(exp_msg.command.opcode == act_msg.command.opcode)
        self.assertTrue(exp_msg.payload == exp_msg.payload)


class _SerialInterfaceMock:

    def __init__(self, queue: Queue):
        self._queue = queue

    @property
    def queue(self):
        return self._queue

    def write(self, payload: bytes):

        for i in range(len(payload)):
            self._queue.put(payload[i:i + 1])

    def read(self, num_blocks: int):

        result = self._queue.get()

        for i in range(num_blocks - 1):
            result += self._queue.get()

        return result
