import logging
import queue
import threading
from time import sleep

import serial
import socketio

from uart_handler.ticp_common import TICPCommand
from uart_handler.ticp_common import TICPMessageTypeFactory, TICPMessageType
from uart_handler.ticp_message import TICPMessageAllSensorData, TICPMessageAllCommandData
from uart_handler.ticp_message import TICPMessagePrototype


class TICPMessageIOStreamInterface:
    """ Defines the Interface for read/write TICPMessages to an IO Stream """

    def write(self, ticp_message: TICPMessagePrototype) -> None:
        """
        Write a TICPMessagePrototype Object into an IO Stream.


        Args:
            ticp_message: TICPMessage to be writen in the IO Streamm
        """
        raise NotImplemented

    def read(self):
        """
        Read bytes from the IO Stream and create an TICPMessage Object out of the data.$

        Args:
            num_bytes: Number of Bytes to read from the Stream
        """
        raise NotImplemented

    def in_waiting(self) -> int:
        """
        Return the number of bytes waiting in the input buffer
        Returns:

        """
        raise NotImplemented


class _TICPToSerialAdapter(TICPMessageIOStreamInterface):

    def __init__(self, serial: serial.Serial):
        self._serial = serial
        self._message_type_factory = TICPMessageTypeFactory()

    def write(self, ticp_message: TICPMessagePrototype) -> None:
        # logging.info(ticp_message.__str__())
        packet = TICPCommand.SYN_BC.opcode
        # packet += ticp_message.command.opcode
        packet += ticp_message.payload

        self._serial.write(packet)

    def read(self) -> TICPMessagePrototype:
        opcode_byte = self._serial.read(1)

        try:
            command = TICPCommand(opcode_byte)
            ticp_message_type = self._message_type_factory.get_ticp_message_type_from_command(
                command)
            if ticp_message_type.payload_size != 0:
                payload = self._serial.read(ticp_message_type.payload_size)
            else:
                payload = None

            return TICPMessageAllSensorData(payload)

        except ValueError:
            return TICPMessagePrototype(TICPMessageType.SPC_END_OF_FRAM_MSG, None)

    def in_waiting(self) -> int:
        return self._serial.in_waiting


class TICPHandler:

    def __init__(self, io_interface: TICPMessageIOStreamInterface):
        """

        Returns:
            object: 
        """
        # Private attributes
        self._io_interface = io_interface
        self._write_queue = queue.Queue(maxsize=100)
        self.serial_lock = threading.Lock()

        self._stopped = False

        # Initialize Threading
        self.thread1 = threading.Thread(target=self.read_thread)
        self.thread2 = threading.Thread(target=self.write_thread)

        self.thread1.start()
        self.thread2.start()

    @property
    def serial_interface(self) -> serial.Serial:
        return self._io_interface

    def write_message(self, message: TICPMessagePrototype, block: bool = True, timeout: int = 0.1):
        self._write_queue.put(message, block, timeout)

    def read_thread(self):
        while not self._stopped:

            # self.serial_lock.acquire()
            message = self._io_interface.read()
            # self.serial_lock.release()

            if message.command is not TICPCommand.SPC_END_OF_FRAME:
                try:
                    message.handle_message()

                except NotImplementedError:
                    # ToDo Handle if Message was not decoded right
                    pass
            else:
                logging.debug(
                    "TICPHandler - read_thread: Received invalid/empty message within the "
                    "timeout-> drop")
                pass

    def write_thread(self):
        while not self._stopped:

            if not self._write_queue.empty():
                message = self._write_queue.get()

                # self.serial_lock.acquire()
                self._io_interface.write(message)
                # self.serial_lock.release()

    def stop_handler(self):
        self._stopped = True
        self.thread1.join()
        self.thread2.join()


class _SerialInterfaceMock:

    def __init__(self, queue: queue.Queue):
        self._queue = queue

    @property
    def queue(self):
        return self._queue

    def write(self, payload: bytes):

        for i in range(len(payload)):
            self._queue.put(payload[i:i + 1])

    def read(self, num_blocks: int):

        try:
            result = self._queue.get(True, 0.1)

            for i in range(num_blocks - 1):
                result += self._queue.get(True, 0.1)

            return result

        except queue.Empty:
            return None


# TODO Move helper Class to frontend Package

class WebStream:

    def __init__(self):
        self.sio = socketio.Client()
        self.sio.connect('http://192.168.10.1:5000')

    def write(self, msg):
        self.sio.emit('message', msg)

    def flush(self):
        pass

    def close(self):
        self.sio.disconnect()


if __name__ == "__main__":
    ticp_logger = logging.getLogger()
    ticp_logger.setLevel(logging.INFO)

    webStream = WebStream()

    web_logger = logging.StreamHandler(webStream)
    web_logger.setLevel(logging.DEBUG)
    web_logger.terminator = ''

    formatter = logging.Formatter('%(asctime)s %(name)-12s: %(levelname)-8s %(message)s')

    web_logger.setFormatter(formatter)

    ticp_logger.addHandler(web_logger)

    queueTest = queue.Queue(maxsize=1000)
    ser = serial.Serial(port="/dev/serial0", baudrate=19200, parity=serial.PARITY_NONE, stopbits=1)
    serial_mock = _SerialInterfaceMock(queueTest)
    adapter = _TICPToSerialAdapter(ser)
    handler = TICPHandler(adapter)

    logging.info("#### Stop ####")

    for i in range(30):
        msg = TICPMessageAllCommandData.from_values(start_signal=False, round_counter=0,
                                                    stop_signal=False)
        handler.write_message(msg)
        sleep(0.1)

    logging.info("#### Runde 1 ####")

    for i in range(130):
        msg = TICPMessageAllCommandData.from_values(start_signal=True, round_counter=0,
                                                    stop_signal=False)
        handler.write_message(msg)
        sleep(0.1)

    logging.info("#### Runde 2 ####")

    for i in range(50):
        msg = TICPMessageAllCommandData.from_values(start_signal=True, round_counter=1,
                                                    stop_signal=False)
        handler.write_message(msg)
        sleep(0.1)

    logging.info(" #### Stop ####")

    for i in range(100):
        msg = TICPMessageAllCommandData.from_values(start_signal=True, round_counter=2,
                                                    stop_signal=True)
        handler.write_message(msg)

        sleep(0.1)

    exit()
    handler.stop_handler()
    webStream.close()
