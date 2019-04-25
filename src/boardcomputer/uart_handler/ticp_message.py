import logging
import sys

from uart_handler.ticp_common import TICPCommand, TICPMessageType


class TICPMessagePrototype(object):

    def __init__(self, message_type: TICPMessageType, payload: bytes):
        """
        Object representation of TICP Messages with their corresponding payload. This class is
        designed for inheritance. If you want to modiyfy the propertys in an child class just
        modify the _get_* methods instead of the actual propertys.


        Args:
            message_type: TICPMessageType Enum to define the message type and size
            payload: Payload to be attached to the object. Payload size must match the expectet
                     size. Throws ValueError if size of payloads do not match
        """
        self.message_type = message_type
        self.payload = payload

    @property
    def command(self) -> TICPCommand:
        return self._get_command()

    def _get_command(self) -> TICPCommand:
        return self.message_type.command

    @property
    def payload_size(self) -> int:
        return self._get_payload_size()

    def _get_payload_size(self) -> int:
        return self.message_type.payload_size

    @property
    def payload(self) -> bytes:
        return self._payload

    @payload.setter
    def payload(self, payload: bytes) -> None:
        """
                Sets the payload of the TICPMessage with the given bytes[] when the length
                corresponds to the expected value for that message type.

                Throws an ValueError Exeption if the lengths don't correspond

                Use this function to overload the property set_payload in subclasses

                Args:
                    payload: The bytes[] array you want to attach to the
                """
        if payload is None:
            self._payload = b''
        else:
            if len(payload) == self.payload_size:
                self._payload = payload
            else:
                raise ValueError("Passed payload size (" + str(len(payload)) +
                             ") does not match with expected size (" +
                             str(self.payload_size) + ")")

    def handle_message(self):
        raise NotImplemented


class TICPMessageAllCommandData(TICPMessagePrototype):

    def __init__(self, payload: bytes):

        super(TICPMessageAllCommandData, self).__init__(TICPMessageType.RES_ALL_COMMAND_DATA_MSG,
                                                        payload)

    @classmethod
    def from_values(cls, start_signal: bool, round_counter: int, stop_signal: bool):
        payload = 0

        if start_signal:
            payload = payload | 0x80

        if stop_signal:
            payload = payload | 0x04
        if round_counter > 15:
            raise ValueError("Round Counter is bigger than 4 bit (15)")
        else:
            payload = payload | (round_counter << 3)
            payload_bytes = payload.to_bytes(TICPMessageType.RES_ALL_COMMAND_DATA_MSG.payload_size,
                                             byteorder=sys.byteorder,
                                             signed=False)

        return TICPMessageAllCommandData(payload_bytes)

    @property
    def start_signal(self) -> bool:

        payload_int = int.from_bytes(self._payload, byteorder=sys.byteorder, signed=False)
        if (payload_int & 0x80) == 0x80:
            return True

        else:
            return False

    @property
    def stop_signal(self) -> bool:

        payload_int = int.from_bytes(self._payload, byteorder=sys.byteorder, signed=False)
        if (payload_int & 0x04) == 0x04:
            return True

        else:
            return False

    @property
    def round_counter(self) -> int:

        payload_int = int.from_bytes(self._payload, byteorder=sys.byteorder, signed=False)
        return (payload_int & 0x78) >> 3

    def handle_message(self):
        print(self.payload)
        logging.info(self.__str__())

    def __str__(self):
        string = "Start: " + str(self.start_signal) + " | Stop: " + str(self.stop_signal) + \
                 " | Round: " + str(self.round_counter)
        return string


class TICPMessageAllSensorData(TICPMessagePrototype):

    def __init__(self, payload: bytes):
        """
        Constructor for the creation of a TICPMessageAllSensorData Object from Bytes. Use this
        Constructor to generate an Object out of a given Payload from the UART Interface. If you
        want to create an Messae Object out of given Data, use the from_sensor_values()
        Class Methode.

        Args:
            payload: 10 Bytes Payload
        """
        super(TICPMessageAllSensorData, self).__init__(TICPMessageType.RES_ALL_SENSOR_DATA_MSG,
                                                       payload
                                                       )

    def __str__(self):
        string = "Speed: " + str(self.speed) + " TOF (1/2): " + str(self.tof1) + "|" + \
                 str(self.tof2) + " Accel (x/y): " + str(self.accel_x) + "|" + \
                 str(self.accel_y) + " Servo: " + str(self.servo) + " FSM: " + str(self.fsm_state)

        return string

    @classmethod
    def from_sensor_values(cls, tof1_data: int,
                           tof2_data: int,
                           accelx_data: int,
                           accely_data: int,
                           speed_data: int,
                           servo1_data: int,
                           fsm_state: int):
        """
        Factory Methode for the creation of new TICPMessageAllSensorData Object from Data Values
        instead of bytes

        Args:
            tof1_data:  Value of TOF1 in mm (0-255)
            tof2_data:  Value of TOF2 in mm (0-255)
            accelx_data: Value of Acceleration X in mG (-32'768 - 32'767)
            accely_data: Value of Acceleration Y in mG (-32'768 - 32'767)
            speed_data: Value of Speed in mm/s (-32'768 - 32'767)
            servo1_data: Value of Servo in Degree (-128 - 127)
            fsm_state: Value of FSM State (0-255)

        Returns: TICPMessageALLSensorData Object with the given Values as Payload

        """
        payload_tmp = _TICPMessageDecoder.convert_from_tof_to_byte(tof1_data)
        payload_tmp += _TICPMessageDecoder.convert_from_tof_to_byte(tof2_data)
        payload_tmp += _TICPMessageDecoder.convert_from_accel_to_bytes(accelx_data)
        payload_tmp += _TICPMessageDecoder.convert_from_accel_to_bytes(accely_data)
        payload_tmp += _TICPMessageDecoder.convert_from_speed_to_bytes(speed_data)
        payload_tmp += _TICPMessageDecoder.convert_from_servo_to_bytes(servo1_data)
        payload_tmp += _TICPMessageDecoder.convert_from_fsm_to_bytes(fsm_state)

        return TICPMessageAllSensorData(payload_tmp)

    @property
    def tof1(self) -> int:
        return _TICPMessageDecoder.convert_byte_to_tof_data(self._payload[0:1])

    @property
    def tof2(self) -> int:
        return _TICPMessageDecoder.convert_byte_to_tof_data((self._payload[1:2]))

    @property
    def accel_x(self) -> int:
        return _TICPMessageDecoder.convert_from_bytes_to_accel(self._payload[2:4])

    @property
    def accel_y(self) -> int:
        return _TICPMessageDecoder.convert_from_bytes_to_accel(self._payload[4:6])

    @property
    def speed(self) -> int:
        return _TICPMessageDecoder.convert_from_bytes_to_speed(self._payload[6:8])

    @property
    def servo(self) -> int:
        return _TICPMessageDecoder.convert_from_bytes_to_servo(self._payload[8:9])

    @property
    def fsm_state(self) -> int:
        return _TICPMessageDecoder.convert_from_byte_to_fsm(self._payload[9:10])

    def handle_message(self):
        logging.info(self.__str__())
        print(self.__str__())


class _TICPMessageDecoder:

    @staticmethod
    def convert_from_tof_to_byte(tof_data: int) -> bytes:
        """
        Converts 8-bit TOF Data to bytes. Throws ValueError if the given int is bigger than
        255.

        Args:
            tof_data: Data to convert

        Returns: Single unsigned byte with byteorder sys.byteorder

        """
        if tof_data <= 255:
            return tof_data.to_bytes(1, byteorder=sys.byteorder, signed=False)
        else:
            raise ValueError("Value of TOF1 is bigger than 255 "
                             "and therefor can't be decoded to 1 Byte block")

    @staticmethod
    def convert_from_accel_to_bytes(accel_data: int) -> bytes:
        """
        Converts 16-bit ACCEL Data to bytes.

        Args:
            accel_data: Data to covert

        Returns: Two signed bytes with byteorder sys.byteorder

        """
        return accel_data.to_bytes(2, byteorder=sys.byteorder, signed=True)

    @staticmethod
    def convert_from_speed_to_bytes(speed_data: int) -> bytes:
        """
        Converts 16-bit SPEED Data to bytes.

        Args:
            speed_data: Data to convert

        Returns: Two signed bytes with byteorder sys.byteorder

        """
        return speed_data.to_bytes(2, byteorder=sys.byteorder, signed=True)

    @staticmethod
    def convert_from_servo_to_bytes(servo_data: int) -> bytes:
        """
        Converts 8-Bit SERVO Data to one byte. Throws ValueError if the given int is not between
        -128 and 127.

        Args:
            servo_data: Data to convert (-128 <= servo_data <=127)

        Returns: Two signed bytes with byteorder sys.byteorder

        """
        if 127 >= servo_data >= -128:
            return servo_data.to_bytes(1, byteorder=sys.byteorder, signed=True)
        else:
            raise ValueError("Value of Servo (" + str(servo_data) +
                             ") is not in a valid Range (-128 - 127) ",
                             "and therefor can't be decoded to 1 Byte block")

    @staticmethod
    def convert_from_fsm_to_bytes(fsm_data: int) -> bytes:
        """
        Converts from FSM Data to one byte. Throws ValueError if the given int is bigger than
        255.

        Args:
            fsm_data: Data to convert (0 <= fsm_data <= 255)

        Returns: One unsigned byte with byteorder sys.byteorder

        """
        if 0 <= fsm_data <= 255:
            return fsm_data.to_bytes(1, byteorder=sys.byteorder, signed=False)
        else:
            raise ValueError("Value of FSM Data is bigger than 255 "
                             "and therefor can't be decoded to 1 Byte block")

    @staticmethod
    def convert_byte_to_tof_data(byte_data: bytes) -> int:
        """
        Convert byte to TOF Data.

        Args:
            byte_data: 1 unsigned byte datablock with byteorder=sys.byteorder

        Returns: Value of the given block

        """
        return int().from_bytes(byte_data, byteorder=sys.byteorder, signed=False)

    @staticmethod
    def convert_from_bytes_to_accel(byte_data: bytes) -> int:
        """
        Convert two bytes to ACCEL Data

        Args:
            byte_data: 2 Bytes signed data with byteorder=sys.byteorder

        Returns: Value of the given block

        """
        return int.from_bytes(byte_data, byteorder=sys.byteorder, signed=True)

    @staticmethod
    def convert_from_bytes_to_speed(byte_data: bytes) -> int:
        """
        Convert two bytes to SPEED Data
        Args:
            byte_data:  2 Bytes signed data with byteorder=sys.byteorder

        Returns: Value of the given byte block

        """
        return int.from_bytes(byte_data, byteorder=sys.byteorder, signed=True)

    @staticmethod
    def convert_from_bytes_to_servo(byte_data: bytes) -> int:
        """
        Converts one byte to SERVO Data.

        Args:
            byte_data: 1 Byte signed data with byteorder=sys.byteorder

        Returns: Value of the given byte block

        """
        return int.from_bytes(byte_data, byteorder=sys.byteorder, signed=True)

    @staticmethod
    def convert_from_byte_to_fsm(byte_data: bytes) -> int:
        """
        Convert one byte to FSM Data

        Args:
            byte_data: One Byte unsigned data with byteorder=sys.byteorder

        Returns: Value of the given byte block

        """
        return int.from_bytes(byte_data, byteorder=sys.byteorder, signed=False)
