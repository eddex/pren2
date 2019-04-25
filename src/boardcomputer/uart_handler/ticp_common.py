import sys
from enum import Enum


class TICPCommand(Enum):
    """Enumeration for Command Opcodes of the TICP Protocol.
    This Class is a specialization of the Enum class.
    Use this Enumeration für decoding 8 bit Message Blocks to the corresponding Opcode

    Example:
        The easiest way to initialize an TICPCommand Enum for a given byte is by passing
        it to the constuctor of the Enum as follow::

            $ opcode = (np.uint8) uart.pull(8)
            $ command = TICPCommand(opcode)

        or by passing an value::

            $ command = TICPCommand(1)

    _Git Repository:
        https://github.com/eddex/pren2

    """
    # Request Messages (REQ)
    REQ_ALL_SENSOR_DATA = bytes.fromhex("01")
    REQ_TOF1_DATA = bytes.fromhex("02")
    REQ_TOF2_DATA = bytes.fromhex("03")
    REQ_ACCEL_DATA = bytes.fromhex("04")
    REQ_SPEED_DATA = bytes.fromhex("05")
    REQ_SERVO_DATA = bytes.fromhex("06")
    REQ_FSM_STATE = bytes.fromhex("07")

    # Response Messages (RES)
    RES_ALL_SENSOR_DATA = bytes.fromhex("21")
    RES_TOF1_SENSOR_DATA = bytes.fromhex("22")
    RES_TOF2_SENSOR_DATA = bytes.fromhex("23")
    RES_ACCEL_DATA = bytes.fromhex("24")
    RES_SPEED_DATA = bytes.fromhex("25")
    RES_SERVO_DATA = bytes.fromhex("26")
    RES_FSM_STATE = bytes.fromhex("27")
    RES_ALL_COMMAND_DATA = bytes.fromhex("28")

    # Special Commands (SPC)
    SPC_LOOPBACK = bytes.fromhex("70")
    SPC_LOOPBACK_RESPONSE = bytes.fromhex("71")
    SPC_ACKNOWLEDGE = bytes.fromhex("79")
    SPC_START_OF_FRAME = bytes.fromhex("7A")
    SPC_ESCAPE = bytes.fromhex("7D")
    SPC_END_OF_FRAME = bytes.fromhex("7E")

    # Synchronization
    SYN_MC = bytes.fromhex("00")
    SYN_BC = bytes.fromhex("7F")

    def __init__(self, opcode: bytes):
        self._opcode = opcode

    @property
    def opcode(self) -> bytes:
        """ Returns a 8bit unsigned number containing the Opcoode of the Command

        Returns:
             8bit unsigned number representing the Opcoode
        """
        return self._opcode[0:1]


class TICPMessageType(Enum):
    """Enumeration for Message Types of the TICP Protocol. This Class is a specialization of
    the Enum class. Use this Enumeration for the creation of TICP Messages.

    Example:
        For the creation of TICPMessageType Enum from existing TICPCommands use
        the TICPCMessageTypeFactory

            $ factory = TICPMessageTypeFactory()
            $ msg = factory.get_TICPMessageType_from_TICPCommand(TICPCommand.SYN_MC)


    _Git Repository:
        https://github.com/eddex/pren2

    """

    # Request Messages REQ
    REQ_ALL_SENSOR_DATA_MSG = (TICPCommand.REQ_ALL_SENSOR_DATA, 0)
    REQ_TOF1_DATA_MSG = (TICPCommand.REQ_TOF1_DATA, 0)
    REQ_TOF2_DATA_MSG = (TICPCommand.REQ_TOF2_DATA, 0)
    REQ_ACCEL_DATA_MSG = (TICPCommand.REQ_ACCEL_DATA, 0)
    REQ_SPEED_DATA_MSG = (TICPCommand.REQ_SPEED_DATA, 0)
    REQ_SERVO_DATA_MSG = (TICPCommand.REQ_SERVO_DATA, 0)
    REQ_FSM_STATE_MSG = (TICPCommand.REQ_FSM_STATE, 0)

    # Response Message RES
    RES_ALL_SENSOR_DATA_MSG = (TICPCommand.RES_ALL_SENSOR_DATA, 10)
    RES_TOF1_SENSOR_DATA_MSG = (TICPCommand.RES_TOF1_SENSOR_DATA, 1)
    RES_TOF2_SENSOR_DATA_MSG = (TICPCommand.RES_TOF2_SENSOR_DATA, 1)
    RES_ACCEL_DATA_MSG = (TICPCommand.RES_ACCEL_DATA, 4)
    RES_SPEED_DATA_MSG = (TICPCommand.RES_SPEED_DATA, 2)
    RES_SERVO_DATA_MSG = (TICPCommand.RES_SERVO_DATA, 1)
    RES_FSM_STATE_MSG = (TICPCommand.RES_FSM_STATE, 1)
    RES_ALL_COMMAND_DATA_MSG = (TICPCommand.RES_ALL_COMMAND_DATA, 1)

    # Special Commands SPC
    SPC_LOOPBACK_MSG = (TICPCommand.SPC_LOOPBACK, 1)
    SPC_LOOPBACK_RESPONSE_MSG = (TICPCommand.SPC_LOOPBACK_RESPONSE, 1)
    SPC_ACKNOWLEDGE_MSG = (TICPCommand.SPC_ACKNOWLEDGE, 0)
    SPC_START_OF_FRAME_MSG = (TICPCommand.SPC_START_OF_FRAME, 0)
    SPC_ESCAPE_MSG = (TICPCommand.SPC_ESCAPE, 0)
    SPC_END_OF_FRAM_MSG = (TICPCommand.SPC_END_OF_FRAME, 0)

    # Synchronization
    SYN_MC_MSG = (TICPCommand.SYN_MC, 0)
    SYN_BC_MSG = (TICPCommand.SYN_BC, 0)

    def __init__(self, command: TICPCommand, payload_size: int):
        self._command = command
        self._payload_size = payload_size

    @property
    def command(self) -> TICPCommand:
        """ Return the TICPCommand Enum of the TICPMessageType

        :return: TICPCommand
        """
        return self._command

    @property
    def payload_size(self) -> int:
        """ Returns the length of the message payload as number of byte (8bit) blocks.

        :return: int
        """
        return self._payload_size


class TICPMessageTypeFactory:
    """
    Factory for creation of TICPMessageType Enumerations

    _Git Repository:
        https://github.com/eddex/pren2

    """

    def __init__(self):
        self.__command_to_msg_type_dict = {}
        self.__init_command_to_msg_type_dict()

    def __init_command_to_msg_type_dict(self) -> None:
        """
        Initializes a Python Dictionary for a translation of TICPCommand to TICPMessageType
        Returns:

        """
        for msg in TICPMessageType:
            self.__command_to_msg_type_dict[msg.command] = msg

    def get_ticp_message_type_from_command(self, command: TICPCommand) -> TICPMessageType:
        """ Factory methode for creating an TICPMessageType from a TICPCommand

        Args:
            command: The TICPCommand for which a TICPMessageType should be created

        Returns:
            The return value in form of an TICPMessageType Enum

        """
        return self.__command_to_msg_type_dict[command]

    def get_ticp_message_type_from_int(self, opcode: int) -> TICPMessageType:
        """ Factory methode for creating an TICPMessageType from a 'uint8' opcode

        Args:
            opcode: The opcode for which a TICPMessageType should be created

        Returns:
            The return value in form of an TICPMessageType Enum

        """
        opcode_bytes = opcode.to_bytes(1, byteorder=sys.byteorder, signed=False)
        cmd = TICPCommand(opcode_bytes)
        return self.__command_to_msg_type_dict[cmd]

    def get_ticp_message_type_from_byte(self, opcode: bytes) -> TICPMessageType:

        cmd = TICPCommand(opcode)
        return self.__command_to_msg_type_dict[cmd]
