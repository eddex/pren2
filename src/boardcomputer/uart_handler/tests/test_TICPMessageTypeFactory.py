from unittest import TestCase

from uart_handler.ticp_common import TICPCommand, TICPMessageTypeFactory, TICPMessageType


class TestTICPMessageTypeFactory(TestCase):

    def test_single_get_ticp_message_type_from_command(self):

        factory = TICPMessageTypeFactory()
        msg_target = TICPMessageType.RES_ALL_SENSOR_DATA_MSG
        msg_actual = factory.get_ticp_message_type_from_command(TICPCommand.RES_ALL_SENSOR_DATA)

        self.assertEqual(msg_target, msg_actual)

    def test_single_get_ticp_message_type_from_int(self):

        factory = TICPMessageTypeFactory()
        msg_target = TICPMessageType.RES_ALL_SENSOR_DATA_MSG
        cmd = TICPCommand.RES_ALL_SENSOR_DATA
        msg_actual = factory.get_ticp_message_type_from_int(cmd.opcode)

        self.assertEqual(msg_target, msg_actual)

    def test_all_get_ticp_message_type_from_command(self):

        factory = TICPMessageTypeFactory()

        for command in TICPCommand:
            try:
                msg = factory.get_ticp_message_type_from_command(command)
                if msg.command != command:
                    break

            except KeyError:
                self.fail("No TICPMessage defined for the given TICPCommand: " + command.__str__())

        else:
            self.assertTrue(True)

    def test_all_get_ticp_message_type_from_int(self):
        factory = TICPMessageTypeFactory()
        for command in TICPCommand:
            try:
                opcode = command.opcode
                msg = factory.get_ticp_message_type_from_int(opcode)
                if msg.command != command:
                    break
            except KeyError:
                self.fail(
                    "No TICPMessage defined for the given TICPCommand: " + command.__str__())
        else:
            self.assertTrue(True)
