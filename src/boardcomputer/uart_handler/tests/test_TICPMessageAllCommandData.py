from unittest import TestCase

from uart_handler.ticp_message import TICPMessageAllCommandData


class TestTICPMessageAllCommandData(TestCase):

    def test_from_values(self):
        msg = TICPMessageAllCommandData.from_values(start_signal=True,
                                                    round_counter=10,
                                                    stop_signal=False)

        self.assertTrue(msg.start_signal == True and
                        msg.stop_signal == False and
                        msg.round_counter == 10
                        )

    def test_from_bytes(self):
        msg = TICPMessageAllCommandData(bytes.fromhex("9C"))

        self.assertTrue(msg.stop_signal == True and
                        msg.stop_signal == True and
                        msg.round_counter == 3
                        )

    def test_to_big_round_counter_exeption(self):
        with self.assertRaises(ValueError):
            msg = TICPMessageAllCommandData.from_values(start_signal=True,
                                                        round_counter=16,
                                                        stop_signal=False)
