from enum import Enum


class SignalType(Enum):
    INFO = 0
    STOP = 1
    START = 2


class Signal(Enum):
    INFO_ZERO = (SignalType.INFO, 0)
    INFO_ONE = (SignalType.INFO, 1)
    INFO_TWO = (SignalType.INFO, 2)
    INFO_THREE = (SignalType.INFO, 3)
    INFO_FOUR = (SignalType.INFO, 4)
    INFO_FIVE = (SignalType.INFO, 5)
    INFO_SIX = (SignalType.INFO, 6)
    INFO_SEVEN = (SignalType.INFO, 7)
    INFO_EIGHT = (SignalType.INFO, 8)
    INFO_NINE = (SignalType.INFO, 9)

    STOP_ZERO = (SignalType.STOP, 0)
    STOP_ONE = (SignalType.STOP, 1)
    STOP_TWO = (SignalType.STOP, 2)
    STOP_THREE = (SignalType.STOP, 3)
    STOP_FOUR = (SignalType.STOP, 4)
    STOP_FIVE = (SignalType.STOP, 5)
    STOP_SIX = (SignalType.STOP, 6)
    STOP_SEVEN = (SignalType.STOP, 7)
    STOP_EIGHT = (SignalType.STOP, 8)
    STOP_NINE = (SignalType.STOP, 9)

    START = (SignalType.START, 0)

    def __init__(self, signal_type: SignalType, number: int):
        self._signal_type = signal_type
        self._number = number

    @property
    def signal_type(self) -> SignalType:
        return self._signal_type

    @property
    def number(self) -> int:
        return self._number