from enum import Enum


class SignalType(Enum):
    """
    Different types of signals that can be located next to the train tracks.
    """
    INFO_SIGNAL = 0
    STOP_SIGNAL = 1
    START_SIGNAL = 2
    UNDEFINED = 3


class RecognizedNumber(Enum):
    """
    Possible Numbers that can be found on the signals.
    """
    ZERO = 0
    ONE = 1
    TWO = 2
    THREE = 3
    FOUR = 4
    FIVE = 5
    SIX = 6
    SEVEN = 7
    EIGHT = 8
    NINE = 9
    UNDEFINED = 10


class AnalysisOutcome:
    """
    Contains the information found in an image.
    """
    signal_type: SignalType
    recognized_number: RecognizedNumber

    def __init__(self, recognized_number: RecognizedNumber, signal_type: SignalType):
        """
        Args:
            recognized_number (RecognizedNumber)
            signal_type (SignalType)
        """
        self.recognized_number = recognized_number
        self.signal_type = signal_type


def analyze_image(image):
    """
    Analyzes an image to recognize different types of signals.
    Args:
        image: An image taken by the camera.

    Returns:
        The outcome of the image analysis as an AnalysisOutcome.
    """
    # TODO: implement image analysis logic.
    return AnalysisOutcome(RecognizedNumber.UNDEFINED, SignalType.UNDEFINED)
