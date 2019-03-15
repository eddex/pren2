from image_analysis.image_analysis import analyze_image, SignalType, RecognizedNumber, AnalysisOutcome


def test_analyze_image():
    outcome = analyze_image(None)
    assert outcome.signal_type == SignalType.UNDEFINED
    assert outcome.recognized_number == RecognizedNumber.UNDEFINED


def test_analysis_outcome():
    outcome = AnalysisOutcome(RecognizedNumber.EIGHT, SignalType.START_SIGNAL)
    assert outcome.signal_type == SignalType.START_SIGNAL
    assert outcome.recognized_number == RecognizedNumber.EIGHT
