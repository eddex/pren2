from image_analysis.image_analysis import analyze_image, SignalType, SignalNumber, AnalysisOutcome


def test_analyze_image():
    outcome = analyze_image(None)
    assert outcome.signal_type == SignalType.UNDEFINED
    assert outcome.signal_number == SignalNumber.UNDEFINED


def test_analysis_outcome():
    outcome = AnalysisOutcome(SignalType.START_SIGNAL, SignalNumber.EIGHT)
    assert outcome.signal_type == SignalType.START_SIGNAL
    assert outcome.signal_number == SignalNumber.EIGHT
