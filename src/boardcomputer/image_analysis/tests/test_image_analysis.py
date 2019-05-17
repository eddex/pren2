from mock import Mock
import sys
sys.modules['openvino'] = Mock()

from image_analysis.image_analysis import ImageAnalyzer


def test_analyze_image():
    # TODO: write tests
    pass