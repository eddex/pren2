from mock import Mock
import mock
import sys
sys.modules['openvino.inference_engine'] = Mock()

from image_analysis.image_analysis import ImageAnalyzer

def test_analyze_image():
    # TODO: write tests
    pass