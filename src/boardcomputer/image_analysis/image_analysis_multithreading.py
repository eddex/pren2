#import sys
#[sys.path.append(i) for i in ['.', '..']]

from fsm.signals import Signal
import os, sys, heapq
import cv2
import time
import numpy as np, math
import logging
import multiprocessing as mp
import threading
from fsm.config import BaseConfig

try:
    # Raspberry
    sys.path.insert(0, '/opt/intel/openvino/python/python3.5')
    from armv7l.openvino.inference_engine import IENetwork, IEPlugin
    from picamera import PiCamera
    from fsm.config import BaseImageAnalysisNCS2Config as config
except:
    # Dev PC
    from openvino.inference_engine import IENetwork, IEPlugin
    from fsm.config import BaseImageAnalysisCPUConfig as config

yolo_scale_13 = 13
yolo_scale_26 = 26
yolo_scale_52 = 52

classes = 7
coords = 4
num = 3
anchors = [10,14, 23,27, 37,58, 81,82, 135,169, 344,319]

LABELS = (
    "signal-1",
    "signal-2",
    "signal-3",
    "signal-4",
    "signal-5",
    "signal-6",
    "signal-7",
    "signal-8",
    "signal-9",
    "signal-start",)

LABELS_TMS_INFO = (
    Signal.INFO_ONE,
    Signal.INFO_TWO,
    Signal.INFO_THREE,
    Signal.INFO_FOUR,
    Signal.INFO_FIVE,
    Signal.INFO_SIX,
    Signal.INFO_SEVEN,
    Signal.INFO_EIGHT,
    Signal.INFO_NINE,
    Signal.START)

LABELS_TMS_STOP = (
    Signal.STOP_ONE,
    Signal.STOP_TWO,
    Signal.STOP_THREE,
    Signal.STOP_FOUR,
    Signal.STOP_FIVE,
    Signal.STOP_SIX,
    Signal.STOP_SEVEN,
    Signal.STOP_EIGHT,
    Signal.STOP_NINE,
    Signal.START)

processes = []

label_text_color = (255, 255, 255)
label_background_color = (125, 175, 75)
box_color = (255, 128, 0)
box_thickness = 1


def entry_index(side, l_coords, l_classes, location, entry):
    n = int(location / (side * side))
    loc = location % (side * side)
    return int(n * side * side * (l_coords + l_classes + 1) + entry * side * side + loc)


def parse_yolov3_output_classes_only(blob, threshold):
    """
    Args:
        blob: the output from the neural network
        threshold: the threshold for the confidence of the detected object

    Returns: the detected objects
    """
    objects = []
    out_blob_h = blob.shape[2]

    side = out_blob_h  # == 13

    side_square = side * side
    output_blob = blob.flatten()

    for i in range(side_square):
        for n in range(num):
            obj_index = entry_index(side, coords, classes, n * side * side + i, coords)
            scale = output_blob[obj_index]
            if scale < threshold:
                continue
            for j in range(classes):
                class_index = entry_index(side, coords, classes, n * side_square + i, coords + 1 + j)
                prob = scale * output_blob[class_index]
                if prob < threshold:
                    continue
                obj = DetectionObjectFaster(j, prob)
                objects.append(obj)
    return objects


class DetectionObject:

    def __init__(self, x, y, h, w, class_id, confidence, h_scale, w_scale):
        self.x_min = int((x - w / 2) * w_scale)
        self.y_min = int((y - h / 2) * h_scale)
        self.x_max = int(self.x_min + w * w_scale)
        self.y_max = int(self.y_min + h * h_scale)
        self.class_id = class_id
        self.confidence = confidence


class DetectionObjectFaster:

    def __init__(self, class_id, confidence):
        self.class_id = class_id
        self.confidence = confidence


class NcsWorker(object):

    def __init__(self, frame_buffer, results, video_fps):
        self.frameBuffer = frame_buffer
        self.model_xml = "image_analysis/model/{}.xml".format(config.MODEL_NAME)
        self.model_bin = "image_analysis/model/{}.bin".format(config.MODEL_NAME)
        self.threshold = 0.4
        self.num_requests = 4
        self.inferred_request = [0] * self.num_requests
        self.heap_request = []
        self.inferred_cnt = 0
        self.plugin = IEPlugin(device=config.DEVICE)
        self.net = IENetwork(model=self.model_xml, weights=self.model_bin)
        self.input_blob = next(iter(self.net.inputs))
        self.exec_net = self.plugin.load(network=self.net, num_requests=self.num_requests)
        self.results = results
        self.predict_async_time = 800
        self.skip_frame = 0
        self.roop_frame = 0
        self.video_fps = video_fps
        self.new_w = 416
        self.new_h = 416
        

    def get_index_of_item_in_list(self, l, x, not_found_value=-1):
        """
        Args:
            l: Search list
            x: Search target value
            not_found_value: return value if index out of range

        Returns:
            if x is in the list: the index of the item in the list
            else: the not_found_value
        """
        if x in l:
            return l.index(x)
        else:
            return not_found_value


    def skip_frame_measurement(self):
        surplustime_per_second = (1000 - self.predict_async_time)
        if surplustime_per_second > 0.0:
            frame_per_millisecond = (1000 / self.video_fps)
            total_skip_frame = surplustime_per_second / frame_per_millisecond
            self.skip_frame = int(total_skip_frame / self.num_requests)
        else:
            self.skip_frame = 0


    def intersection_over_union(self, box_1, box_2):
        width_of_overlap_area = min(box_1.xmax, box_2.xmax) - max(box_1.xmin, box_2.xmin)
        height_of_overlap_area = min(box_1.ymax, box_2.ymax) - max(box_1.ymin, box_2.ymin)
        if width_of_overlap_area < 0.0 or height_of_overlap_area < 0.0:
            area_of_overlap = 0.0
        else:
            area_of_overlap = width_of_overlap_area * height_of_overlap_area
        box_1_area = (box_1.ymax - box_1.ymin) * (box_1.xmax - box_1.xmin)
        box_2_area = (box_2.ymax - box_2.ymin) * (box_2.xmax - box_2.xmin)
        area_of_union = box_1_area + box_2_area - area_of_overlap

        if area_of_union <= 0.0:
            result = 0.0
        else:
            result = (area_of_overlap / area_of_union)
        return result


    def predict_async(self):
        try:
            if self.frameBuffer.empty():
                return

            self.roop_frame += 1
            if self.roop_frame <= self.skip_frame:
                self.frameBuffer.get()
                return
            self.roop_frame = 0

            prepared_image = self.frameBuffer.get()
            reqnum = self.get_index_of_item_in_list(self.inferred_request, 0)

            if reqnum > -1:
                self.exec_net.start_async(request_id=reqnum, inputs={self.input_blob: prepared_image})
                self.inferred_request[reqnum] = 1
                self.inferred_cnt += 1
                if self.inferred_cnt == sys.maxsize:
                    self.inferred_request = [0] * self.num_requests
                    self.heap_request = []
                    self.inferred_cnt = 0
                heapq.heappush(self.heap_request, (self.inferred_cnt, reqnum))

            cnt, dev = heapq.heappop(self.heap_request)

            # check if device is ready
            if self.exec_net.requests[dev].wait(0) == 0:
                self.exec_net.requests[dev].wait(-1)

                objects = []
                outputs = self.exec_net.requests[dev].outputs
                for output in outputs.values():
                    objects = parse_yolov3_output_classes_only(output, self.threshold)

                objlen = len(objects)
                for i in range(objlen):
                    if objects[i].confidence == 0.0:
                        continue
                    for j in range(i + 1, objlen):
                        if self.intersection_over_union(objects[i], objects[j]) >= 0.4:
                            if objects[i].confidence < objects[j].confidence:
                                objects[i], objects[j] = objects[j], objects[i]
                            objects[j].confidence = 0.0

                self.results.put(objects)
                self.inferred_request[dev] = 0
            else:
                heapq.heappush(self.heap_request, (cnt, dev))
        except:
            import traceback
            traceback.print_exc()


class ImageAnalyzer:


    def __init__(self):

        self.running = True  # set to False to stop all processes
        self.search_high_signals = True # set to false to search for low signals
        self.m_input_size = 416
        self.results = mp.Queue()


    def prepare_image_for_processing(self, color_image):
        # resized_image = color_image
        # resized_image = cv2.resize(color_image, (self.new_w, self.new_h), interpolation = cv2.INTER_CUBIC)

        # resize image to shape 416x416 by cutting off the edges
        # the images should already be 416x416 from the PiCamera.
        resized_image = []
        for row in color_image[:416]:
            resized_image.append(row[:416])

        # create all black 416x416 image
        canvas = np.full((self.m_input_size, self.m_input_size, 3), 0)
        
        if self.search_high_signals:
            # searching for info and start signals in upper half of the image
            canvas[:208, :416, :] = resized_image
        else:
            # stop signals are in the lower half of the image
            canvas[208:416, :416, :] = resized_image

        prepimg = canvas
        prepimg = prepimg[np.newaxis, :, :, :]     # Batch size axis add
        prepimg = prepimg.transpose((0, 3, 1, 2))  # NHWC to NCHW

        return prepimg

    def cam_thread(self, frame_buffer):
        """
        Args:
            frame_buffer: the queue for the images.
            frame_buffer type: Queue[Any]
        """

        # initialize USB camera
        '''
        cam = cv2.VideoCapture(0)
        if cam.isOpened() != True:
            print("USB Camera Open Error!!!")
            self.running = False
        cam.set(cv2.CAP_PROP_FPS, 15)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
        '''
        #img = cv2.imread("/home/larry/work/tiny-yolov3-on-intel-neural-compute-stick-2/images/5-1.jpg")
        #img = cv2.imread("/home/pi/pren2/src/boardcomputer/image_analysis/5-1_416.jpg")

        # initialise PiCamera
        base_config = BaseConfig()

        camera = PiCamera()
        camera.resolution = base_config.CAMERA_RESOLUTION
        camera.brightness = base_config.CAMERA_BRIGHTNESS
        camera.framerate = base_config.CAMERA_FRAMERATE
        camera.shutter_speed = base_config.CAMERA_SHUTTERSPEED
        camera.exposure_mode = base_config.CAMERA_EXPOSUREMODE
        camera.rotation = base_config.CAMERA_ROTATION
        camera.iso = base_config.CAMERA_ISO

        time.sleep(2)

        color_image = np.empty((camera.resolution[1], camera.resolution[0], 3), dtype=np.uint8)

        while self.running:

            # USB Camera Stream Read
            '''
            s, color_image = cam.read()
            if not s:
                continue
            '''

            # PyCam Stream read
            camera.capture(color_image, format='bgr', use_video_port=True)
            
            # if frame buffer full, remove oldest
            if frame_buffer.full():
                frame_buffer.get()

            preprocessed_image = self.prepare_image_for_processing(color_image.copy())
            frame_buffer.put(preprocessed_image)


    def async_detection(self, ncs_worker):

        while True:
            ncs_worker.predict_async()


    def detect_signals(self, results, frame_buffer, video_fps):
        """
        Args:
            results: the queue to write results of the inference into.
            frame_buffer: the queue containing images from the cam_thread.
            video_fps: the speed at which the camera takes images.
        """

        # Init infer thread
        detection_thread = threading.Thread(
            target=self.async_detection,
            args=(NcsWorker(frame_buffer, results, video_fps),))
        detection_thread.start()
        detection_thread.join()  # wait for the thread to terminate


    def initialize(self):

        video_fps = 15

        mp.set_start_method('forkserver')
        frame_buffer = mp.Queue(10)

        # Start async signal detection

        # Activation the detection algorithm
        p = mp.Process(target=self.detect_signals, args=(self.results, frame_buffer, video_fps), daemon=True)
        p.start()
        processes.append(p)

        time.sleep(7)

        # Start filling frame buffer from camera
        p = mp.Process(target=self.cam_thread, args=(frame_buffer,), daemon=True)
        p.start()
        processes.append(p)


    def detect_signal(self) -> Signal:

        t = time.time()
        while True:            
            try:
                if self.results.qsize() > 0:
                    r = self.results.get()
                    if len(r) > 0:
                        print('class: {}'.format(r[0].class_id))
                        if (self.search_high_signals):
                             return LABELS_TMS_INFO[r[0].class_id]
                        else:
                             return LABELS_TMS_STOP[r[0].class_id]
                    print('{:04f}, FPS: {}'.format(time.time() - t, 1/(time.time() - t)))
                    t = time.time()
            except KeyboardInterrupt:
                self.running = False
                sys.exit(0)


if __name__ == "__main__":
    '''
    Only for testing.
    '''
    ia = ImageAnalyzer()
    ia.initialize()
    while True:
        ia.detect_signal()
