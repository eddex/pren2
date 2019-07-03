import sys
[sys.path.append(i) for i in ['.', '..']]

import os
import heapq
import cv2
import time
import numpy as np
import logging
import multiprocessing as mp
from fsm.config import BaseConfig
from fsm.signals import Signal

try:
    # Raspberry
    sys.path.insert(0, '/opt/intel/openvino/python/python3.5')
    from armv7l.openvino.inference_engine import IENetwork, IEPlugin
    try:
        from picamera import PiCamera
    except:
        # no PyCamera, such a shame. Use fake_cam_thread()
        pass
    from fsm.config import BaseImageAnalysisNCS2Config as config
except:
    # Dev PC
    from openvino.inference_engine import IENetwork, IEPlugin
    from fsm.config import BaseImageAnalysisCPUConfig as config

yolo_scale_13 = 13
yolo_scale_26 = 26
yolo_scale_52 = 52

classes = 10
coords = 4
num = 3
anchors = [10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319]

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
    "signal-start")

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


class DetectionObjectFaster:

    def __init__(self, class_id, confidence):
        self.class_id = class_id
        self.confidence = confidence


class NcsWorker(object):


    def __init__(self, image_queue, results):
        self.image_queue = image_queue
        self.model_xml = "image_analysis/model/{}.xml".format(config.MODEL_NAME)
        self.model_bin = "image_analysis/model/{}.bin".format(config.MODEL_NAME)
        self.threshold = 0.4
        self.num_requests = 2
        self.inferred_request = [0] * self.num_requests
        self.heap_request = []
        self.inferred_cnt = 0
        self.plugin = IEPlugin(device=config.DEVICE)
        self.net = IENetwork(model=self.model_xml, weights=self.model_bin)
        self.input_blob = next(iter(self.net.inputs))
        self.exec_net = self.plugin.load(network=self.net, num_requests=self.num_requests)
        self.results = results
        

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


    def predict_synchronous(self):
        if self.image_queue.empty():
                return

        print("got image")
        prepared_image = self.image_queue.get()
        outputs = self.exec_net.infer(inputs={self.input_blob: prepared_image})

        objects = []
        for output in outputs.values():
            objects = parse_yolov3_output_classes_only(output, 0.4)
        
        if len(objects) > 0:
            self.results.put(objects)


    def predict_async(self):
        try:
            if self.image_queue.empty():
                return

            print("got image")
            prepared_image = self.image_queue.get()
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
                
                if len(objects) > 0:
                    self.results.put(objects)
                self.inferred_request[dev] = 0
            else:
                print("device {} not ready".format(dev))
                heapq.heappush(self.heap_request, (cnt, dev))    
        except:
            import traceback
            traceback.print_exc()


class ImageAnalyzer:


    def __init__(self):

        self.running = True  # set to False to stop all processes
        self.search_high_signals = True  # set to false to search for low signals
        self.input_size = 416
        self.results = mp.Queue()
        self.processes = []


    def prepare_image_for_processing(self, color_image):
        """
        Cut off the lower or higher part of the image, depoending on if we
            search for high or low signals.
        Also transforms the image into the correct format for the NCS2.
        """
        # EXPECTATION: the images should already be 416x416 from the PiCamera.

        # create all green 416x416 image
        canvas = np.full((self.input_size, self.input_size, 3), [0,255,0])
        
        if self.search_high_signals:
            # searching for info and start signals in upper half of the image
            canvas[:208, :416, :] = color_image[:208, :416]
        else:
            # stop signals are in the lower half of the image
            canvas[208:416, :416, :] = color_image[208:416, :416, :]

        prepimg = canvas
        prepimg = prepimg[np.newaxis, :, :, :]     # Batch size axis add
        prepimg = prepimg.transpose((0, 3, 1, 2))  # NHWC to NCHW

        #cv2.imwrite("lol.jpg", canvas)

        return prepimg

    def prepare_image_for_processing_double_image(self, color_image_1, color_image_2):
        """
        Combines 2 images to one. Like this we can process 2 images at the same time.
        We only need to look at the top or bottom half of each image, since we only search
            for high or low signals at the same time.
        Also transforms the image into the correct format for the NCS2.
        """
        # EXPECTATION: the images should already be 416x416 from the PiCamera.

        # create all black 416x416 image
        canvas = np.full((self.input_size, self.input_size, 3), 0)
        
        if self.search_high_signals:
            # combine the 2 upper halfs of the image to one image
            canvas[:208, :416, :] = color_image_1[:208, :416]
            canvas[208:416, :416, :] = color_image_2[:208, :416]
        else:
            # combine the 2 lower halfs of the image to one image
            canvas[:208, :416, :] = color_image_1[208:416, :416, :]
            canvas[208:416, :416, :] = color_image_2[208:416, :416, :]
        #cv2.imwrite("/home/pi/boardcomputer/image_analysis/prepared-image{}.jpg".format(time.time()), img=canvas)
        #time.sleep(1)
        prepimg = canvas
        prepimg = prepimg[np.newaxis, :, :, :]     # Batch size axis add
        prepimg = prepimg.transpose((0, 3, 1, 2))  # NHWC to NCHW

        #cv2.imwrite("double.jpg", canvas)

        return prepimg


    def fake_cam_thread(self, image_queue):
        """
        Pretends to do the same as the cam_thread() but it's fake!
        Can be used if no PyCamera is in the reach of the developer.
        """
        img = cv2.imread("./image_analysis/signal-5431.jpg")
        preprocessed_image_1 = self.prepare_image_for_processing(img.copy())
        
        img = cv2.imread("./image_analysis/signal-50.jpg")
        preprocessed_image_2 = self.prepare_image_for_processing(img.copy())
        
        img_3 = cv2.imread("./image_analysis/signal-5038.jpg")
        preprocessed_image_3 = self.prepare_image_for_processing(img_3.copy())
        
        img_4 = cv2.imread("./image_analysis/signal-835.jpg")
        preprocessed_image_4 = self.prepare_image_for_processing(img_4.copy())

        preprocessed_image_double = self.prepare_image_for_processing_double_image(img_3, img_4)
        
        while self.running:
            #image_queue.put(preprocessed_image_1)
            #image_queue.put(preprocessed_image_2)

            #image_queue.put(preprocessed_image_3)
            #image_queue.put(preprocessed_image_4)

            image_queue.put(preprocessed_image_double)


    def cam_thread(self, image_queue):
        """
        Capture images with the PiCamera.
        Does preprocessing on the images to optimize them for the neural network.
        Args:
            image_queue: the queue for the images.
        """

        # initialise PiCamera
        base_config = BaseConfig()
        camera = PiCamera()
        camera.exposure_mode = 'sports'
        camera.exposure_compensation = -10
        camera.resolution = (416, 416)
        camera.brightness = 55
        camera.framerate = 30
        camera.shutter_speed = 400
        camera.rotation = 0
        camera.iso = 1200

        time.sleep(2)

        images = []
        while self.running:
            
            color_image = np.empty((camera.resolution[1], camera.resolution[0], 3), dtype=np.uint8)
            
            # PyCam Stream read
            camera.capture(color_image, format='bgr', use_video_port=True)
            images.append(color_image.copy())

            if len(images) == 2:
                # if image queue full, remove oldest
                if image_queue.full():
                    image_queue.get()
                preprocessed_image = self.prepare_image_for_processing_double_image(images[0], images[1])
                image_queue.put(preprocessed_image)
                images = []


    def detect_signals(self, results, image_queue):
        """
        Args:
            results: the queue to write results of the inference into.
            frame_buffer: the queue containing images from the cam_thread.
            video_fps: the speed at which the camera takes images.
        """
        worker = NcsWorker(image_queue, results)
        while self.running:
            worker.predict_async()
            #worker.predict_synchronous()


    def initialize(self):

        try:
            # this fails somethimes because the start method is already set
            mp.set_start_method('forkserver')
        except:
            # that's ok :)
            pass
        
        image_queue = mp.Queue(10)
        
        # Start async signal detection

        # Activation the detection algorithm
        p = mp.Process(target=self.detect_signals, args=(self.results, image_queue), daemon=True)
        p.start()
        self.processes.append(p)

        time.sleep(7)

        # Start filling frame buffer from camera
        p = mp.Process(target=self.cam_thread, args=(image_queue,), daemon=True)
        p.start()
        self.processes.append(p)

        log = logging.getLogger()
        log.info("ImageAnalyzer initialized.")
        

    def detect_signal(self) -> Signal:

        t = time.time()
        while True:
            try:
                if self.results.qsize() > 0:
                    results_set = self.results.get()
                    if len(results_set) > 0:
                        best_result = results_set[0]
                        for r in results_set:
                            print(LABELS[r.class_id], r.confidence, "%")
                            if r.confidence > best_result.confidence:
                                best_result = r
                        
                        print('CLASS: {}, CONFIDENCE {:.2f}%'.format(LABELS[best_result.class_id], best_result.confidence * 100))
                        # FPS * 2 because we process 2 images at the same time!
                        print('TIME: {:4f}s, FPS: {:4f}'.format(time.time() - t, 1/(time.time() - t)*2))
                        t = time.time()
                        if (self.search_high_signals):
                             return LABELS_TMS_INFO[best_result.class_id]
                        else:
                             return LABELS_TMS_STOP[best_result.class_id]
            except KeyboardInterrupt:
                self.running = False
                sys.exit(0)


    def stop_everything(self):
        self.running = False


if __name__ == "__main__":
    '''
    Only for testing.
    '''
    mp.set_start_method('forkserver')
    ia = ImageAnalyzer()


    ia.search_high_signals = True
    ia.initialize()
    time.sleep(5)
    while True:
        ia.detect_signal()
