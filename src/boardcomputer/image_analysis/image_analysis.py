from fsm.signals import Signal
import os, sys, heapq
import cv2
import time
import numpy as np, math
import logging
from concurrent.futures import ThreadPoolExecutor
import multiprocessing as mp
import threading
from picamera import PiCamera
from fsm.config import BaseConfig
import socketio
import base64

try:
    # Raspberry
    sys.path.insert(0, '/opt/intel/openvino/python/python3.5')
    from armv7l.openvino.inference_engine import IENetwork, IEPlugin
    from fsm.config import BaseImageAnalysisNCS2Config as config
except:
    # Dev PC
    from openvino.inference_engine import IENetwork, IEPlugin
    from fsm.config import BaseImageAnalysisNCS2Config as config

m_input_size = 416

yolo_scale_13 = 13
yolo_scale_26 = 26
yolo_scale_52 = 52

classes = 7
coords = 4
num = 3
anchors = [10,14, 23,27, 37,58, 81,82, 135,169, 344,319]

LABELS = ("info-3", "info-9", "info-4", "stop-5", "stop-2", "stop-9", "info-1")

processes = []

label_text_color = (255, 255, 255)
label_background_color = (125, 175, 75)
box_color = (255, 128, 0)
box_thickness = 1

def EntryIndex(side, lcoords, lclasses, location, entry):
    n = int(location / (side * side))
    loc = location % (side * side)
    return int(n * side * side * (lcoords + lclasses + 1) + entry * side * side + loc)
    
def ParseYOLOV3OutputFaster(blob, threshold):

    objects = []
    out_blob_h = blob.shape[2]

    side = out_blob_h # == 13

    side_square = side * side
    output_blob = blob.flatten()

    for i in range(side_square):
        for n in range(num):
            obj_index = EntryIndex(side, coords, classes, n * side * side + i, coords)
            scale = output_blob[obj_index]
            if (scale < threshold):
                continue
            for j in range(classes):
                class_index = EntryIndex(side, coords, classes, n * side_square + i, coords + 1 + j)
                prob = scale * output_blob[class_index]
                if prob < threshold:
                    continue
                obj = DetectionObjectFaster(j, prob)
                objects.append(obj)
    return objects

class DetectionObject():
    xmin = 0
    ymin = 0
    xmax = 0
    ymax = 0
    class_id = 0
    confidence = 0.0

    def __init__(self, x, y, h, w, class_id, confidence, h_scale, w_scale):
        self.xmin = int((x - w / 2) * w_scale)
        self.ymin = int((y - h / 2) * h_scale)
        self.xmax = int(self.xmin + w * w_scale)
        self.ymax = int(self.ymin + h * h_scale)
        self.class_id = class_id
        self.confidence = confidence

class DetectionObjectFaster():
    class_id = 0
    confidence = 0.0

    def __init__(self, class_id, confidence):
        self.class_id = class_id
        self.confidence = confidence


class NcsWorker(object):


    def __init__(self, devid, frameBuffer, results, camera_width, camera_height, number_of_ncs, vidfps):
        self.devid = devid
        self.frameBuffer = frameBuffer
        self.model_xml = "image_analysis/model/{}.xml".format(config.MODEL_NAME)
        self.model_bin = "image_analysis/model/{}.bin".format(config.MODEL_NAME)
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.m_input_size = 416
        self.threshould = 0.4
        self.num_requests = 4
        self.inferred_request = [0] * self.num_requests
        self.heap_request = []
        self.inferred_cnt = 0
        self.plugin = IEPlugin(device="MYRIAD")
        self.net = IENetwork(model=self.model_xml, weights=self.model_bin)
        self.input_blob = next(iter(self.net.inputs))
        self.exec_net = self.plugin.load(network=self.net, num_requests=self.num_requests)
        self.results = results
        self.number_of_ncs = number_of_ncs
        self.predict_async_time = 800
        self.skip_frame = 0
        self.roop_frame = 0
        self.vidfps = vidfps
        #self.new_w = int(camera_width * min(self.m_input_size/camera_width, self.m_input_size/camera_height))
        #self.new_h = int(camera_height * min(self.m_input_size/camera_width, self.m_input_size/camera_height))
        self.new_w = 416
        self.new_h = 416

        #self._socketio = socketio.Client()
        #self._socketio.connect('http://192.168.0.1:5000')
        

    # l = Search list
    # x = Search target value
    def searchlist(self, l, x, notfoundvalue=-1):
        if x in l:
            return l.index(x)
        else:
            return notfoundvalue


    def showPic(self, img):
        image = cv2.imencode(".jpeg", img)[1]

        b64_bytes = base64.b64encode(image)
        b64_str = b64_bytes.decode()

        #self._socketio.emit('pictureSet', b64_str)


    def image_preprocessing(self, color_image):
        #resized_image = color_image
        ##resized_image = cv2.resize(color_image, (self.new_w, self.new_h), interpolation = cv2.INTER_CUBIC)
        resized_image = []
        for row in color_image[:416]:
            resized_image.append(row[:416])

        canvas = np.full((self.m_input_size, self.m_input_size, 3), 0)
        canvas[:416,:416, :] = resized_image
        prepimg = canvas
        prepimg = prepimg[np.newaxis, :, :, :]     # Batch size axis add
        prepimg = prepimg.transpose((0, 3, 1, 2))  # NHWC to NCHW

        #self.showPic(prepimg.copy())
        return prepimg


    def skip_frame_measurement(self):
            surplustime_per_second = (1000 - self.predict_async_time)
            if surplustime_per_second > 0.0:
                frame_per_millisecond = (1000 / self.vidfps)
                total_skip_frame = surplustime_per_second / frame_per_millisecond
                self.skip_frame = int(total_skip_frame / self.num_requests)
            else:
                self.skip_frame = 0


    def predict_async(self):
        try:

            if self.frameBuffer.empty():
                return

            self.roop_frame += 1
            if self.roop_frame <= self.skip_frame:
               self.frameBuffer.get()
               return
            self.roop_frame = 0

            prepimg = self.image_preprocessing(self.frameBuffer.get())
            reqnum = self.searchlist(self.inferred_request, 0)

            if reqnum > -1:
                self.exec_net.start_async(request_id=reqnum, inputs={self.input_blob: prepimg})
                self.inferred_request[reqnum] = 1
                self.inferred_cnt += 1
                if self.inferred_cnt == sys.maxsize:
                    self.inferred_request = [0] * self.num_requests
                    self.heap_request = []
                    self.inferred_cnt = 0
                heapq.heappush(self.heap_request, (self.inferred_cnt, reqnum))

            cnt, dev = heapq.heappop(self.heap_request)

            if self.exec_net.requests[dev].wait(0) == 0:
                self.exec_net.requests[dev].wait(-1)

                objects = []
                outputs = self.exec_net.requests[dev].outputs
                for output in outputs.values():
                    objects = ParseYOLOV3OutputFaster(output, self.threshould)

                objlen = len(objects)
                for i in range(objlen):
                    if (objects[i].confidence == 0.0):
                        continue
                    for j in range(i + 1, objlen):
                        if (IntersectionOverUnion(objects[i], objects[j]) >= 0.4):
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

        self.running = True

    def IntersectionOverUnion(self, box_1, box_2):
        width_of_overlap_area = min(box_1.xmax, box_2.xmax) - max(box_1.xmin, box_2.xmin)
        height_of_overlap_area = min(box_1.ymax, box_2.ymax) - max(box_1.ymin, box_2.ymin)
        area_of_overlap = 0.0
        if (width_of_overlap_area < 0.0 or height_of_overlap_area < 0.0):
            area_of_overlap = 0.0
        else:
            area_of_overlap = width_of_overlap_area * height_of_overlap_area
        box_1_area = (box_1.ymax - box_1.ymin)  * (box_1.xmax - box_1.xmin)
        box_2_area = (box_2.ymax - box_2.ymin)  * (box_2.xmax - box_2.xmin)
        area_of_union = box_1_area + box_2_area - area_of_overlap
        retval = 0.0
        if area_of_union <= 0.0:
            retval = 0.0
        else:
            retval = (area_of_overlap / area_of_union)
        return retval


    def ParseYOLOV3Output(self, blob, resized_im_h, resized_im_w, original_im_h, original_im_w, threshold):

        objects = []
        out_blob_h = blob.shape[2]

        side = out_blob_h
        anchor_offset = 0

        if side == yolo_scale_13:
            anchor_offset = 2 * 3
        elif side == yolo_scale_26:
            anchor_offset = 2 * 0        

        side_square = side * side
        output_blob = blob.flatten()

        for i in range(side_square):
            row = int(i / side)
            col = int(i % side)
            for n in range(num):
                obj_index = EntryIndex(side, coords, classes, n * side * side + i, coords)
                box_index = EntryIndex(side, coords, classes, n * side * side + i, 0)
                scale = output_blob[obj_index]
                if (scale < threshold):
                    continue
                x = (col + output_blob[box_index + 0 * side_square]) / side * resized_im_w
                y = (row + output_blob[box_index + 1 * side_square]) / side * resized_im_h
                height = math.exp(output_blob[box_index + 3 * side_square]) * anchors[anchor_offset + 2 * n + 1]
                width = math.exp(output_blob[box_index + 2 * side_square]) * anchors[anchor_offset + 2 * n]
                for j in range(classes):
                    class_index = EntryIndex(side, coords, classes, n * side_square + i, coords + 1 + j)
                    prob = scale * output_blob[class_index]
                    if prob < threshold:
                        continue
                    obj = DetectionObject(x, y, height, width, j, prob, (original_im_h / resized_im_h), (original_im_w / resized_im_w))
                    objects.append(obj)
        return objects


    def camThread(self, frameBuffer, camera_width, camera_height, vidfps):

        '''
        cam = cv2.VideoCapture(0)
        if cam.isOpened() != True:
            print("USB Camera Open Error!!!")
            self.running = False
        cam.set(cv2.CAP_PROP_FPS, vidfps)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
        '''
        #img = cv2.imread("/home/larry/work/tiny-yolov3-on-intel-neural-compute-stick-2/images/5-1.jpg")
        #img = cv2.imread("/home/pi/pren2/src/boardcomputer/image_analysis/5-1_416.jpg")

        baseConfig = BaseConfig()

        _camera = PiCamera()
        _camera.resolution = baseConfig.CAMERA_RESOLUTION
        _camera.brightness = baseConfig.CAMERA_BRIGHTNESS
        _camera.framerate = baseConfig.CAMERA_FRAMERATE
        _camera.shutter_speed = baseConfig.CAMERA_SHUTTERSPEED
        _camera.exposure_mode = baseConfig.CAMERA_EXPOSUREMODE
        _camera.rotation = baseConfig.CAMERA_ROTATION
        # _camera.color_effects = (128 , 128)
        _camera.iso = baseConfig.CAMERA_ISO

        time.sleep(2)

        color_image = np.empty((_camera.resolution[1], _camera.resolution[0], 3), dtype=np.uint8)

        while self.running:

            # USB Camera Stream Read
            '''
            s, color_image = cam.read()
            if not s:
                continue
            '''

            # PyCam Stream read
            _camera.capture(color_image, format='bgr', use_video_port=True)
            
            # if frame buffer full, remove oldest
            if frameBuffer.full():
                frameBuffer.get()
            

            frameBuffer.put(color_image.copy())


    def async_infer(self, ncsworker):

        while True:
            ncsworker.predict_async()


    def inferencer(self, results, frameBuffer, number_of_ncs, camera_width, camera_height, vidfps):

        # Init infer threads
        device_id = 0
        threads = []
        thworker = threading.Thread(target=self.async_infer, args=(NcsWorker(device_id, frameBuffer, results, camera_width, camera_height, number_of_ncs, vidfps),))
        thworker.start()
        threads.append(thworker)

        for th in threads:
            th.join()


    def initialize_async(self):

        self.camera_width = 640
        self.camera_height = 480
        vidfps = 15
        number_of_ncs = 1

        mp.set_start_method('forkserver')
        frameBuffer = mp.Queue(10)
        results = mp.Queue()

        # Start async signal detection
        # Activation of inferencer
        p = mp.Process(target=self.inferencer, args=(results, frameBuffer, number_of_ncs, self.camera_width, self.camera_height, vidfps), daemon=True)
        p.start()
        processes.append(p)

        time.sleep(number_of_ncs * 7)

        # Start streaming
        p = mp.Process(target=self.camThread, args=(frameBuffer, self.camera_width, self.camera_height, vidfps), daemon=True)
        p.start()
        processes.append(p)

        t = time.time()
        while True:            
            try:
                if results.qsize() > 0:
                    r = results.get()
                    if len(r) > 0:
                        print ('class: {}'.format(r[0].class_id))
                    print ('{:04f}, FPS: {}'.format(time.time() - t, 1/(time.time() - t)))
                    t = time.time()
            except KeyboardInterrupt:
                self.running = False
                sys.exit(0)


    def initialize(self):
        

        baseConfig = BaseConfig()

        _camera = PiCamera()
        _camera.resolution = baseConfig.CAMERA_RESOLUTION
        _camera.brightness = baseConfig.CAMERA_BRIGHTNESS
        _camera.framerate = baseConfig.CAMERA_FRAMERATE
        _camera.shutter_speed = baseConfig.CAMERA_SHUTTERSPEED
        _camera.exposure_mode = baseConfig.CAMERA_EXPOSUREMODE
        _camera.rotation = baseConfig.CAMERA_ROTATION
        # _camera.color_effects = (128 , 128)
        _camera.iso = baseConfig.CAMERA_ISO

        time.sleep(2)

        self.cam = _camera
        self.raw_image = np.empty((_camera.resolution[1], _camera.resolution[0], 3), dtype=np.uint8)
        self.cam.capture(self.raw_image, format='bgr', use_video_port=True)

        self.camera_width = 800
        self.camera_height = 600
        self.new_w = int(self.camera_width * min(m_input_size/self.camera_width, m_input_size/self.camera_height))
        self.new_h = int(self.camera_height * min(m_input_size/self.camera_width, m_input_size/self.camera_height))

        model_xml = "image_analysis/model/{}.xml".format(config.MODEL_NAME)
        model_bin = "image_analysis/model/{}.bin".format(config.MODEL_NAME)

        '''
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FPS, 30)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.cam.read()
        '''
        
        self.plugin = IEPlugin(device=config.DEVICE)
        if "CPU" in config.DEVICE:
            self.plugin.add_cpu_extension("libcpu_extension_sse4.so")
        self.net = IENetwork(model=model_xml, weights=model_bin)
        self.input_blob = next(iter(self.net.inputs))
        self.exec_net = self.plugin.load(network=self.net)


    def analyze_image(self):
        """
        Analyzes an image to recognize different types of signals.
        Args:
            image: An image taken by the camera.

        Returns:
            The outcome of the image analysis as an AnalysisOutcome.
        """
        
        start_time = time.time()
        initial_time = start_time

        #img = cv2.imread("/home/larry/work/tiny-yolov3-on-intel-neural-compute-stick-2/images/5-1.jpg")
        
        '''
        success, img = self.cam.read()
        if (success == False):
            logging.getLogger().warn('no image from cam. is the camera connected?')
            return None
        '''
        self.cam.capture(self.raw_image, format='bgr', use_video_port=True)

        #print('\nget image: {0:.5f}s'.format(time.time() - start_time))
        #start_time = time.time()

        resized_image = []
        for row in self.raw_image[:416]:
            resized_image.append(row[:416])

        canvas = np.full((m_input_size, m_input_size, 3), 0)
        canvas[:416,:416, :] = resized_image
        prepimg = canvas
        prepimg = prepimg[np.newaxis, :, :, :]     # Batch size axis add
        prepimg = prepimg.transpose((0, 3, 1, 2))  # NHWC to NCHW
        outputs = self.exec_net.infer(inputs={self.input_blob: prepimg})

        #print('neural network: {0:.5f}s'.format(time.time() - start_time))
        #start_time = time.time()

        objects = []

        executor = ThreadPoolExecutor(max_workers=4)
        results = []
        for output in outputs.values():
            if not config.DRAW_RECTANGLES:
                promise = executor.submit(self.ParseYOLOV3Output, output, self.new_h, self.new_w, self.camera_height, self.camera_width, 0.4)
                results.append(promise)
            else:
                promise = executor.submit(self.ParseYOLOV3Output, output, self.new_h, self.new_w, self.camera_height, self.camera_width, 0.4)
                results.append(promise)

        for r in results:
            objects.extend(r.result())

        for object in objects:
            print(LABELS[object.class_id], object.confidence, "%")

        #print('parse output: {0:.5f}s'.format(time.time() - start_time))
        #start_time = time.time()

        # Filtering overlapping boxes
        objlen = len(objects)
        for i in range(objlen):
            if (objects[i].confidence == 0.0):
                continue
            for j in range(i + 1, objlen):
                if (self.IntersectionOverUnion(objects[i], objects[j]) >= 0.4):
                    if objects[i].confidence < objects[j].confidence:
                        objects[i], objects[j] = objects[j], objects[i]
                    objects[j].confidence = 0.0

        #print('filter overlapping boxes: {0:.5f}s'.format(time.time() - start_time))
        #start_time = time.time()

        # Drawing boxes
        for obj in objects:
            if obj.confidence < 0.02:
                continue
            label = obj.class_id
            confidence = obj.confidence
            #if confidence >= 0.2:
            label_text = LABELS[label] + " (" + "{:.1f}".format(confidence * 100) + "%)"
            if not config.DRAW_RECTANGLES:
                cv2.rectangle(self.raw_image, (obj.xmin, obj.ymin), (obj.xmax, obj.ymax), box_color, box_thickness)
                cv2.putText(self.raw_image, label_text, (obj.xmin, obj.ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, label_text_color, 1)
                cv2.imwrite("/home/pi/boardcomputer/image_analysis/Result.jpg", self.raw_image)

        #print('draw boxes: {0:.5f}s'.format(time.time() - start_time))
        #print('total time: {0:.5f}s'.format(time.time() - initial_time))
        print('FPS: {0:.2f}'.format(1 / (time.time() - initial_time)))

        #cv2.imwrite("/home/pi/boardcomputer/image_analysis/Result.jpg", self.raw_image)

    def Dispose(self):
        '''
        Delete stuff.
        '''
        del self.net
        del self.exec_net
        del self.plugin

if __name__ == "__main__":
    '''
    Only for testing.
    '''

    ia = ImageAnalyzer()
    #ia.initialize_async()
    ia.initialize_async()
    ia.analyze_image()