from fsm.signals import Signal
import os
import cv2
import time
import numpy as np, math
import logging
from concurrent.futures import ThreadPoolExecutor

try:
    # Raspberry
    from armv7l.openvino.inference_engine import IENetwork, IEPlugin
    from fsm.config import BaseImageAnalysisNCS2Config as config
except:
    # Dev PC
    from openvino.inference_engine import IENetwork, IEPlugin
    from fsm.config import BaseImageAnalysisCPUConfig as config

m_input_size = 416

yolo_scale_13 = 13
yolo_scale_26 = 26
yolo_scale_52 = 52

classes = 7
coords = 4
num = 3
anchors = [10,14, 23,27, 37,58, 81,82, 135,169, 344,319]

LABELS = ("info-3", "info-9", "info-4", "stop-5", "stop-2", "stop-9", "info-1")

label_text_color = (255, 255, 255)
label_background_color = (125, 175, 75)
box_color = (255, 128, 0)
box_thickness = 1

def EntryIndex(side, lcoords, lclasses, location, entry):
    n = int(location / (side * side))
    loc = location % (side * side)
    return int(n * side * side * (lcoords + lclasses + 1) + entry * side * side + loc)
    

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

class ImageAnalyzer:


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


    def ParseYOLOV3OutputFaster(self, blob, threshold):

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


    def initialize(self):
        
        self.camera_width = 800
        self.camera_height = 600
        self.new_w = int(self.camera_width * min(m_input_size/self.camera_width, m_input_size/self.camera_height))
        self.new_h = int(self.camera_height * min(m_input_size/self.camera_width, m_input_size/self.camera_height))

        model_xml = "image_analysis/model/{}.xml".format(config.MODEL_NAME)
        model_bin = "image_analysis/model/{}.bin".format(config.MODEL_NAME)

        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FPS, 30)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.cam.read()

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
        
        
        success, img = self.cam.read()
        if (success == False):
            logging.getLogger().warn('no image from cam. is the camera connected?')
            return None
        

        print('\nget image: {0:.5f}s'.format(time.time() - start_time))
        start_time = time.time()

        resized_image = cv2.resize(img, (self.new_w, self.new_h), interpolation = cv2.INTER_CUBIC)
        canvas = np.full((m_input_size, m_input_size, 3), 128)
        canvas[(m_input_size-self.new_h)//2:(m_input_size-self.new_h)//2 + self.new_h,(m_input_size-self.new_w)//2:(m_input_size-self.new_w)//2 + self.new_w,  :] = resized_image
        prepimg = canvas
        prepimg = prepimg[np.newaxis, :, :, :]     # Batch size axis add
        prepimg = prepimg.transpose((0, 3, 1, 2))  # NHWC to NCHW
        outputs = self.exec_net.infer(inputs={self.input_blob: prepimg})

        print('neural network: {0:.5f}s'.format(time.time() - start_time))
        start_time = time.time()

        objects = []

        executor = ThreadPoolExecutor(max_workers=4)
        results = []
        for output in outputs.values():
            if not config.DRAW_RECTANGLES:
                promise = executor.submit(self.ParseYOLOV3OutputFaster, output, 0.4)
                results.append(promise)
            else:
                promise = executor.submit(self.ParseYOLOV3Output, output, self.new_h, self.new_w, self.camera_height, self.camera_width, 0.4)
                results.append(promise)

        for r in results:
            objects.extend(r.result())

        for object in objects:
            print(LABELS[object.class_id], object.confidence, "%")

        print('parse output: {0:.5f}s'.format(time.time() - start_time))
        start_time = time.time()

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

        print('filter overlapping boxes: {0:.5f}s'.format(time.time() - start_time))
        start_time = time.time()

        # Drawing boxes
        for obj in objects:
            if obj.confidence < 0.02:
                continue
            label = obj.class_id
            confidence = obj.confidence
            #if confidence >= 0.2:
            label_text = LABELS[label] + " (" + "{:.1f}".format(confidence * 100) + "%)"
            if config.DRAW_RECTANGLES:
                cv2.rectangle(img, (obj.xmin, obj.ymin), (obj.xmax, obj.ymax), box_color, box_thickness)
                cv2.putText(img, label_text, (obj.xmin, obj.ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, label_text_color, 1)

        print('draw boxes: {0:.5f}s'.format(time.time() - start_time))
        print('total time: {0:.5f}s'.format(time.time() - initial_time))
        print('FPS: {0:.2f}\n'.format(1 / (time.time() - initial_time)))

        cv2.imwrite("Result.jpg", img)

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
    ia.initialize()
    ia.analyze_image()