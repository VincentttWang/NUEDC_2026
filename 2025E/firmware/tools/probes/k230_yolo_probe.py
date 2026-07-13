"""Finite COCO YOLOv8n probe using a second K230 sensor channel."""

import gc
import time

import aidemo
import nncase_runtime as nn
import ulab.numpy as np
from libs.AI2D import Ai2d
from libs.AIBase import AIBase
from libs.Utils import letterbox_pad_param
from media.media import *
from media.sensor import *


LABELS = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
    "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
    "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
]


class CocoDetector(AIBase):
    def __init__(self):
        model_size = [224, 224]
        rgb_size = [224, 224]
        super().__init__(
            "/sdcard/examples/kmodel/yolov8n_224.kmodel",
            model_size,
            rgb_size,
            0,
        )
        self.model_input_size = model_size
        self.rgb888p_size = rgb_size
        self.ai2d = Ai2d(0)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8,
        )

    def config_preprocess(self):
        top, bottom, left, right, _ = letterbox_pad_param(
            self.rgb888p_size, self.model_input_size)
        self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [128, 128, 128])
        self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
        self.ai2d.build([1, 3, 224, 224], [1, 3, 224, 224])

    def preprocess(self, input_np):
        return [nn.from_numpy(input_np)]

    def postprocess(self, results):
        output = results[0][0].transpose()
        return aidemo.yolov8_det_postprocess(
            output.copy(), [224, 224], [224, 224], [240, 320],
            len(LABELS), 0.20, 0.45, 30)


sensor = None
detector = None
try:
    sensor = Sensor(width=320, height=240)
    sensor.reset()
    sensor.set_framesize(width=320, height=240, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
    sensor.set_framesize(width=224, height=224, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
    MediaManager.init()
    sensor.run()

    detector = CocoDetector()
    detector.config_preprocess()
    for frame_index in range(20):
        frame = sensor.snapshot(chn=CAM_CHN_ID_2)
        start = time.ticks_us()
        result = detector.run(frame.to_numpy_ref())
        elapsed_ms = time.ticks_diff(time.ticks_us(), start) / 1000.0
        detections = []
        if result:
            for index in range(len(result[0])):
                class_id = result[1][index]
                detections.append((LABELS[class_id], result[2][index], result[0][index]))
        print("YOLO", frame_index, "MS", elapsed_ms, "DET", detections)
        gc.collect()
finally:
    try:
        if detector:
            detector.deinit()
    except Exception:
        pass
    try:
        if sensor:
            sensor.stop()
    except Exception:
        pass
    try:
        MediaManager.deinit()
    except Exception:
        pass
    print("YOLO_PROBE_END")
