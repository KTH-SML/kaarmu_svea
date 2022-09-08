#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo

import cv2
import numpy as np

# YOLOv4 implementation (tianxiaomo)
from tool.utils import *
from tool.torch_utils import *
from tool.darknet2pytorch import Darknet

# SORT Object Tracking
from sort import Sort

from rosonic import Node, Parameter, Name
from rsu_msgs.msg import Object, StampedObjectArray


class object_detect(Node):

    ## Options

    USE_CUDA = Parameter('~use_cuda', False)
    ENABLE_BBOX_IMAGE = Parameter('~enable_bbox_image', False)

    ## Topics

    SUB_IMAGE = Parameter('~sub_image', 'image')
    PUB_OBJECTS = Parameter('~pub_objects', 'objects')
    PUB_BBOX_IMAGE = Parameter('~pub_bbox_image', 'bbox_image')

    SUB_CAMERA_INFO = (
        SUB_IMAGE.copy()
        .then(Name('camera_info').next_to)
        .then(str)
    )

    PUB_CAMERA_INFO = (
        PUB_BBOX_IMAGE.copy()
        .then(Name('camera_info').next_to)
        .then(str)
    )

    ## Auxiliary

    MAX_AGE = Parameter('~max_age', 30)
    CFG_PATH = Parameter('~cfg_path')
    WEIGHTS_PATH = Parameter('~weights_path')
    LABELS_PATH = Parameter('~labels_path')


    def __init__(self):

        ## Neural Network

        rospy.loginfo('Getting labels from %s', self.LABELS_PATH)
        self.labels = load_class_names(self.LABELS_PATH)

        rospy.loginfo('Loading network from %s', self.CFG_PATH)
        self.model = Darknet(self.CFG_PATH)

        rospy.loginfo('Loading network weights from %s', self.WEIGHTS_PATH)
        self.model.load_weights(self.WEIGHTS_PATH)

        if self.USE_CUDA:
            rospy.loginfo('CUDA enabled')
            self.model.cuda()
        else:
            rospy.loginfo('CUDA disabled')

        ## SORT multi-object tracker

        self.tracked_objects = []
        self.sort = Sort(max_age=self.MAX_AGE, min_hits=3, iou_threshold=0.3)

        ## Publishers

        self.pub_objects = rospy.Publisher(self.PUB_OBJECTS, StampedObjectArray, queue_size=10)
        self.log_event(self.PUB_OBJECTS)

        if self.ENABLE_BBOX_IMAGE:

            self.pub_bbox_image = rospy.Publisher(self.PUB_BBOX_IMAGE, Image, queue_size=1)
            self.log_event(self.PUB_BBOX_IMAGE)

        ## Subscribers

        rospy.Subscriber(self.SUB_IMAGE, Image, self.callback)
        self.log_event(self.SUB_IMAGE)

        ## Relay (sub->pub) camera info

        if self.ENABLE_BBOX_IMAGE:
            pub = rospy.Publisher(self.PUB_CAMERA_INFO, CameraInfo, queue_size=1)
            rospy.Subscriber(self.SUB_CAMERA_INFO, CameraInfo, pub.publish)


    def callback(self, image):

        ## Detect objects

        frame = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)

        resized_frame = cv2.resize(frame, (self.model.width, self.model.height))
        boxes = do_detect(self.model, resized_frame, 0.4, 0.6, self.USE_CUDA, show_stats=False)[0]

        ## Update tracker

        self.tracked_objects = self.sort.update(
            np.array(boxes)[:, :4] if boxes else
            np.empty((0, 5))
        )

        ## Create object messages

        objects = []
        for box in boxes:

            ## Get box

            # box have normalized u, v
            u1, v1, u2, v2, _, conf, label_id = box

            # get real pixel coords
            u1, u2 = [min(image.width, max(0, round(u*image.width))) for u in (u1, u2)]
            v1, v2 = [min(image.height, max(0, round(v*image.height))) for v in (v1, v2)]

            # do not continue if box has no size
            if u1 != u2 and v1 != v2:

                # pick best matched tracked object
                trk = [0, 0, 0, 0, 0]
                trk_iou = 0
                trk_ind = -1
                for i, _trk in enumerate(self.tracked_objects):
                    _iou = iou(_trk[:4], box[:4])
                    if trk_iou < _iou:
                        trk = _trk
                        trk_iou = _iou
                        trk_ind = i

                tracker = self.sort.trackers[trk_ind]

                obj = Object()
                obj.id = int(trk[-1])
                obj.label = self.labels[label_id]
                obj.detection_conf = conf
                obj.tracking_conf = tracker.kf.likelihood
                obj.roi.x_offset = u1
                obj.roi.y_offset = v1
                obj.roi.width = u2 - u1
                obj.roi.height = v2 - v1
                objects.append(obj)

            # if enabled, modify frame (add bounding boxes)
            if self.ENABLE_BBOX_IMAGE:
                frame = self.plot_label(frame, box)

        # Publish objects
        if objects:
            object_array = StampedObjectArray()
            object_array.header = image.header
            object_array.objects = objects
            self.pub_objects.publish(object_array)

        if self.ENABLE_BBOX_IMAGE:
            new_image = Image()
            new_image.header = image.header
            new_image.height = frame.shape[0]
            new_image.width = frame.shape[1]
            new_image.encoding = 'rgb8'
            new_image.step = frame.size // new_image.height
            new_image.data = frame.tostring()

            self.pub_bbox_image.publish(new_image)

    def plot_label(self, image, box, append=''):
        image = np.copy(image)

        height, width, _ = image.shape

        x1, y1, x2, y2, _, cls_conf, cls_id, *_ = box

        x1 = int(x1 * width)
        y1 = int(y1 * height)
        x2 = int(x2 * width)
        y2 = int(y2 * height)

        label = self.labels[cls_id] if cls_id < len(self.labels) else '?'

        bbox_thick = int(0.6 * (height + width) / 600)
        color = (255, 0, 0) # tuple(map(int, np.random.rand(3) * 255))
        msg = f'{label} ({cls_conf:.0%}) {append}'
        msg_w, msg_h = cv2.getTextSize(msg, 0, 0.7, thickness=bbox_thick // 2)[0]

        cv2.rectangle(image, (x1, y1), (x1 + msg_w, y1 - msg_h - 3), color, -1)

        image = cv2.putText(image, msg, (x1, y1-2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), bbox_thick//2, lineType=cv2.LINE_AA)
        image = cv2.rectangle(image, (x1, y1), (x2, y2), color, bbox_thick)

        return image


def iou(box1, box2):

    u11, v11, u21, v21 = box1
    u12, v12, u22, v22 = box2

    u1_max = max(u11, u12)
    v1_max = max(v11, v12)
    u2_min = min(u21, u22)
    v2_min = min(v21, v22)

    # area of intersection
    w = abs(u1_max - u2_min)
    h = abs(v1_max - v2_min)
    area_i = w * h

    # area of box1
    w = abs(u21 - u11)
    h = abs(v21 - v11)
    area_1 = w * h

    # area of box2
    w = abs(u22 - u12)
    h = abs(v22 - v12)
    area_2 = w * h

    # area of union
    area_u = area_1 + area_2 - area_i

    # intersection-over-union
    return area_i / area_u

if __name__ == '__main__':

    ##  Start node  ##

    object_detect.run()

