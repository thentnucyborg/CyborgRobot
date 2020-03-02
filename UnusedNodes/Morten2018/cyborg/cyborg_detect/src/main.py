#!/usr/bin/env python

import os
#import pyyolo
import rospy
import roslib
import rospkg
import sys
import time

import cv2 as cv
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from cyborg_detect.cfg import DetectorConfig
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import Point32
from cyborg_detect.msg import Prediction, Predictions
from sensor_msgs.msg import Image
from std_msgs.msg import Header

NAME = 'cyborg_detect'


class Pred(object):
    def __init__(self, label, confidence, box, distance=0):
        self.label = label
        self.confidence = confidence
        self.box = box
        self.distance = distance

    def draw(self, frame):
        """ Draw the prediction on the provided frame """
        left, right, top, bottom = self.box['left'], self.box['right'], self.box['top'], self.box['bottom']
        text = '{}: {:.2f} ({:.2f} m)'.format(self.label, self.confidence, self.distance)

        # Draw label
        text_size, baseline = cv.getTextSize(text, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, text_size[1])

        cv.rectangle(frame, (left, top - text_size[1]), (left + text_size[0], top + baseline), (255, 255, 255), cv.FILLED)
        cv.putText(frame, text, (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

        # Draw bounding box
        cv.rectangle(frame, (left, top), (right, bottom), (0, 255, 0))

    def to_msg(self):
        """ Convert the Prediction to an ROS message """
        msg = Prediction()
        msg.label = self.label
        msg.confidence = self.confidence
        msg.bounding_box.points = [
                Point32(x=self.box['left'], y=self.box['top']),
                Point32(x=self.box['right'], y=self.box['top']),
                Point32(x=self.box['right'], y=self.box['bottom']),
                Point32(x=self.box['left'], y=self.box['bottom'])]
        msg.distance = self.distance

        return msg


class PredictionContainer(object):
    def __init__(self, data, image_header, depth=None):
        self.image_header = image_header
        self.predictions = []

        for res in data:
            label = res['label']
            confidence = res['prob']
            box = {'left': res['left'], 'right': res['right'], 'top': res['top'], 'bottom': res['bottom']}
            distance = self.__calculate_distance(box, depth) if depth is not None else 0
        
            self.predictions.append(Pred(label, confidence, box, distance))

    @staticmethod
    def __calculate_distance(box, depth):
        l = box['left']
        r = box['right']
        t = box['top']
        b = box['bottom']

        distance = np.nanmedian(depth[l:r, t:b].astype(np.float32))

        if np.isnan(distance):
            center = (r - (r - l) // 2, b - (b - t) // 2)
            distance = depth[center[1], center[0]] 

        return distance

    def draw(self, frame):
        """ Draw all predictions on the provided frame """
        for prediction in self.predictions:
            prediction.draw(frame)

    def to_msg(self):
        """ Convert all predictions to an ROS message """
        msg = Predictions()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.image_header = self.image_header
        msg.predictions = [prediction.to_msg() for prediction in self.predictions]

        return msg


class Detector(object):
    def __init__(self):
        rospy.init_node(NAME, anonymous=True)

        # Image detection parameters
        self.conf_thresh = 0.6
        self.hier_thresh = 0.8
        self.frame_height = 416
        self.frame_width = 416

        # Set up dynamic reconfigure
        self.srv = Server(DetectorConfig, self.reconf_cb)

        # Set up OpenCV Bridge
        self.bridge = CvBridge()

        # Load network
        rp = rospkg.RosPack()

        # Load object labels
        classfile = os.path.join(rp.get_path(NAME), rospy.get_param('~label_file_path'))
        self.classes = None
        with open(classfile, 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')

        # Load network
        cfgfile = os.path.join(rp.get_path(NAME), rospy.get_param('~cfg_file_path'))
        weightsfile = os.path.join(rp.get_path(NAME), rospy.get_param('~weights_file_path'))
        self.net = cv.dnn.readNet(cfgfile, weightsfile)
        self.net.setPreferableTarget(cv.dnn.DNN_TARGET_OPENCL)

        # Set up detection image publisher
        det_topic = rospy.get_param('~detection_image_topic_name')
        self.det_pub = rospy.Publisher(det_topic, Image, queue_size=1, latch=True)

        # Set up prediction publisher
        pred_topic = rospy.get_param('~predictions_topic_name')
        self.pred_pub = rospy.Publisher(pred_topic, Predictions, queue_size=1, latch=True)

        # The first subscriber retrieves image data from the left image and passes it to the callback function
        image_topic = rospy.get_param('~camera_topic_name') 
        image_sub = Subscriber(image_topic, Image, queue_size=1)

        # The second subscriber retrieves depth data from the camera as 32 bit floats with values in meters and maps this onto an image structure, which is passed to callback2
        if rospy.has_param('~depth_topic_name'):
            depth_topic = rospy.get_param('~depth_topic_name') 
            depth_sub = Subscriber(depth_topic, Image, queue_size=1)
        else:
            depth_sub = None

        # We want to receive both images in the same callback
        if depth_sub:
            rospy.loginfo("with depth sub")
            ts = ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=1, slop=0.1)
        else:
            rospy.loginfo("without depth sub")
            ts = ApproximateTimeSynchronizer([image_sub], queue_size=1, slop=0.1)

        ts.registerCallback(self.image_cb)

    def __postprocess(self, frame, outs):
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]

        layerNames = self.net.getLayerNames()
        lastLayerId = self.net.getLayerId(layerNames[-1])
        lastLayer = self.net.getLayer(lastLayerId)

        data = []

        if self.net.getLayer(0).outputNameToIndex('im_info') != -1:  # Faster-RCNN or R-FCN
            # Network produces output blob with a shape 1x1xNx7 where N is a number of
            # detections and an every detection is a vector of values
            # [batchId, classId, confidence, left, top, right, bottom]
            assert(len(outs) == 1)
            out = outs[0]
            for detection in out[0, 0]:
                confidence = detection[2]
                if confidence > self.conf_thresh:
                    class_id = int(detection[1]) - 1 # Skip background label
                    label = self.classes[class_id] if self.classes else str(class_id)

                    data.append({
                        'left': int(detection[3]),
                        'top': int(detection[4]),
                        'right': int(detection[5]),
                        'bottom': int(detection[6]),
                        'label': label,
                        'prob': confidence
                    })
        elif lastLayer.type == 'DetectionOutput':
            # Network produces output blob with a shape 1x1xNx7 where N is a number of
            # detections and an every detection is a vector of values
            # [batchId, classId, confidence, left, top, right, bottom]
            assert(len(outs) == 1)
            out = outs[0]
            for detection in out[0, 0]:
                confidence = detection[2]
                if confidence > self.conf_thresh:
                    class_id = int(detection[1]) - 1  # Skip background label
                    label = self.classes[class_id] if self.classes else str(class_id)

                    data.append({
                        'left': int(detection[3] * frameWidth),
                        'top': int(detection[4] * frameHeight),
                        'right': int(detection[5] * frameWidth),
                        'bottom': int(detection[6] * frameHeight),
                        'label': label,
                        'prob': confidence
                    })
        elif lastLayer.type == 'Region':
            # Network produces output blob with a shape NxC where N is a number of
            # detected objects and C is a number of classes + 4 where the first 4
            # numbers are [center_x, center_y, width, height]
            classIds = []
            confidences = []
            boxes = []
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > self.conf_thresh:
                        label = self.classes[class_id] if self.classes else str(class_id)

                        center_x = int(detection[0] * frameWidth)
                        center_y = int(detection[1] * frameHeight)
                        width = int(detection[2] * frameWidth)
                        height = int(detection[3] * frameHeight)

                        data.append({
                            'left': center_x - width / 2,
                            'top': center_y - height / 2,
                            'right': center_x + width / 2,
                            'bottom': center_y + height / 2,
                            'label': label,
                            'prob': confidence
                        })
        return data

    def get_output_names(self, net):
        layersNames = net.getLayerNames()
        return [layersNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    def get_predictions(self, net, frame, frame_size=None):
        frame_size = frame_size if frame_size else (frame.shape[0], frame.shape[1])
        blob = cv.dnn.blobFromImage(frame, 1.0/255.0, frame_size, [0, 0, 0])
        net.setInput(blob)

        output = net.forward(self.get_output_names(net))

        return self.__postprocess(frame, output)

    def image_cb(self, image_msg, depth_msg=None):
        det_conns = self.det_pub.get_num_connections()
        pred_conns = self.pred_pub.get_num_connections()

        begin = rospy.get_rostime()

        # Early return if no one is listening
        #if not det_conns and not pred_conns:
        #    return

        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

        if depth_msg:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        else:
            depth = None

        output = self.get_predictions(self.net, frame, (self.frame_height, self.frame_width))

        now = rospy.get_rostime()
        diff = now - begin
        rospy.loginfo(diff)

        predictions = PredictionContainer(output, image_msg.header, depth)

        #if det_conns:
        predictions.draw(frame)
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        self.det_pub.publish(image_msg)

        #if pred_conns:
        pred_msg = predictions.to_msg()
        self.pred_pub.publish(pred_msg)

    def reconf_cb(self, config, level):
        rospy.loginfo('Reconfigure request')
        self.conf_thresh = config['conf_threshold']

        return config


def main(args):
    ir = Detector()	

    try:
        rospy.spin()
    except KeyboardInterrupt: 
        print "Shutting down ROS Image feature detector module"
    

if __name__ == "__main__":
    main(sys.argv)
