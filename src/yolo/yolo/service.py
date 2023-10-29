import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge
from yolov7_package import Yolov7Detector

from sensor_msgs.msg import CompressedImage # Image is the message type
from foxglove_msgs.msg import ImageAnnotations, PointsAnnotation, Point2, Color, TextAnnotation


class YoloSubscriber(Node):

    def __init__(self):
        super().__init__('yolo_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/photo/webcam_capture',
            self.listener_callback,
            10)
        
        self.counter = 0
        
        # self.publisher_classes = self.create_publisher(UInt16MultiArray, 'yolo_classes', 10)
        self.publisher_boxes = self.create_publisher(ImageAnnotations, 'yolo_boxes', 10)
        # self.publisher_scores = self.create_publisher(Float32MultiArray, 'yolo_scores', 10)
        
        self.br = CvBridge()
        self.det = Yolov7Detector(traced=False)
        
    def listener_callback(self, msg):        
        img = self.br.compressed_imgmsg_to_cv2(msg)
        classes, boxes, scores = self.det.detect(img)
        
        if len(classes[0]) == 0:
            self.publisher_boxes.publish(ImageAnnotations())
            return

        
        boxes_msg = ImageAnnotations()
        boxes_msg.points = []
        boxes_msg.texts = []
        
        for i, box in enumerate(boxes[0]):
            
            # Add Box
            
            x, y, x2, y2 = box
        
            points = PointsAnnotation()
            points.type = 2
            points.points = [
                Point2(x=x, y=y),
                Point2(x=x2, y=y),
                Point2(x=x2, y=y2),
                Point2(x=x, y=y2),
            ]
            
            points.outline_color = Color(r=1.0, g=0.0, b=0.0, a=1.0)
            points.fill_color = Color(r=1.0, g=0.0, b=0.0, a=0.0)
            points.thickness = 2.0
            boxes_msg.points.append(points)
            
            # Add Class
            
            score = scores[0][i] * 100
            
            text = TextAnnotation()
            text.position = Point2(x=x, y=y)
            text.text = f'{self.det.names[classes[0][i]]} {score:.2f}%' 
            text.font_size = 24.0
            text.text_color = Color(r=1.0, g=1.0, b=1.0, a=1.0)
            text.background_color = Color(r=1.0, g=0.0, b=0.0, a=1.0)
            boxes_msg.texts.append(text)
        
        self.publisher_boxes.publish(boxes_msg)

def main(args=None):
    rclpy.init(args=args)

    yolo_subscriber = YoloSubscriber()

    rclpy.spin(yolo_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolo_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
