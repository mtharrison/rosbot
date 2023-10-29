import rclpy
from rclpy.node import Node
import cv2 
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge



class CapturePublisher(Node):

    def __init__(self):
        super().__init__('capture_publisher')
        self.publisher_small = self.create_publisher(CompressedImage, 'webcam_capture_small', 1)
        self.publisher_hd = self.create_publisher(CompressedImage, 'webcam_capture_hd', 1)
        
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
         
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480/2)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320/2)
        self.cap = cap

        self.br = CvBridge()
        
    def timer_callback(self):
        _, img = self.cap.read()
        
        small_msg = self.br.cv2_to_compressed_imgmsg(img, [int(cv2.IMWRITE_JPEG_QUALITY), 10])
        hd_msg = self.br.cv2_to_compressed_imgmsg(img, [int(cv2.IMWRITE_JPEG_QUALITY), 20])
        
        small_msg.header.frame_id = 'camera_link'
        hd_msg.header.frame_id = 'camera_link'
        
        small_msg.header.stamp = self.get_clock().now().to_msg()
        hd_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher_small.publish(small_msg)
        self.publisher_hd.publish(hd_msg)
        


def main(args=None):
    rclpy.init(args=args)
    capture_publisher = CapturePublisher()
    rclpy.spin(capture_publisher)
    capture_publisher.cap.release()
    capture_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
