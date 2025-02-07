import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher') 
        self.publisher_ = self.create_publisher(Image, 'webcam/image_raw', 10)  # 웹캠 토픽 설정 ### blank space ###
        self.timer = self.create_timer(1/30, self.publish_frame)  # 10Hz(0.1) ->30fps(1/30) ### blank space ###
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Joytron 웹캠을 /dev/video0으로 연결

        if not self.cap.isOpened():
            self.get_logger().error('Webcam is not opened... error...!!!')
            exit()
        else:
            self.get_logger().info('Webcam is connected! Ready to start.')


    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # OpenCV 이미지를 ROS 이미지 메시지로 변환
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('웹캠에서 프레임 가져오는거 실패')

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()  # 웹캠 해제


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
