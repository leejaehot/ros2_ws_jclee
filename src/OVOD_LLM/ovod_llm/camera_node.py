import rclpy  # ROS 2의 Python 클라이언트 라이브러리를 가져옴
from rclpy.node import Node  # ROS 2에서 노드를 생성하는 기본 클래스
from sensor_msgs.msg import Image  # ROS 2의 센서 메시지 중 이미지 메시지 타입을 사용
from cv_bridge import CvBridge  # OpenCV 이미지와 ROS 이미지 메시지를 변환하는 라이브러리
import cv2  # OpenCV 라이브러리, 웹캠에서 영상을 받아오기 위해 사용

class WebcamPublisher(Node):  # ROS 2 노드를 상속하여 웹캠 퍼블리셔 클래스 생성
    def __init__(self):
        super().__init__('webcam_publisher')  # 노드의 이름을 'webcam_publisher'로 설정
        self.publisher_ = self.create_publisher(Image, 'webcam/image_raw', 10)  # 'webcam/image_raw' 토픽을 생성하고 버퍼 크기를 10으로 설정 ### blank space ###
        self.timer = self.create_timer(1/30, self.publish_frame)  # 30fps (1/30초마다 실행)로 타이머 설정 ### blank space ###
        self.bridge = CvBridge()  # OpenCV와 ROS 이미지 메시지를 변환하기 위한 CvBridge 객체 생성
        self.cap = cv2.VideoCapture(0)  # 웹캠(디폴트: /dev/video0)에서 영상을 받아오기 위한 VideoCapture 객체 생성

        if not self.cap.isOpened():  # 웹캠이 정상적으로 열리지 않았다면
            self.get_logger().error('Webcam is not opened... error...!!!')  # 에러 메시지 출력
            exit()  # 프로그램 종료
        else:
            self.get_logger().info('Webcam is connected! Ready to start.')  # 정상적으로 웹캠이 연결되었음을 로그로 출력

    def publish_frame(self):
        ret, frame = self.cap.read()  # 웹캠에서 한 프레임을 읽어옴
        if ret:  # 정상적으로 프레임을 받아왔을 경우
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')  # OpenCV의 BGR 이미지를 ROS 메시지로 변환
            self.publisher_.publish(msg)  # 변환된 이미지 메시지를 'webcam/image_raw' 토픽으로 퍼블리시
        else:  # 프레임을 가져오지 못한 경우
            self.get_logger().warn('웹캠에서 프레임 가져오는거 실패')  # 경고 메시지 출력

    def destroy_node(self):
        super().destroy_node()  # 기본 노드 종료 동작 실행
        self.cap.release()  # 웹캠 자원 해제

def main(args=None):
    rclpy.init(args=args)  # ROS 2 노드 초기화
    node = WebcamPublisher()  # WebcamPublisher 노드 객체 생성
    try:
        rclpy.spin(node)  # 노드를 계속 실행 (ROS 2 이벤트 루프 유지)
    except KeyboardInterrupt:  # Ctrl+C가 입력되면
        pass  # 아무 작업도 하지 않고 넘어감
    finally:
        node.destroy_node()  # 노드 종료 및 자원 해제
        rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':  # 스크립트가 직접 실행될 때만 main 함수 실행
    main()
