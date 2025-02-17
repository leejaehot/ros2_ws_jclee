import rclpy  # ROS 2의 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS 2에서 노드를 생성하는 기본 클래스
from sensor_msgs.msg import Image  # ROS 2에서 이미지 메시지를 처리하는 메시지 타입
from std_msgs.msg import String  # ROS 2에서 문자열 메시지를 처리하는 메시지 타입
from cv_bridge import CvBridge  # OpenCV 이미지와 ROS 2 이미지 메시지를 변환하는 라이브러리
import cv2  # OpenCV 라이브러리 (이미지 처리 및 웹캠 스트리밍)
import torch  # PyTorch 라이브러리 (YOLO 및 CLIP 모델 실행)
from ultralytics import YOLOWorld  # YOLOWorld 모델을 불러오기 위한 라이브러리
import os  # 파일 및 디렉토리 조작을 위한 라이브러리
import random  # 랜덤 색상을 바운딩 박스에 적용하기 위해 사용
import time  # 시간 측정 및 대기 기능 사용
import clip  # CLIP 모델을 사용하여 객체 필터링

class DetectionNode(Node):  # ROS 2 노드를 상속하여 객체 감지 노드 생성
    def __init__(self):
        super().__init__('detection_node')  # 노드 이름을 'detection_node'로 설정

        ###======================= 토픽 관련 세팅 =======================
        qos_profile = rclpy.qos.QoSProfile(depth=10)  # QoS 설정 (버퍼 크기: 10)

        # 웹캠에서 이미지 수신
        self.image_subscription = self.create_subscription(
            Image,  # ROS 2에서 이미지 데이터를 받을 메시지 타입
            'webcam/image_raw',  # 웹캠에서 발행하는 이미지 토픽을 구독
            self.detect_callback,  # 이미지가 수신될 때 실행할 콜백 함수
            qos_profile
        )
        
        # 감지된 이미지를 퍼블리시
        self.captured_image_publisher = self.create_publisher(
            Image,  # ROS 2에서 퍼블리시할 이미지 메시지 타입
            'captured/image_raw',  # 'captured/image_raw' 토픽으로 퍼블리시
            qos_profile
        )

        # 사용자 텍스트 입력을 수신하는 토픽
        self.text_subscriber = self.create_subscription(
            String,  # ROS 2에서 문자열 데이터를 받을 메시지 타입
            '/user_text_input',  # 사용자 입력을 받을 토픽
            self.update_class_callback,  # 클래스 업데이트를 수행하는 콜백 함수
            qos_profile
        )
        
        self.bridge = CvBridge()  # OpenCV와 ROS 2 이미지 메시지 변환을 위한 CvBridge 객체 생성
        self.get_logger().info("YOLOWorld 노드 시작")  # 노드 시작 로그 출력

        ###======================= YOLOWorld 설정 부분 =======================
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # GPU 사용 가능 여부 확인
        self.get_logger().info(f"Using device: {self.device}")  # 사용 중인 장치 출력

        # YOLOWorld 모델 불러오기
        s_t = time.time()  # 모델 로딩 시작 시간 기록
        self.model = YOLOWorld("yolov8l-worldv2.pt")  # YOLOWorld 모델 가중치 불러오기
        self.model.to(self.device)  # 모델을 선택한 장치(GPU 또는 CPU)로 이동
        load_t = time.time() - s_t  # 모델 로딩 시간 측정
        self.get_logger().info(f"YOLOWorld model loaded successfully in {load_t:.4f} seconds")  # 모델 로딩 완료 로그 출력

        ###======================= CLIP 모델 설정 =======================
        s_t = time.time()  # 모델 로딩 시작 시간 기록
        self.clip_model, self.clip_preprocess = clip.load("ViT-B/32", device=self.device)  # CLIP 모델 로드
        load_t = time.time() - s_t  # 모델 로딩 시간 측정
        self.get_logger().info(f"CLIP model loaded successfully in {load_t:.4f} seconds")  # CLIP 모델 로딩 완료 로그 출력

        ###======================= 기타 설정 =======================
        # 프레임 저장 디렉토리 설정
        self.save_directory = "/home/rcv/ros2_ws_jclee/src/OVOD_LLM/ovod_llm/captured_frames"
        os.makedirs(self.save_directory, exist_ok=True)  # 저장 디렉토리가 없으면 생성
        self.frame_count = 0  # 저장된 프레임 개수 카운트
        self.label_colors = {}  # 클래스별 색상 저장을 위한 딕셔너리

        # OpenCV 창 설정 (선택 사항)
        cv2.namedWindow("YOLOWorld + CLIP Object Detection", cv2.WINDOW_AUTOSIZE)

    def update_class_callback(self, msg):
        """ 사용자 입력 토픽(/user_text_input)으로 감지할 클래스를 변경하는 함수 """
        new_class = msg.data.strip().lower()  # 사용자 입력 데이터를 소문자로 변환하여 저장
        if not new_class:  # 입력이 비어 있으면 무시
            self.get_logger().warn("Received empty class input. Ignoring.")
            return
        
        self.current_class = new_class  # 현재 감지할 클래스 업데이트
        self.model.set_classes([self.current_class])  # YOLOWorld 모델에 새로운 클래스 적용
        self.get_logger().info(f"Updated YOLOWorld class to: {self.current_class}")  # 변경된 클래스 로그 출력

    def detect_callback(self, data):
        """ YOLOWorld를 사용하여 객체 감지 및 CLIP을 활용한 필터링 수행 """
        # ROS Image 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')  
        results = self.model.predict(cv_image)  # YOLOWorld 모델로 객체 감지 수행

        detected_objects = []  # 감지된 객체 이름 리스트
        object_boxes = []  # 감지된 객체의 바운딩 박스 정보

        # YOLOWorld 결과에서 바운딩 박스, 클래스, 신뢰도 추출
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
                conf = box.conf[0]  # 신뢰도
                cls = box.cls[0]  # 클래스 ID
                class_name = self.model.names[int(cls)]  # 클래스 이름

                detected_objects.append(class_name)
                object_boxes.append((x1, y1, x2, y2, conf, class_name))

        if detected_objects:
            selected_objects = self.filter_objects_with_clip(detected_objects, object_boxes)  # CLIP 필터링 수행
        else:
            selected_objects = []

        # 필터링된 객체만 화면에 표시
        for (x1, y1, x2, y2, conf, class_name) in selected_objects:
            label = f"{class_name} {conf:.2f}"
            cv2.rectangle(cv_image, (x1 ,y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 감지 결과를 화면에 표시
        cv2.imshow('YOLOWorld + CLIP Object Detection', cv_image)

        # 'c' 키를 누르면 프레임을 저장하고 퍼블리시
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            file_path = os.path.join(self.save_directory, "captured_frame.jpg")
            cv2.imwrite(file_path, cv_image)
            self.get_logger().info(f"Captured frame saved at: {file_path}")

            captured_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.captured_image_publisher.publish(captured_msg)
            self.get_logger().info("Captured frame published.")

def main(args=None):
    """ ROS 2 노드 실행 함수 """
    rclpy.init(args=args)  
    node = DetectionNode()  
    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:  
        pass  
    finally:
        node.destroy_node()  
        rclpy.shutdown()  

if __name__ == '__main__':  
    main()
