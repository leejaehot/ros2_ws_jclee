import rclpy  # ROS 2의 Python 클라이언트 라이브러리 가져오기
from rclpy.node import Node  # ROS 2 노드를 생성하는 기본 클래스
from sensor_msgs.msg import Image  # 센서 메시지 중 이미지 메시지 타입
from std_msgs.msg import String  # 문자열 메시지 타입 (사용자 입력을 받을 때 사용)
from cv_bridge import CvBridge  # OpenCV 이미지와 ROS 이미지 메시지를 변환하는 라이브러리
import cv2  # OpenCV 라이브러리 (이미지 처리 및 웹캠 스트리밍)
import torch  # PyTorch 라이브러리 (YOLO 모델 실행)
from ultralytics import YOLOWorld  # YOLOWorld 모델을 불러오기 위한 라이브러리
import os  # 파일 및 디렉토리 조작을 위한 라이브러리
import random  # 랜덤 색상을 바운딩 박스에 적용하기 위해 사용
import time  # 시간 관련 기능 사용

class DetectionNode(Node):  # ROS 2 노드를 상속하여 객체 감지 노드 생성
    def __init__(self):
        super().__init__('detection_node')  # 노드의 이름을 'detection_node'로 설정

        ###======================= 토픽 관련 세팅 =======================
        qos_profile = rclpy.qos.QoSProfile(depth=10)  # QoS 설정 (버퍼 크기: 10)
        self.image_subscription = self.create_subscription(
            Image,  # ROS 2에서 이미지 데이터를 받을 메시지 타입
            'webcam/image_raw',  # 웹캠에서 발행하는 이미지 토픽을 구독
            self.detect_callback,  # 콜백 함수로 detect_callback 사용
            qos_profile
        )
        
        self.captured_image_publisher = self.create_publisher(
            Image,  # 캡처된 프레임을 퍼블리싱할 이미지 메시지 타입
            'captured/image_raw',  # 'captured/image_raw' 토픽으로 퍼블리시
            qos_profile
        )

        self.text_subscriber = self.create_subscription(
            String,  # 문자열 메시지 타입 (사용자 입력)
            '/user_text_input',  # 사용자 입력을 받을 토픽
            self.update_class_callback,  # 콜백 함수로 update_class_callback 사용
            qos_profile
        )
        
        self.bridge = CvBridge()  # OpenCV와 ROS 2 이미지 메시지 변환을 위한 CvBridge 객체 생성
        self.get_logger().info("yoloworld 노드 시작")  # 노드가 시작되었음을 로그로 출력

        ###======================= YOLOWorld 설정 부분 =======================
        # YOLOWorld 모델 로드
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # GPU 사용 가능 여부 확인 후 설정
        self.get_logger().info(f"Using device: {self.device}")  # 사용 중인 장치 출력

        # YOLOWorld 모델 불러오기
        # self.model = YOLOWorld("yolov8s-world.pt")  # YOLOWorld 모델 가중치 (주석 처리됨)
        self.model = YOLOWorld("yolov8x-worldv2.pt")  # YOLOWorld 모델 가중치 (사용 중)
        self.model.to(self.device)  # 모델을 선택한 장치(GPU 또는 CPU)로 이동

        self.get_logger().info("YOLOWorld model loaded successfully")  # 모델이 성공적으로 로드되었음을 출력

        ###=============================================================

        # 프레임 저장 디렉토리 설정
        self.save_directory = "/home/rcv/ros2_ws_jclee/src/OVOD_LLM/ovod_llm/captured_frames"
        os.makedirs(self.save_directory, exist_ok=True)  # 저장 디렉토리가 없으면 생성
        self.frame_count = 0  # 프레임 저장을 위한 카운터 초기화
        
        # 클래스별 색상 저장을 위한 딕셔너리
        self.label_colors = {}

        # 결과를 시각화할 OpenCV 창 설정 (선택 사항)
        cv2.namedWindow("YOLOWorld Object Detection", cv2.WINDOW_AUTOSIZE)

    def update_class_callback(self, msg):
        """ 사용자가 /user_text_input 토픽을 통해 감지할 클래스를 변경하면 업데이트 """
        new_class = msg.data.strip()  # 문자열 메시지에서 클래스 이름 가져오기
        if not new_class:  # 입력이 비어 있으면 무시
            self.get_logger().warn("Received empty class input. Ignoring.")
            return
        
        self.current_class = new_class  # 현재 감지할 클래스 업데이트
        self.model.set_classes([self.current_class])  # YOLOWorld 클래스 설정 업데이트
        self.get_logger().info(f"Updated YOLOWorld class to: {self.current_class}")  # 변경된 클래스 로그 출력

    def detect_callback(self, data):
        """ 웹캠에서 받은 이미지를 처리하여 객체 감지 수행 """
        ###======================= 모델 추론 부분 =======================
        # ROS Image 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')  # YOLO는 BGR 형식을 처리 가능
        results = self.model.predict(cv_image, max_det=100, iou=0.01, conf=0.003)  # YOLOWorld 모델 추론 수행
        
        boxes = []
        confidences = []
        class_ids = []


        ###======================= 결과 시각화 코드 =======================
        # YOLOWorld 결과에서 바운딩 박스, 클래스, 신뢰도 추출
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표 추출
                conf = box.conf[0].item()  # 신뢰도 추출
                class_id = int(box.cls[0].item()) # 클래스 ID 추출

                boxes.append([x1, y1, x2-x1, y2-y1])
                confidences.append(conf)
                class_ids.append(class_id)

                class_name = self.model.names[class_id]  # 클래스 이름 변환
                label = f"{class_name} {conf:.2f}"  # 클래스 이름과 신뢰도 표시 형식

                # 클래스별 랜덤 색상 할당
                if class_name not in self.label_colors:
                    self.label_colors[class_name] = [random.randint(0,255) for _ in range(3)]
                label_color = self.label_colors[class_name]  # 클래스별 색상 가져오기


        # Apply Non-Maximum Suppression (NMS)
        nms_indices = cv2.dnn.NMSBoxes(boxes, confidences, score_threshold=0.3, nms_threshold=0.5)
        
        if len(nms_indices) > 0:
            nms_indices = nms_indices.flatten()
            for i in nms_indices:
                x, y, w, h = boxes[i]
                class_id = class_ids[i]
                conf = confidences[i]
                class_name = self.model.names[class_id]
                label_color = self.label_colors[class_name]
                
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), label_color, 2)
                label = f"{class_name} {conf:.2f}"
                cv2.putText(cv_image, label, (x + 1, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_color, 2)


        # OpenCV 창에 탐지 결과 표시
        cv2.imshow('YOLOWorld Object Detection', cv_image)

        ###======================= 캡처 및 저장 및 퍼블리시 =======================
        key = cv2.waitKey(1) & 0xFF  # 키 입력 감지
        if key == ord('c'):  # 'c' 키를 누르면 캡처
            file_path = os.path.join(self.save_directory, f"captured_frame.jpg")  # 저장할 파일 경로 설정
            cv2.imwrite(file_path, cv_image)  # 이미지 저장
            self.get_logger().info(f"-=-=-=-=-=-{file_path} 여기에 저장됨-=-=-=-=-=-=")  # 저장 완료 로그 출력

            # 캡처된 프레임을 ROS 2 메시지로 변환하여 퍼블리시
            captured_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.captured_image_publisher.publish(captured_msg)  # 'captured/image_raw' 토픽으로 전송
            self.get_logger().info("-=-=-=-=-=Published captured frame-=-=-=-=-=")  # 퍼블리시 완료 로그 출력

def main(args=None):
    """ ROS 2 노드 실행 함수 """
    rclpy.init(args=args)  # ROS 2 노드 초기화
    node = DetectionNode()  # DetectionNode 객체 생성
    try:
        rclpy.spin(node)  # 노드를 실행하여 이벤트 루프 유지
    except KeyboardInterrupt:  # Ctrl+C 입력 시 종료 처리
        pass  # 아무 작업도 하지 않고 종료
    finally:
        node.destroy_node()  # 노드 종료 및 자원 해제
        rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':  # 스크립트가 직접 실행될 때만 main 함수 실행
    main()
