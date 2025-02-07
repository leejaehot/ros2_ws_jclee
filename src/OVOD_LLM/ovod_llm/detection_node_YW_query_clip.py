import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLOWorld
import os
import random
import time
import clip

class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        ###======================= 토픽 관련 세팅 =======================
        qos_profile = rclpy.qos.QoSProfile(depth=10)
        self.image_subscription = self.create_subscription(
            Image,   # from sensor_msgs.msg import Image 메세지 타입 지정
            'webcam/image_raw',  # Joytron 웹캠에서 발행하는 이미지 토픽
            self.detect_callback,
            qos_profile
        )
        
        self.captured_image_publisher = self.create_publisher(
            Image,  # 캡처된 프레임 퍼블리싱
            'captured/image_raw',
            qos_profile
        )

        self.text_subscriber = self.create_subscription(
            String,
            '/user_text_input',
            self.update_class_callback,
            qos_profile
        )
        
        self.bridge = CvBridge()
        self.get_logger().info("yoloworld 노드 시작")

        ###======================= YOLOWorld 설정 부분 =======================
        # YOLOWorld 모델 로드
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # GPU or CPU
        self.get_logger().info(f"Using device: {self.device}")

        # YOLOWorld 모델 불러오기
        s_t = time.time()
        #self.model = YOLOWorld("yolov8s-world.pt")  # YOLOWorld 모델 가중치
        self.model = YOLOWorld("yolov8l-worldv2.pt")  # YOLOWorld 모델 가중치
        self.model.to(self.device)

        # YOLO 모델에 설정
        # self.model.set_classes(['water bottle'])
        load_t = time.time() - s_t
        self.get_logger().info(f"YOLOWorld model loaded successfully in {load_t:.4f} seconds")

        ###=============================================================


        ###====================== CLIP model ===================
        s_t = time.time()
        self.clip_model, self.clip_preprocess = clip.load("ViT-B/32", device=self.device)
        load_t = time.time() - s_t
        self.get_logger().info(f"CLIP model loaded successfully in {load_t:.4f} seconds")
        






        # 프레임 저장 디렉토리 설정
        self.save_directory = "/home/rcv/ros2_ws_jclee/src/OVOD_LLM/ovod_llm/captured_frames"
        os.makedirs(self.save_directory, exist_ok=True)
        self.frame_count = 0
        
        ###
        self.label_colors = {}

        # For visualization (optional)
        cv2.namedWindow("YOLOWorld Object Detection", cv2.WINDOW_AUTOSIZE)



    def update_class_callback(self, msg):
        """ /user_text_input topic class update """
        new_class = msg.data.strip().lower() # Receiving the text data
        if not new_class:
            self.get_logger().warn("Received empty class input. Ignoring.")
            return
        
        self.current_class = new_class
        self.model.set_classes([self.current_class]) # YOLOWorld Class setting update
        self.get_logger().info(f"Updated YOLOWorld class to: {self.current_class}")


    def detect_callback(self, data):
        ###======================= 모델 추론 부분 =======================
        # ROS Image 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')  # YOLO는 BGR 형식을 처리 가능
        results = self.model.predict(cv_image)  # YOLOWorld 모델 추론 수행

        detected_objects = []
        object_boxes = []

        ###======================= 결과 시각화 코드 =======================
        # YOLOWorld 결과에서 바운딩 박스, 클래스, 신뢰도 추출
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표
                conf = box.conf[0]  # 신뢰도
                cls = box.cls[0]  # 클래스 ID
                class_name = self.model.names[int(cls)]

                detected_objects.append(class_name)
                object_boxes.append((x1, y1, x2, y2, conf, class_name))

        if detected_objects:
            selected_objects = self.filter_objects_with_clip(detected_objects, object_boxes)
        else:
            selected_objects = []

        

        for (x1, y1, x2, y2, conf, class_name) in selected_objects:
            label = f"{class_name} {conf:.2f}"
            # if class_name not in self.label_colors:
            #     self.label_colors[class_name] = [random.randint(0,255) for _ in range(3)]
            #     label_color = self.label_colors[class_name]
            
            cv2.rectangle(cv_image, (x1 ,y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Displaying the predictions
        cv2.imshow('YOLOWorld + CLIP Object Detection', cv_image)

        ###======================= 캡처 및 저장 및 퍼블리시 =======================
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            # self.frame_count += 1
            file_path = os.path.join(self.save_directory, f"captured_frame.jpg")#f"frame_{self.frame_count:04d}.jpg")
            cv2.imwrite(file_path, cv_image)
            self.get_logger().info(f"-=-=-=-=-=-{file_path}여기에 저장됨-=-=-=-=-=-=")

            # 캡처된 프레임을 퍼블리싱
            captured_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.captured_image_publisher.publish(captured_msg)
            self.get_logger().info("-=-=-=-=-=Published captured frame-=-=-=-=-=")


    def filter_object_with_clip(self, detected_objects, object_boxes):
        """
        user text query + YOLOWorld detected object -> CLIP
        """

def main(args=None):
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
