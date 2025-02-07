import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from openai import OpenAI
import os
from dotenv import load_dotenv
import base64




class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        # 구독할 캡처된 프레임 토픽 설정
        self.image_subscription = self.create_subscription(
            Image,
            'captured/image_raw',
            self.callback,
            10
        )

        self.bridge = CvBridge()
        self.get_logger().info("llm node 시작 카메라 노드에서 캡쳐하시오")

        # 환경 변수 로드
        
        load_dotenv("/home/rcv/ros2_ws/src/ovod_llm/.env")
        self.api_key = os.getenv("OPENAI_API_KEY")

        if not self.api_key:
            self.get_logger().error("OpenAI API Key 를 .env에서 찾을 수 없음.")
            raise ValueError("API Key 못찾음.")

        # OpenAI 클라이언트 설정
        self.client = OpenAI(api_key=self.api_key)
        #-=-=-=-=-=-=-=-=-=-=-=-=-모호한 표현 + 프롬프팅-=-=-=-=-=-=-=-=-=-=-=-=-=-
        self.prompt = "해당 사진에 대해서 간단하게 설명해줘"
    # 파일시스템 이미지 인코딩
    # def encode_image(image_path):
    #     with open(image_path, "rb") as image_file:
    #         return base64.b64encode(image_file.read()).decode('utf-8')

    def callback(self, msg):
        try:
            print("callback함수가 실행됩니다. 캡쳐된 프레임 받음!")
            # ROS 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # # 이미지를 파일로 저장 후 아래과정 진행하는 코드
            # # OpenCV 이미지를 특정 파일 형식(여기서는 .jpg)으로 메모리(buffer)에 저장(실제로 파일로 저장하지 않고, 이미지 데이터를 파일 형식의 바이트 배열로 변환하는 것)
            # # OpenCV 이미지를 GPT API로 전달하기 전에 표준 파일 형식으로 변환해야 함!
            # _, buffer = cv2.imencode('.jpg', cv_image) # buffer에 .jpg형식으로 cv_image(numpy배열) 저장
            # image_base64 = buffer.tobytes() # buffer에 들어있는 numpy배열을 바이트 데이터(바이너리데이터를 8비트 단위로 쪼갠)로 변경
            
            # OpenCV 이미지(cv_image)를 JPG로 인코딩
            _, buffer = cv2.imencode('.jpg', cv_image)
            # 버퍼 데이터를 Base64로 인코딩
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # image_base64 = self.encode_image('/home/rcv/ros2_ws/src/ovod_llm/ovod_llm/captured_frames/captured_frame.jpg')

            
            
            self.get_logger().info("...GPT 답변 생성중...")

            # GPT API 요청 데이터 구성 및 요청
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                store=False, # 요청 데이터를 OpenAI의 서버에 저장할지를 결정
                messages=[
                    {"role": "system", "content": "You are an AI assistant that describes images. 답변할 때는 한국어로 답변해줘"},
                    {"role": "user", "content": [
                        {"type": "text", "text": f"{self.prompt}"},
                        {"type": "image_url", "image_url": {
                            "url": f"data:image/jpg;base64,{image_base64}"}
            }  
        ]}
                    # {"role": "user", "content": self.prompt},
                    # {"role": "user", "content": f"Image: {image_base64}"}
                ]
            )
            
        

            # 응답 처리
            answer = response.choices[0].message.content#["content"]
            self.get_logger().info(f"GPT Response: {answer}")

        except Exception as e:
            self.get_logger().error(f"erorororororoororoororor LLMNode: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
