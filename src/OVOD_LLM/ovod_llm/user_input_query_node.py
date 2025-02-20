#user_input_query_node.py

import rclpy  # ROS 2의 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS 2에서 노드를 생성하는 기본 클래스
from std_msgs.msg import String  # ROS 2에서 문자열 메시지를 처리하는 메시지 타입
import tkinter as tk  # Tkinter GUI 라이브러리 (사용자 입력을 위한 간단한 인터페이스 제공)

class UserTextInputNode(Node):  # ROS 2 노드를 상속하여 사용자 입력 노드 생성
    def __init__(self):
        super().__init__('user_text_input_node')  # 노드 이름을 'user_text_input_node'로 설정
        
        # 문자열 메시지를 '/user_text_input' 토픽으로 퍼블리시하는 퍼블리셔 생성
        self.publisher_ = self.create_publisher(String, '/user_text_input', 10)

        # Tkinter GUI 설정
        self.window = tk.Tk()  # Tkinter 창 생성
        self.window.title("User Text Input")  # 창의 제목 설정

        # 입력 안내 라벨 추가
        self.label = tk.Label(self.window, text="Enter query for YOLOWorld:")  
        self.label.pack(pady=5)  # 패딩을 추가하여 GUI 배치

        # 텍스트 입력 필드 생성
        self.entry = tk.Entry(self.window, width=30)  # 입력 필드 너비를 30자로 설정
        self.entry.pack(padx=5, pady=5)  # 패딩 추가
        self.entry.bind('<Return>', self.send_text)  # 엔터 키 이벤트 바인딩 추가

        # 버튼 추가 (클릭 시 send_text 함수 실행)
        self.button = tk.Button(self.window, text="Query Send", command=self.send_text)
        self.button.pack(pady=5)  # 패딩 추가

    def send_text(self):
        """ 입력된 텍스트를 '/user_text_input' 토픽으로 퍼블리시하는 함수 """
        text_to_send = self.entry.get()  # 입력 필드에서 텍스트 가져오기
        if text_to_send.strip():  # 입력된 텍스트가 비어있지 않은 경우
            msg = String()  # ROS 2 String 메시지 생성
            msg.data = text_to_send  # 메시지 데이터에 사용자 입력 텍스트 저장
            self.publisher_.publish(msg)  # 토픽 퍼블리시
            self.get_logger().info(f"Sent text: {text_to_send}")  # 퍼블리시한 내용 로그 출력
        else:  # 입력이 비어있으면 경고 메시지 출력
            self.get_logger().warn("Attempted to send empty text...!!")

    def run_gui(self):
        """ Tkinter GUI 실행 함수 """
        self.window.mainloop()  # Tkinter 이벤트 루프 실행

def main(args=None):
    """ ROS 2 노드를 초기화하고 실행하는 메인 함수 """
    rclpy.init(args=args)  # ROS 2 노드 초기화
    node = UserTextInputNode()  # UserTextInputNode 객체 생성

    try:
        node.run_gui()  # GUI 실행
    except KeyboardInterrupt:  # Ctrl+C 입력 시 실행 중단
        pass
    finally:
        node.destroy_node()  # 노드 종료 및 자원 해제
        rclpy.shutdown()  # ROS 2 종료

if __name__ == "__main__":  # 스크립트가 직접 실행될 때만 main 함수 실행
    main()
