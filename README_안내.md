# 7주차 ROS2 + Camera + OVD 일정

## 🔗 실습 관련 자료

### 📂 ROS2 기초 실습 topic, service, action 노션

[![Notion](https://img.shields.io/badge/Notion-000000?style=for-the-badge&logo=notion&logoColor=white)](https://spot-swoop-3a0.notion.site/7-ROS2-19d1f27600f780da81ccf49581d06517?pvs=4)

### 🐢 ROS2 Turtlesim 실습
[![Notion](https://img.shields.io/badge/Notion-000000?style=for-the-badge&logo=notion&logoColor=white)](https://handsomely-marjoram-14d.notion.site/ROS2-Turtlesim-19f7023971eb80829a91c4f589b6fd2e?pvs=4)

### 📷 Webcam 실습
[![Notion](https://img.shields.io/badge/Webcam%20실습-1a070239?style=for-the-badge&logo=notion&logoColor=white)](https://handsomely-marjoram-14d.notion.site/webcam-1a07023971eb80e893fed384f9f98b30?pvs=4)

### 🖼️ SSD Detection 실습
[![Notion](https://img.shields.io/badge/ROS2%20SSD%20Detection-FF6F00?style=for-the-badge&logo=notion&logoColor=white)](https://handsomely-marjoram-14d.notion.site/ROS2-SSD-Detection-1a07023971eb80a09e65f2fe1e90db09?pvs=4)

### 🛠 YOLO World Detection & Query Nodes
#### 📥 YOLO World Weights 다운로드
[![Google Drive](https://img.shields.io/badge/Download-YOLO%20World-4285F4?style=for-the-badge&logo=googledrive&logoColor=white)](https://drive.google.com/drive/folders/1D6fT7jRrgdAIvwt4HQS8_759zAnJv2lo?usp=sharing)

#### 🎯 Detection Node (YOLO World)
[![Notion](https://img.shields.io/badge/Detection%20Node%20YOLO%20World-FFAA00?style=for-the-badge&logo=notion&logoColor=white)](https://handsomely-marjoram-14d.notion.site/3-Detection_Node_YoloWorld-1917023971eb80e7a970e1539dbc9426?pvs=4)

#### 📝 Text Query Node
[![Notion](https://img.shields.io/badge/Text%20Query%20Node-00AAFF?style=for-the-badge&logo=notion&logoColor=white)](https://handsomely-marjoram-14d.notion.site/4-1-Text_Query_Node-1917023971eb8008a62ce1461bde7bbe?pvs=4)

#### 🔍 Detection Node (YOLO World + Query)
[![Notion](https://img.shields.io/badge/Detection%20Node%20YOLO%20World%20Query-FF4455?style=for-the-badge&logo=notion&logoColor=white)](https://handsomely-marjoram-14d.notion.site/4-2-Detection_Node_YoloWorld_Query-1a07023971eb8079a6c6fc26ad644ad0?pvs=4)





---
## 💡 팁
- 터미널에 복붙을 하고 싶을 땐, ctrl+shift+c / ctrl+shift+v 하면 돼요~
- Jetson 보드 비번 : 2025rcvwinter



## 💡 이번주 목표

- Camera에 대한 이해
- Jetson Orin Nano & Ubuntu & ROS2 에 대한 이해
- ROS2 를 이용한 topic, service, action 에 대한 이해 및 실습
- ROS2 + OVD(Open Vocabulary Detection) 심화 실습

## 💡 진행 시 참고
Jetson Control with jetson-stats
    - jetson-stats이 제공하는 Jetson 시리즈 시스템 모니터링 Package
```
# Install jtop (현재는 이미 설치된 상태)
sudo -H pip install -U jetson-stats

# Jetson-stats 기능 실행
jtop
```

## 📌 1일차

- Camera Grabber 세미나
- ROS2 기초 세미나
- Jetson Orin Nano 환경설정 (생략, 대부분의 setting 요소 이미 설치)
- ROS2 topic, service, action 실습


## 📌 2일차

- ROS2 웹캠 실습
- ROS2 SSD detection 실습

## 📌 3일차

- OVD(Open Vocabulary Detection) 세미나
- ROS2 + OVD(Open Vocabulary Detection) 실습
- ROS2에 대한 이해를 바탕으로 스켈레톤 코드 작성
- 코드에 대한 전반적인 이해


## 📌 (참고) Linux 환경에서 sejong Wifi 사용
- ![image](https://github.com/sejong-rcv/2024.URP.Winter/assets/81506870/8f973c06-fa25-41fc-8ed1-1a025b484827)

- (1) Security: WPA & WPA2 Enterprise
- (2) Authentication: Protected EAP (PEAP)
- (3) Check No certificate
- (4) Inner authentication: GTC
- (5) Username & Password: Sejong Portal ID & PW


## 📌 (참고) Software Updater 관련
- ![image](https://github.com/user-attachments/assets/1e8430a7-bf02-4020-9297-8343369c040f)

- 이거 뜨면 Remind Me Later 눌러주세요!


## 💻 VSCode 설치 가이드 for Ubuntu
[![Notion](https://img.shields.io/badge/VSCode%20Install-007ACC?style=for-the-badge&logo=visualstudiocode&logoColor=white)](https://handsomely-marjoram-14d.notion.site/vscode-install-setup-19d7023971eb80938526e15d438fbd1d?pvs=4)

---
