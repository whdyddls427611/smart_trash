AI 기반 스마트 쓰레기 분류 시스템

라즈베리파이(Raspberry Pi)와 YOLOv8 객체 탐지 모델을 활용해 쓰레기를 자동으로 분류하고,  
정확하게 분리배출하면 포인트가 적립되는 스마트 분리수거 시스템입니다.

---

프로젝트 개요
이 프로젝트는 카메라로 촬영된 영상을 실시간으로 분석하여  
캔 / 유리 / 플라스틱을 자동으로 인식하고, 해당 쓰레기통의 뚜껑을 자동으로 열어주는 시스템입니다.  
또한 올바르게 분리배출하면 웹페이지에 포인트가 적립됩니다.

---
### AI / 모델링
- YOLOv8 (Ultralytics)
- Roboflow – 데이터셋 구축 및 라벨링
- TensorFlow Lite – 모델 경량화 및 라즈베리파이 추론

### 임베디드 / IoT
- Raspberry Pi 4
- Raspberry Pi Camera Module 3
- Servo Motor, LED, Sound Sensor, Ultrasonic Sensor
- GPIO 제어 (Python)

---
### 파일명 - 역할
- colab.txt - 데이터셋 학습용
- login.html - 연동된 웹사이트
- trash5.py - 실행할 프로그램
- test.py - pc환경에서 학습된 데이터 테스트용
