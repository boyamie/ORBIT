# 🛰️ ORBIT (Object Recognition & Intelligent Orbit Navigation)

> **"위대한 비상을 위한 궤도, 미래 드론 기술의 시작점"** > YOLO 기반 실시간 객체 인식 및 PID 제어를 활용한 DJI Tello 자율 추적 드론 프로젝트입니다.

![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![Tello](https://img.shields.io/badge/Drone-DJI_Tello-orange)
![YOLOv8](https://img.shields.io/badge/AI-YOLOv8-yellow)

## 📌 프로젝트 소개
**ORBIT** 프로젝트는 드론 택시 및 무인 드론 배달 시스템의 발전을 위한 기초 자율 비행 기술을 연구합니다. 카메라 비전(YOLO)을 기반으로 대상을 인식하고, 영리하게 궤도를 그리며(Orbit) 자율 비행하는 지능형 시스템을 구현하는 것을 목표로 합니다.

### 🎯 핵심 목표
- **실시간 객체 탐지:** YOLOv8 알고리즘을 활용하여 드론의 카메라 영상에서 특정 객체(사람/얼굴)를 실시간으로 탐지합니다.
- **자율 추적 비행:** 화면 중심과 탐지된 객체 사이의 오차(Pixel Error)를 계산하여, PID 제어기를 통해 드론의 속도 및 방향(상하좌우, 전후진)을 부드럽게 자동 제어합니다.

## 🛠️ 기술 스택 및 요구 사항
- **Hardware:** DJI Tello (또는 Tello EDU)
- **Language:** Python 3.8+
- **Libraries:**
  - `djitellopy`: Tello 드론 제어 및 영상 스트리밍
  - `opencv-python`: 영상 프레임 처리 및 화면 출력
  - `ultralytics`: YOLOv8 모델 로드 및 추론
  - `simple-pid`: 드론 비행의 안정성을 위한 PID 제어 알고리즘 적용

## 📂 리포지토리 구조
```text
ORBIT/
├── main.py                # 전체 시스템 조립 및 실행 파일 (직접 수정 최소화)
├── drone_controller.py    # 드론 비행 명령 및 통신 모듈
├── yolo_detector.py       # YOLO 비전 인식 모듈
├── pid.py                 # PID 제어 오차 계산 모듈
├── config.py              # 드론 속도 제한 및 PID 튜닝 상수 설정
├── requirements.txt       # 프로젝트 실행에 필요한 패키지 목록
└── README.md              # 프로젝트 안내 문서