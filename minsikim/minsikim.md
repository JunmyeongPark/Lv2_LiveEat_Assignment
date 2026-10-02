hi, minsikim's workplace

## 폴더 구조

```
minsikim/
├── filter_FOV.py            RealSense D435 RGB/Depth FOV 시각화
└── yolo/
    ├── realsense_gpt.py     실행: YOLO/HSV 검출 + 깊이 + 녹화(webm)
    ├── tuto.py              RealSense 마우스 위치 3D 좌표 예제
    ├── tools/
    │   └── prepare_dataset.py   makesense YOLO zip → dataset/
    ├── data/labels/         makesense export zip        (git 제외)
    ├── models/              사전학습 모델 yolo26n.pt     (git 제외)
    ├── dataset/             학습용 데이터, 자동 생성      (git 제외)
    ├── runs/                학습 결과 + train.log       (git 제외)
    └── .yolo/               가상환경                    (git 제외)
```

## 실행

```
cd minsikim/yolo
source .yolo/bin/activate

python realsense_gpt.py
```

키: `D` 검출 ON/OFF · `M` YOLO/HSV 전환 · `R` 녹화 · `Q` 종료

YOLO 모델: `runs/target_blue/weights/best.pt`

## 학습

```
python tools/prepare_dataset.py data/labels/<export>.zip <프레임 폴더>
yolo detect train model=models/yolo26n.pt data=dataset/data.yaml imgsz=640 epochs=100 project=$PWD/runs name=target_blue
```

`project=`는 절대 경로로 줘야 함 (ultralytics 전역 설정 runs_dir가 다른 경로를 가리킴)
