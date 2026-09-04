# Smart Car

An autonomous smart car powered by YOLOv8 object detection. It reads traffic signs and signals from a live camera feed and controls a dual-motor chassis in real time.

## How It Works

The system uses a Raspberry Pi camera module to capture frames, runs YOLOv8 inference to detect traffic signs and lights, and maps detections to motor commands:

- **Red light / Stop sign** → full stop
- **Green light** → resume driving
- **Speed limit 30** → slow (30% PWM)
- **Speed limit 60** → medium (60% PWM)
- **Speed limit 100/120** → fast (100% PWM)

## Tech Stack

- **YOLOv8** — real-time object detection via [Ultralytics](https://github.com/ultralytics/ultralytics)
- **Raspberry Pi** — camera input (Picamera2) and GPIO motor control
- **OpenCV** — frame processing and visualization
- **gpiozero / RPi.GPIO** — L298N motor driver interface (dual DC motors with PWM speed control)

## Project Structure

```
smart-car/
├── final.py          # Main loop: camera → YOLO → motor control
├── yolo_camera.py    # Standalone camera + YOLO detection viewer
├── motor_test.py     # Simple forward-motor test script
├── train/            # Training artifacts (curves, confusion matrices, weights)
└── best.pt           # Trained YOLOv8 weights
```

## Hardware

- Raspberry Pi (3B+ or newer)
- Pi Camera Module (or USB webcam with Picamera2 support)
- L298N motor driver
- 2x DC gear motors + chassis
- Power supply (battery pack for Pi + motors)

## GPIO Wiring

| Signal | GPIO Pin | Purpose |
|--------|----------|---------|
| IN1    | 17       | Motor A direction |
| IN2    | 18       | Motor A direction |
| ENA    | 23       | Motor A PWM (speed) |
| IN3    | 27       | Motor B direction |
| IN4    | 22       | Motor B direction |
| ENB    | 24       | Motor B PWM (speed) |

## Usage

Install dependencies:

```bash
pip install ultralytics opencv-python picamera2 gpiozero
```

Run the main loop:

```bash
python final.py
```

Press `q` to quit. Edit the model path in `final.py` if your weights are in a different location.

## Training

The model was trained on a custom traffic sign/light dataset using YOLOv8. Training results and metrics are in `train/`. To retrain:

```bash
yolo detect train data=<your-dataset.yaml> model=yolov8n.pt epochs=100 imgsz=640
```

## License

MIT
