from picamera2 import Picamera2
from ultralytics import YOLO
import cv2
from gpiozero import PWMOutputDevice, DigitalOutputDevice
import time

# Motor setup
IN1 = DigitalOutputDevice(17)
IN2 = DigitalOutputDevice(18)
ENA = PWMOutputDevice(23)

IN3 = DigitalOutputDevice(27)
IN4 = DigitalOutputDevice(22)
ENB = PWMOutputDevice(24)

# Speed levels
speed_levels = {
    'slow': 0.3,
    'medium': 0.6,
    'fast': 1.0
}

# Initial states
driving = False
current_speed = speed_levels['medium']

def stop_motors():
    ENA.value = 0
    ENB.value = 0
    IN1.off()
    IN2.off()
    IN3.off()
    IN4.off()

def move_forward(speed):
    IN1.on()
    IN2.off()
    IN3.on()
    IN4.off()
    ENA.value = speed
    ENB.value = speed

# Load trained model
model = YOLO("/home/pi/Desktop/runs/detect/train/weights/best.pt")

# Init camera
picam2 = Picamera2()
picam2.start()

try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        results = model.predict(source=frame, conf=0.5, verbose=False)

        detected_red = False
        detected_green = False
        new_speed = None

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                if conf < 0.60:
                    continue  # Ignore low-confidence detections

                cls = int(box.cls[0])
                label = model.names[cls].lower()

                # Draw label and box
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # Update states
                if "red light" in label or "stop" in label:
                    detected_red = True
                elif "green light" in label:
                    detected_green = True
                elif "speed limit 30" in label:
                    new_speed = speed_levels['slow']
                elif "speed limit 60" in label:
                    new_speed = speed_levels['medium']
                elif "speed limit 100" in label or "speed limit 120" in label:
                    new_speed = speed_levels['fast']


        # Apply logic
        if detected_red:
            print("🛑 Red or Stop sign detected → stopping")
            driving = False
        elif detected_green or new_speed:
            print("✅ Green or Speed sign detected → moving")
            driving = True

        if new_speed:
            current_speed = new_speed
            print(f"🚦 Speed updated → {current_speed}")

        # Control motors
        if driving:
            move_forward(current_speed)
        else:
            stop_motors()

        cv2.imshow("Smart Car View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("🛑 Interrupted by user")

finally:
    stop_motors()
    picam2.close()
    cv2.destroyAllWindows()
