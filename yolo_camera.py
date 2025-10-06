from picamera2 import Picamera2
from ultralytics import YOLO
import cv2

# Load trained model
model = YOLO("/home/pi/Desktop/runs/detect/train/weights/best.pt")

# Start the camera
picam2 = Picamera2()
picam2.start()

while True:
    frame = picam2.capture_array()

    # Convert BGRA to BGR (remove alpha)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

    # Predict (no show=True)
    results = model.predict(source=frame, conf=0.5, verbose=False)

    # Draw results on the frame
    for r in results:
        for box in r.boxes:
            # Get box coordinates
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = model.names[cls]
            # Draw rectangle
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Show the frame
    cv2.imshow("YOLOv8 Camera", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.close()
cv2.destroyAllWindows()

