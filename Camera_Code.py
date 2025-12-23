import cv2 # OpenCV for camera and face detection
import serial # For USB communication with the ESP32
import time # For small delays

ESP32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1) # Initializes communication with the Pi USB port at 115200 baud with the ESP32
time.sleep(2) # Wait for the ESP32 to initialize

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# Open the webcam (index 0)
cam = cv2.VideoCapture(0)

# This variable stores the smoothed horizontal face movement.
smooth_x = 0

# Set resolution 
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cam.read()  # Capture frame
    
    if not ret:
        print("Cannot read frame")
        break

    # Convert image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect faces
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5) 

    for (x, y, w, h) in faces:
        # Draw rectangle
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Find center of the face
        face_center_x = x + w // 2

        # Find center of the frame
        frame_center_x = frame.shape[1] // 2

        # Calculate horizontal movement
        offset_x = face_center_x - frame_center_x   # "+" --> right, "-" --> left

        # Smooth the movement
        smooth_x = 0.8 * smooth_x + 0.2 * offset_x

        # The communication, which tells the Pi to send one byte of communication to the ESP32

        if smooth_x > 50:
            ESP32.write(b'r') # Turn the base right
        elif smooth_x < -50:
            ESP32.write(b'l') # Turn the base left
    
    if len(faces) == 0:
        ESP32.write(b's') # Stop moving the servos
    
       # Show the camera view
    cv2.imshow("Camera View", frame)

    # Quit with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
