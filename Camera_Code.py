import cv2          # OpenCV library for camera capture and face detection
import serial       # For serial communication with ESP32
import time         # For delays

# ------------------------------
# SERIAL SETUP
# ------------------------------
# Initialize serial communication to the ESP32 over USB
# Ensure the port matches the one your ESP32 appears as
# Baud rate must match the ESP32 code (115200)
ESP32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Wait 2 seconds to allow ESP32 to initialize and avoid missed commands
time.sleep(2)

# ------------------------------
# FACE DETECTION SETUP
# ------------------------------
# Load Haar cascade classifier for frontal face detection
# Make sure 'haarcascade_frontalface_default.xml' is in the same folder
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# Open the webcam (camera index 0)
cam = cv2.VideoCapture(0)

# Optional: set camera resolution
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Variable for smoothing horizontal face movement
smooth_x = 0

# Variable to store the last command sent to the ESP32
# Prevents sending repeated commands every frame
last_command = None

# ------------------------------
# MAIN LOOP
# ------------------------------
while True:
    # Capture a frame from the webcam
    ret, frame = cam.read()
    
    # If the camera fails to capture a frame, exit the loop
    if not ret:
        print("Cannot read frame")
        break

    # Convert frame to grayscale for faster and easier face detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the frame
    # scaleFactor: image size reduction per step
    # minNeighbors: how many neighbors each rectangle should have to retain it
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

    # Variable to determine which command to send to ESP32
    command = None

    # Process each detected face
    for (x, y, w, h) in faces:
        # Draw a green rectangle around the detected face
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Calculate the center x-coordinate of the face
        face_center_x = x + w // 2

        # Calculate the center x-coordinate of the camera frame
        frame_center_x = frame.shape[1] // 2

        # Calculate horizontal offset
        # Positive --> face is to the right
        # Negative --> face is to the left
        offset_x = face_center_x - frame_center_x

        # Apply smoothing to reduce jittery movements
        smooth_x = 0.8 * smooth_x + 0.2 * offset_x

        # Determine which direction the base should move
        if smooth_x > 50:
            command = b'r'  # Turn the base clockwise (right)
        elif smooth_x < -50:
            command = b'l'  # Turn the base counterclockwise (left)

    # If no face detected or face is near the center, stop servo movement
    if len(faces) == 0 or command is None:
        command = b's'  # Stop servo

    # Only send a command if it has changed since last frame
    # This prevents flooding the ESP32 with repeated commands
    if command != last_command:
        ESP32.write(command)
        last_command = command

    # Display the camera frame with rectangles drawn
    cv2.imshow("Camera View", frame)

    # Exit the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cam.release()           # Release the camera
cv2.destroyAllWindows() # Close OpenCV windows
ESP32.close()           # Close serial communication
