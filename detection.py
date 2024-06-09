import cv2
import os
import face_recognition
import numpy as np
from PyQt5.QtWidgets import QApplication, QFileDialog, QMessageBox
import copy
# Function to open a file dialog using PyQt5
def open_file_dialog():
    app = QApplication([])
    file_path, _ = QFileDialog.getOpenFileName(None, "Select Video File", "", "Video Files (*.mp4)")
    return file_path

# Prompt the user to choose between camera and video file
choice = input("Choose 'camera' or 'video': ").lower()

if choice == 'camera':
    video_source = 0
elif choice == 'video':
    video_path = open_file_dialog()
    if not video_path:
        print("No video file selected. Exiting.")
        exit()
    video_source = video_path
else:
    print("Invalid choice. Exiting.")
    exit()

# Initialize the video source
cap = cv2.VideoCapture(video_source)

# Function to clear contents of a directory
def clear_directory(directory):
    for root, dirs, files in os.walk(directory):
        for file in files:
            file_path = os.path.join(root, file)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print(f"Failed to delete {file_path}: {e}")
        for dir in dirs:
            dir_path = os.path.join(root, dir)
            try:
                if os.path.isdir(dir_path):
                    os.rmdir(dir_path)
            except Exception as e:
                print(f"Failed to delete {dir_path}: {e}")

# Directory to save images
base_dir = 'segregated_faces'
if not os.path.exists(base_dir):
    os.makedirs(base_dir)
else :
    clear_directory(base_dir)

# Function to check if a face is already known and get the match percentage
def is_known_face(known_face_encodings, face_encoding):
    if not known_face_encodings:
        return False, -1, 0.0
    face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
    best_match_index = np.argmin(face_distances)
    match_percentage = (1 - face_distances[best_match_index]) * 100
    matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
    if matches[best_match_index]:
        return True, best_match_index, match_percentage
    return False, best_match_index, match_percentage

# Load known faces and encodings
known_face_encodings = []
known_face_names = []

frame_skip = 10  # Process every 10th frame
frame_count = 0

# Loop to continuously get frames
while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame")
        break
    frame_count += 1
    if frame_count % frame_skip != 0:
        continue

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Detect faces in the frame and get the face encodings
    face_locations = face_recognition.face_locations(rgb_frame)
    face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
    
    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
        known, index, match_percentage = is_known_face(known_face_encodings, face_encoding)
        frame_copy = copy.deepcopy(frame)
        person_dir = ''
        if known and match_percentage >= 60:
            person_dir = os.path.join(base_dir, known_face_names[index])
        elif match_percentage < 40:
            # If the face is unknown, create a new directory for this person
            person_dir = os.path.join(base_dir, f"person_{len(known_face_encodings)}")
            if not os.path.exists(person_dir):
                os.makedirs(person_dir)
            known_face_encodings.append(face_encoding)
            known_face_names.append(f"person_{len(known_face_encodings) - 1}")

        if person_dir != '':
            # Draw a rectangle around the face
            cv2.rectangle(frame_copy, (left, top), (right, bottom), (255, 0, 0), 2)
            cv2.putText(frame_copy, f"{match_percentage:.2f}%", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Save the full frame image with the bounding box in the corresponding directory
            frame_counter = len([name for name in os.listdir(person_dir) if "frame" in name])
            frame_name = os.path.join(person_dir, f"frame_{frame_counter}.jpg")
            cv2.imwrite(frame_name, frame_copy)
            print(f"Saved {frame_name} with match percentage {match_percentage:.2f}%")
    
    # Draw all bounding boxes on the frame
    for (top, right, bottom, left) in face_locations:
        cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 2)


    # Display the resulting frame
    cv2.imshow('Face Detection', frame)
    
    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video source and close windows
cap.release()
cv2.destroyAllWindows()
