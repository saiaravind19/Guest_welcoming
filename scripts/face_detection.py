#!/usr/bin/env python

import copy
import rospy
import cv2
import os
import face_recognition
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ai_host.srv import tag_person
from ai_host.msg import FaceRecognition


class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage)

    def __init__(self, video_source=0):
        super().__init__()
        self._run_flag = True
        self.video_source = video_source
        self.known_face_encodings = []
        self.known_face_names = []
        self.base_dir = 'segregated_faces'

        rospy.init_node('webcam_image_publisher', anonymous=True)
        self.image_pub = rospy.Publisher('/webcam_image', Image, queue_size=10)
        self.bridge = CvBridge()

        self.tag_face = False
        self.tagged_face_dir = None
        self.tag_timer = None
        self.recognized_faces = []

        rospy.Subscriber('/recognized_faces', FaceRecognition, self.recognized_faces_callback)



    def clear_directory(self, directory):
        if not os.path.exists(directory):
            os.makedirs(directory)
            print(f"Creating directory: {directory}")
        else:
            for filename in os.listdir(directory):
                file_path = os.path.join(directory, filename)
                try:
                    if os.path.isfile(file_path) or os.path.islink(file_path):
                        os.unlink(file_path)
                    elif os.path.isdir(file_path):
                        os.rmdir(file_path)
                except Exception as e:
                    print(f'Failed to delete {file_path}. Reason: {e}')
            print(f"Cleared directory: {directory}")

    def recognized_faces_callback(self, msg):
        self.recognized_faces.append(msg)

    def run(self):
        cap = cv2.VideoCapture(self.video_source)
        while self._run_flag:
            ret, frame = cap.read()
            if not ret:
                break

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            face_locations = face_recognition.face_locations(rgb_frame)
            face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
            
            if len(self.recognized_faces)>0:
                detection_data = self.recognized_faces.pop()
                cv2.rectangle(frame, (detection_data.left, detection_data.top), (detection_data.right, detection_data.bottom), (0, 255, 0), 2)
                cv2.putText(frame, detection_data.name.split('/')[-1], (detection_data.left, detection_data.top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            else :
                for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
                    if self.tag_face and self.tagged_face_dir:
                        face_image = frame[top:bottom, left:right]
                        frame_counter = len([name for name in os.listdir(self.tagged_face_dir) if "frame" in name])
                        frame_name = os.path.join(self.tagged_face_dir, f"frame_{frame_counter}.jpg")
                        cv2.imwrite(frame_name, face_image)
                        print(f"Saved {frame_name}")

                for (top, right, bottom, left) in face_locations:
                    cv2.rectangle(frame, (left, top), (right, bottom), (255, 0, 0), 2)

            # Convert frame to QImage and emit signal
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            convert_to_qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            p = convert_to_qt_format.scaled(640, 480, aspectRatioMode=0)
            self.change_pixmap_signal.emit(p)

            # Publish frame to ROS
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(ros_image)

        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

    def start_tagging(self, tag_name):
        print("Tagging started")
        self.tag_face = True
        self.tagged_face_dir = os.path.join(self.base_dir, tag_name)
        self.clear_directory(self.tagged_face_dir)

        self.tag_timer = QTimer()
        self.tag_timer.timeout.connect(self.stop_tagging)
        self.tag_timer.start(3000)  # Stop tagging after 3 seconds

    def stop_tagging(self):
        self.tag_face = False
        print("Timeout for tagging faces")

        if self.tag_timer:
            self.tag_timer.stop()
            self.tag_timer = None
        # Check if the service exists before calling it
        #service_list = rospy.get_published_services()
        #if '/start_tagging' in service_list:
        # Call the start_tagging service in FaceRecognitionNode
        try:
            start_tagging_service = rospy.ServiceProxy('process_faces', tag_person)
            response = start_tagging_service(str(self.tagged_face_dir))
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")       
            


