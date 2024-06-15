#!/usr/bin/env python

import rospy
import os
import cv2
import face_recognition
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ai_host.msg import FaceRecognition
from ai_host.srv import tag_person
import numpy as np

class FaceRecognitionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.face_encodings = []
        self.face_names = []
        self.base_dir = 'segregated_faces'
        self.tagged_faces_queue =[]
        rospy.Service('process_faces', tag_person, self.handle_process_faces)
        self.image_sub = rospy.Subscriber('/webcam_image', Image, self.image_callback)
        self.face_pub = rospy.Publisher('/recognized_faces', FaceRecognition, queue_size=10)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if len(self.tagged_faces_queue) >0:
                self.process_faces(self.tagged_faces_queue.pop())
            else:
                rate.sleep()
                
    def handle_process_faces(self, req):
        self.tagged_faces_queue.append(req.name)
        return True
    
    def process_faces(self,dir_name):
        
        encodings = []
        for image_name in os.listdir(dir_name):
            image_path = os.path.join(dir_name, image_name)
            image = cv2.imread(image_path)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            face_locations = face_recognition.face_locations(image)
            face_encodings = face_recognition.face_encodings(image, face_locations)
            for encoding in face_encodings:
                encodings.append(encoding)
        if encodings:
            mean_encoding = np.mean(encodings, axis=0)
            self.face_encodings.append(mean_encoding)
            self.face_names.append(dir_name)

        rospy.loginfo("Finished processing faces and generating encodings")
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        if len(self.face_encodings) > 0:
            
            face_locations = face_recognition.face_locations(rgb_frame)
            face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
            
            for (top, right, bottom, left), encoding in zip(face_locations, face_encodings):
                name = "Unknown"
                distances = np.linalg.norm(self.face_encodings - encoding, axis=1)
                if len(distances) > 0 and np.min(distances) < 0.6:  # 0.6 is a typical threshold for face recognition
                    best_match_index = np.argmin(distances)
                    name = self.face_names[best_match_index]

                rospy.loginfo(f"Detected {name} at {(left, top, right, bottom)}")
                
                face_msg = FaceRecognition()
                face_msg.name = name
                face_msg.top = top
                face_msg.right = right
                face_msg.bottom = bottom
                face_msg.left = left
                
                self.face_pub.publish(face_msg)

if __name__ == "__main__":
    rospy.init_node('face_recognition_node', anonymous=True)
    node = FaceRecognitionNode()
