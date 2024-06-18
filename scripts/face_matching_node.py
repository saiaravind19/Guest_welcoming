#!/usr/bin/env python

import rospy
import os
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ai_host.msg import face_cord_list,face_cord
from ai_host.srv import tag_person
from facenet_pytorch import MTCNN, InceptionResnetV1
import torch

class FaceRecognitionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.face_encodings = []
        self.face_names = []
        self.base_dir = 'segregated_faces'
        self.tagged_faces_queue = []
        
        # Initialize MTCNN for face detection and InceptionResnetV1 for face recognition
        self.mtcnn = MTCNN()
        self.model = InceptionResnetV1(pretrained='vggface2').eval()
        
        rospy.Service('process_faces', tag_person, self.handle_process_faces)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.face_pub = rospy.Publisher('/recognized_faces', face_cord_list, queue_size=10)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if len(self.tagged_faces_queue) > 0:
                self.process_faces(self.tagged_faces_queue.pop())
            else:
                rate.sleep()
                
    def handle_process_faces(self, req):
        self.tagged_faces_queue.append(req.name)
        return True
    
    def process_faces(self, dir_name):
        encodings = []
        for image_name in os.listdir(dir_name):
            image_path = os.path.join(dir_name, image_name)
            img = cv2.imread(image_path)
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            # Resize and normalize for InceptionResnetV1
            face_img = cv2.resize(img_rgb, (160, 160))
            face_img = np.transpose(face_img, (2, 0, 1))
            face_img = torch.from_numpy(face_img).unsqueeze(0).float()
            
            # Get embedding using InceptionResnetV1
            with torch.no_grad():
                embedding = self.model(face_img)
            
            encodings.append(embedding.numpy().flatten())
        
        if encodings:
            mean_encoding = np.mean(encodings, axis=0)
            self.face_encodings.append(mean_encoding)
            self.face_names.append(dir_name)
            rospy.loginfo(f"Processed and encoded {dir_name}.")
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        if len(self.face_encodings) > 0:
            boxes, probs = self.mtcnn.detect(rgb_frame)
            
            if boxes is not None:
                faces = face_cord_list()
                counter =0
                for box in boxes:
                     
                    x1, y1, x2, y2 = box.astype(int)  # Ensure box coordinates are integers
                    face_img = rgb_frame[y1:y2, x1:x2]
                    
                    # Resize and normalize for InceptionResnetV1
                    face_img = cv2.resize(face_img, (160, 160))
                    face_img = np.transpose(face_img, (2, 0, 1))
                    face_img = torch.from_numpy(face_img).unsqueeze(0).float()
                    
                    # Get embedding using InceptionResnetV1
                    with torch.no_grad():
                        embedding = self.model(face_img)
                    
                    face_encodings = embedding.numpy().flatten()
                    
                    distances = np.linalg.norm(self.face_encodings - face_encodings, axis=1)
                    min_distance_index = np.argmin(distances)
                    if np.min(distances) < 0.6:  # Adjust threshold as needed
                        name = self.face_names[min_distance_index]
                        counter+=1
                        # Publish face recognition message
                        face_msg = face_cord()
                        face_msg.name = name
                        face_msg.top = y1
                        face_msg.right = x2
                        face_msg.bottom = y2
                        face_msg.left = x1
                        faces.face_list.append(face_msg)
                    else:
                        name = "Unknown"
    
                    rospy.loginfo(f"Detected {name} at {box}")
                    
                if len(self.face_names) > 0:
                    self.face_pub.publish(faces)

if __name__ == "__main__":
    rospy.init_node('face_recognition_node')
    node = FaceRecognitionNode()
    rospy.spin()
