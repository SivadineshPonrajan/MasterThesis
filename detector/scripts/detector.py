import os
# import sys
# sys.path.append(os.path.join(os.path.dirname(__file__), 'yolo'))
# from drink import pepsi

import rospy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from std_msgs.msg import String
from image_publisher.msg import fname
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class detector:
    def __init__(self):
        rospy.init_node('detector', anonymous=True)
        
        # Initialize a CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        pwd_path = os.path.dirname(os.path.abspath(__file__))
        self.config_path = os.path.join(pwd_path, 'yolov3.cfg')
        self.weights_path = os.path.join(pwd_path, 'yolov3.weights')
        self.classes_path = os.path.join(pwd_path, 'yolov3.txt')

        self.net = cv2.dnn.readNet(self.weights_path, self.config_path)

        self.classes = []
        with open(self.classes_path, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]
        
        # Create subscribers
        self.image_sub = Subscriber('image', Image)
        self.filename_sub = Subscriber('filename', fname)
        
        # Set up the ApproximateTimeSynchronizer
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.filename_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)
        
        # Folder to save images
        self.folder_path = os.path.expanduser('~/Desktop/saved_images')
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)


    def get_output_layers(self, net):
        layer_names = net.getLayerNames()
        try:
            output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
        except:
            output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        return output_layers

    def callback(self, image_msg, filename_msg):
        try:
            # Convert the image to an OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        scale = 0.00392
        blob = cv2.dnn.blobFromImage(cv_image, scale, (416,416), (0,0,0), True, crop=False)

        self.net.setInput(blob)
        outs = self.net.forward(self.get_output_layers(self.net))

        conf_threshold = 0.5
        dynamic_object = False

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if class_id == 0 and confidence > conf_threshold:
                    dynamic_object = True
        
        if dynamic_object:
            print("Person detected in image: ", {filename_msg.filename})
        else:
            print("Person not detected in image: ", {filename_msg.filename})

        # Construct the full file path
        full_path = os.path.join(self.folder_path, filename_msg.filename)
        
        # Save the image to the desktop
        cv2.imwrite(full_path, cv_image)
        rospy.loginfo('Saved image to {}'.format(full_path))
        # pepsi("All cool pepsi")

if __name__ == '__main__':
    print("detector started")
    detector = detector()
    rospy.spin()