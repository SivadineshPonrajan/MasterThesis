import os
import json
import rospy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
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
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        # In case of adding other dynamic objects
        # self.classes = []
        # with open(self.classes_path, 'r') as f:
        #     self.classes = [line.strip() for line in f.readlines()]
        
        # Create subscribers
        self.image_sub = Subscriber('/usb_cam/image_raw', Image)
        self.pose_sub = Subscriber('/orb_slam3/camera_pose', PoseStamped)
        
        # Set up the ApproximateTimeSynchronizer
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.pose_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)
        
        # Folder to save images
        self.folder_path = os.path.expanduser('~/root/saved_images')
        '''if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)'''

        # Json file to save poses
        self.json_path = os.path.join(self.folder_path, "image_pose.json")

        self.image_pose_data = []
        self.image_counter = 0

        # Publishers
        self.image_pub = rospy.Publisher('/detector/image_raw', Image, queue_size=10)
        self.pose_pub = rospy.Publisher('/detector/camera_pose', PoseStamped, queue_size=10)


    def get_output_layers(self, net):
        layer_names = net.getLayerNames()
        try:
            output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
        except:
            output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        return output_layers

    def callback(self, image_msg, pose_msg):
        try:
            # Convert the image to an OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

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

            self.image_counter += 1
            if not dynamic_object:
                self.publish_results(image_msg, pose_msg)
            else:
                rospy.loginfo('Dynamic object detected in image_{}.png'.format(self.image_counter))

        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def publish_results(self, image_msg, pose_msg):
        try:
            filename = 'image_{}.png'.format(self.image_counter)
            # Construct the full file path
            full_path = os.path.join(self.folder_path, filename)
            # Save the image to the desktop
            '''cv2.imwrite(full_path, cv_image)'''
            rospy.loginfo('Saved image to {}'.format(full_path))

            '''# To Json file
            datum = {
                    "file": filename,
                    "pose": str(pose_msg)
                }
            self.image_pose_data.append(datum)
            with open(self.json_path, 'w') as json_file:
                json.dump(self.image_pose_data, json_file, indent=4)'''

            # publish
            #image_msg_out = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            #image_msg_out.header.stamp = pose_msg.header.stamp
            #self.image_pub.publish(image_msg_out)
            self.image_pub.publish(image_msg)
            
            self.pose_pub.publish(pose_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error {0}".format(e))


if __name__ == '__main__':
    print("detector started")
    detector = detector()
    rospy.spin()