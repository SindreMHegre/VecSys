#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Initialize the CvBridge class
bridge = CvBridge()

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "8UC1")

        # Apply Gaussian blur
        blurred_image = cv2.GaussianBlur(cv_image, (15, 15), 0)

        # Convert OpenCV image back to ROS Image message
        blurred_msg = bridge.cv2_to_imgmsg(blurred_image, "8UC1")

        # Set the timestamp of the published message to be the same as the received message
        blurred_msg.header.stamp = msg.header.stamp

        # Publish the processed image
        pub.publish(blurred_msg)

        rospy.loginfo('Published blurred image')

    except CvBridgeError as e:
        rospy.logerr(f'CvBridge Error: {e}')

if __name__ == '__main__':
    rospy.init_node('image_processor')

    rospy.loginfo('Image Processor node started')

    # Publisher for processed images
    pub = rospy.Publisher('/processed_image', Image, queue_size=10)

    # Subscriber for raw images
    rospy.Subscriber('/blackfly_left/blackfly_left', Image, image_callback)

    # Keep the node running
    rospy.spin()