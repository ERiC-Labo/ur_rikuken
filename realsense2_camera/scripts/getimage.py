import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo

import cv2

bridge = CvBridge()
rospy.init_node('get_depth')
def depth_callback(depth_message):
    depth = bridge.imgmsg_to_cv2(depth_message)
    cv2.imwrite('/home/ericlab/hamamoto/ur_ws/src/ur_rikuken/realsense2_camera/scripts/depth_img_720x720.png', depth)
    img_height, img_width = depth.shape[:2]
    print(img_height)
    print(img_width)
    
def Image_callback(Image_message):
    image = bridge.imgmsg_to_cv2(Image_message)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    cv2.imwrite('/home/ericlab/hamamoto/ur_ws/src/ur_rikuken/realsense2_camera/scripts/Image_720x720.png', image)
depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback, queue_size = 1)
Image_sub = rospy.Subscriber('/camera/color/image_raw', Image, Image_callback, queue_size = 1)

while not rospy.is_shutdown():
    rospy.spin()