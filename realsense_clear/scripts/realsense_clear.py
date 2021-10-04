#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np

def process_image():
    bridge = CvBridge()
    conf = rs.config()
    # RGB
    conf.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # 距離
    conf.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)

    # stream開始
    pipe = rs.pipeline()
    profile = pipe.start(conf)

    align_to = rs.stream.color
    align = rs.align(align_to)

    i=1

    try:
        while True:

            # frame処理で合わせる
            frames = pipe.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # decimarion_filterのパラメータ
            decimate = rs.decimation_filter()
            decimate.set_option(rs.option.filter_magnitude, 1)
            # spatial_filterのパラメータ
            spatial = rs.spatial_filter()
            spatial.set_option(rs.option.filter_magnitude, 1)
            spatial.set_option(rs.option.filter_smooth_alpha, 0.25)
            spatial.set_option(rs.option.filter_smooth_delta, 50)
            # hole_filling_filterのパラメータ
            hole_filling = rs.hole_filling_filter()
            # disparity
            depth_to_disparity = rs.disparity_transform(True)
            disparity_to_depth = rs.disparity_transform(False)

            # 省略(フレーム取得処理)

            #filterをかける
            filter_frame = decimate.process(depth_frame)
            filter_frame = depth_to_disparity.process(filter_frame)
            filter_frame = spatial.process(filter_frame)
            filter_frame = disparity_to_depth.process(filter_frame)
            filter_frame = hole_filling.process(filter_frame)
            result_frame = filter_frame.as_depth_frame()

            depth_image_uint8 = np.asanyarray(result_frame.get_data()).astype(np.uint8)
            uint8imgMsg = bridge.cv2_to_imgmsg(depth_image_uint8, 'passthrough')
            pub = rospy.Publisher('/clear_depth_images_uint8', Image, queue_size=10)
            pub.publish(uint8imgMsg)

            nonfil_depth_image = np.asanyarray(depth_frame.get_data())
            depthimgmsg = bridge.cv2_to_imgmsg(nonfil_depth_image, 'passthrough')
            pub = rospy.Publisher('/depth_img', Image, queue_size=10)
            pub.publish(depthimgmsg)

            color_image = np.asanyarray(color_frame.get_data())
            colorimgmsg = bridge.cv2_to_imgmsg(color_image, 'passthrough')
            pub = rospy.Publisher('/color_img', Image, queue_size=10)
            pub.publish(colorimgmsg)

            depth_image_uint16 = np.asanyarray(result_frame.get_data()).astype(np.uint16)
            uint16imgMsg = bridge.cv2_to_imgmsg(depth_image_uint16, 'passthrough')
            pub = rospy.Publisher('/clear_depth_images_uint16', Image, queue_size=10)
            pub.publish(uint16imgMsg)
            
           
            
            key = cv2.waitKey(0) & 0xFF
            if key == ord('q'):
                break

    finally:

        #ストリーミング停止
        pipe.stop()


   

           
def main():
    rospy.init_node('realsense_clear')
    rospy.loginfo('realsense_clear node started')
    process_image()
    rospy.spin()

if __name__ == '__main__':
    main()