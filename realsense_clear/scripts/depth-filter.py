import pyrealsense2 as rs
import numpy as np
import cv2
import os
import matplotlib.pyplot as plt



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

        # frameデータを取得
        # color_frame = frames.get_color_frame()
        # depth_frame = frames.get_depth_frame()

        # 画像データに変換
        # color_image = np.asanyarray(color_frame.get_data())
        # # 距離情報をカラースケール画像に変換する
        # depth_color_frame = rs.colorizer().colorize(depth_frame)
        # depth_image = np.asanyarray(depth_color_frame.get_data())

        #お好みの画像保存処理
        # depth_data = depth_frame.get_distance(x,y)

        # frame処理で合わせる
        frames = pipe.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        #print(depth_frame.shape)
        #print(depth_frame.type)
        #print(color_frame.shape)
        #print(color_frame.type)

        

        # conf = rs.config()
        # conf.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        # conf.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        # conf.enable_record_to_file('export_filename.bag')  #ADD

        # # stream開始
        # pipe = rs.pipeline()
        # profile = pipe.start(conf)


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


        color_image = np.asanyarray(color_frame.get_data())

        # depth_image = np.asanyarray(result_frame.get_data())
        # nonfil_depth_image = np.asanyarray(depth_frame.get_data())    #uint16に近い画像が得られる

        depth_image = np.asanyarray(result_frame.get_data()).astype(np.uint8)
        nonfil_depth_image = np.asanyarray(depth_frame.get_data()).astype(np.uint8)

        # depth_image = np.asanyarray(result_frame.get_data()).astype(np.uint16)
        # nonfil_depth_image = np.asanyarray(depth_frame.get_data()).astype(np.uint16)

        # depth_image = np.asanyarray(result_frame.get_data()).astype(np.uint32)
        # nonfil_depth_image = np.asanyarray(depth_frame.get_data()).astype(np.uint32)    #TypeError: Expected cv::UMat for argument 'mat'

        # depth_image = np.asanyarray(result_frame.get_data()).astype(np.float32)
        # nonfil_depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32)

        #img = cv2.imread('Figure_1.png')

        # print(img.shape)
        # print(type(img))


        # print(color_image.shape)
        # print(type(color_image))
        # print(depth_image.shape)
        # print(type(depth_image))



        # dp_path = '/home/ericlab/rikuken/pacraft_ggcnn/ggcnn/depth'
        # rgb_path = '/home/ericlab/rikuken/pacraft_ggcnn/ggcnn/rgb'
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

        # print(depth_colormap.shape)
        # print(type(depth_colormap))

        #images = np.hstack((color_image, depth_image))
        #images = np.hstack((color_image, depth_image))
        #images = np.hstack((pil_img, pil_dp))
        #disp_gray = cv2.cvtColor(depth_image,cv2.COLOR_GRAY2BGR)

        #m_h = cv2.hconcat([color_image,disp_gray])
        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('color', color_image)
        cv2.imshow('depth', depth_image)
        cv2.imshow('nonfil_depth', nonfil_depth_image)
        

        # color_image = cv2.imread(color_image)
        # depth_image = cv2.imread(depth_image)
        

        key= cv2.waitKey(0) & 0xFF
        if key == ord('c'):
            cv2.imwrite(os.path.join('depth',str(i)+'_uint8_pefect_depth.tiff'),depth_image)
            cv2.imwrite(os.path.join('color',str(i)+'_uint8_RGB.png'),color_image)
            cv2.imwrite(os.path.join('nonfilter_depth',str(i)+'_uint8_nonfiler_perfect_depth.tiff'),nonfil_depth_image)
        
            i+=1
        elif key == ord('q'):
            break


        # align_to = rs.stream.color
        # align = rs.align(align_to)

        

finally:

    #ストリーミング停止
    pipe.stop()
