import pyrealsense2 as rs
import numpy as np
import sys
import cv2
from std_msgs.msg import String, Float32

DRAW_GRID = False

def talker():
    pub = rospy.Publisher('depth_frame', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        distance = grid()
        pub.publish(distance)
        rate.sleep()


def grid(color_image, w_portion, h_portion, w_color, h_color, thickness):
    # horizontal lines
    cv2.line(color_image, (0, w_portion), (640, w_portion), (w_color, 0, 0), thickness, 1)
    cv2.line(color_image, (0, 2*w_portion), (640, 2*w_portion), (w_color, 0, 0), thickness, 1)
    cv2.line(color_image, (0, 3*w_portion), (640, 3*w_portion), (w_color, 0, 0), thickness, 1)
    cv2.line(color_image, (0, 4*w_portion), (640, 4*w_portion), (w_color, 0, 0), thickness, 1)

    # vertical line
    cv2.line(color_image, (h_portion, 0), (h_portion, 480), (0, 0, h_color), thickness, 1)
    cv2.line(color_image, (2*h_portion, 0), (2*h_portion, 480), (0, 0, h_color), thickness, 1)
    cv2.line(color_image, (3*h_portion, 0), (3*h_portion, 480), (0, 0, h_color), thickness, 1)
    cv2.line(color_image, (4*h_portion, 0), (4*h_portion, 480), (0, 0, h_color), thickness, 1)
    cv2.line(color_image, (5*h_portion, 0), (5*h_portion, 480), (0, 0, h_color), thickness, 1)

    if DRAW_GRID:
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

    try:
        pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # Start streaming
        pipeline.start(config)

        # filters
        hole_filling = rs.hole_filling_filter()

        # get camera intrinsics
        profile = pipeline.get_active_profile()
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()


        h_portion = int(640*(1.0/5.0))
        w_portion = int(480*(1.0/5.0))
        print(h_portion, w_portion)

        while True:
            # This call waits until a new coherent set of frames is available on a device
            # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable
            print('Getting frame data now')
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            # depth_frame = hole_filling.process(depth_frame)
            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))

            right_image = depth_image[ 2*w_portion:4*w_portion , 4*h_portion: ]

            right_distances = depth_scale*right_image

            mean = right_distances[right_distances > 0]

            if DRAW_GRID:
                grid(color_image, w_portion, h_portion, 255, 255, 2)
                cv2.imshow('RealSense', images)
                cv2.waitKey(1)
        pipeline.stop()

        return float(mean)

    except Exception as e:
        print('except : ', e)
        pass


if __name__ == '__main__':
try:
    talker()
except rospy.ROSInterruptException:
    pass