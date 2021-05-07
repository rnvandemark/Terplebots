#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from terple_msgs.msg import DecentralizedCombinedPaths

ROS_NODE_NAME = "terple_decentralized_visualizer"

# Number of pixels on both the y and x axes
MY_CV_SIZE = 800
MY_CV_HALF_SIZE = MY_CV_SIZE//2
MY_CV_ORIGIN = (MY_CV_HALF_SIZE,MY_CV_HALF_SIZE)
# Cartesian length of one quadrant for both the x and y axes
MY_CV_SPACE = 3.0
# Pixels/meter, accounting for all four quadrants
MY_CV_SCALE = MY_CV_SIZE / (2*MY_CV_SPACE)

def white_image_with_axes(shape):
    img = np.ones(shape, dtype=np.uint8)*255
    img = cv2.line(img, (MY_CV_HALF_SIZE,0), (MY_CV_HALF_SIZE,MY_CV_SIZE), (0,0,0), 2)
    img = cv2.line(img, (0,MY_CV_HALF_SIZE), (MY_CV_SIZE,MY_CV_HALF_SIZE), (0,0,0), 2)
    return img

def scale(v):
    nx = int(v.x*MY_CV_SCALE)+MY_CV_HALF_SIZE
    ny = MY_CV_HALF_SIZE-int(v.y*MY_CV_SCALE)
    return nx, ny

def render(msg):
    img_render = white_image_with_axes((MY_CV_SIZE,MY_CV_SIZE,3))
    for robot_id, path in zip(msg.robot_ids, msg.paths):
        color = np.random.randint(255, size=3)
        prev = None
        for position in path.data:
            img_render = cv2.circle(img_render, scale(position), 5, color, -1)
            if prev is None:
                img_render = cv2.putText(
                    img_render,
                    "#{0}".format(robot_id),
                    scale(position),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    color,
                    2,
                    cv2.LINE_AA
                )
            else:
                img_render = cv2.line(img_render, scale(prev), scale(position), color, 2)
            prev = position

    window_name = str(np.random.randint(np.iinfo(np.int64).max))
    cv2.imshow(window_name, img_render)
    cv2.waitKey(0)
    cv2.destroyWindow(window_name)

if __name__ == "__main__":
    # Init ROS elements and wait for paths
    rospy.init_node(ROS_NODE_NAME)
    robot_readings_sub = rospy.Subscriber(
        "/terple/decentralized/combined_paths",
        DecentralizedCombinedPaths,
        render,
        queue_size=1
    )
    rospy.loginfo("Decentralized visualizer ready.")
    rospy.spin()
    cv2.destroyAllWindows()
