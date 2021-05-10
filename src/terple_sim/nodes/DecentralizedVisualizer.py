#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from math import sqrt
from terple_msgs.msg import DecentralizedCombinedPaths

ROS_NODE_NAME = "terple_decentralized_visualizer"

# Number of pixels on both the y and x axes
MY_CV_SIZE = 800
MY_CV_HALF_SIZE = MY_CV_SIZE//2
MY_CV_ORIGIN = (MY_CV_HALF_SIZE,MY_CV_HALF_SIZE)
# Cartesian length of one quadrant for both the x and y axes
MY_CV_SPACE = 5.0
# Pixels/meter, accounting for all four quadrants
MY_CV_SCALE = MY_CV_SIZE / (2*MY_CV_SPACE)

def white_image_with_axes(shape):
    img = np.ones(shape, dtype=np.uint8)*255
    #img = cv2.line(img, (MY_CV_HALF_SIZE,0), (MY_CV_HALF_SIZE,MY_CV_SIZE), (0,0,0), 2)
    #img = cv2.line(img, (0,MY_CV_HALF_SIZE), (MY_CV_SIZE,MY_CV_HALF_SIZE), (0,0,0), 2)
    return img

def scale(v):
    nx = int(v.x*MY_CV_SCALE)+MY_CV_HALF_SIZE
    ny = MY_CV_HALF_SIZE-int(v.y*MY_CV_SCALE)
    return nx, ny

def dist(v1, v2):
    return sqrt((v2.x-v1.x)**2 + (v2.y-v1.y)**2)

def render(msg):
    img_render = white_image_with_axes((MY_CV_SIZE,MY_CV_SIZE,3))

    for i in range(len(msg.paths)):
        idx = None
        for j in range(1, len(msg.paths[i].data)):
            if dist(msg.paths[i].data[0], msg.paths[i].data[j]) < 0.01:
                idx = j
            else:
                break
        if idx is not None:
            msg.paths[i].data = [msg.paths[i].data[0]] + msg.paths[i].data[idx+1:]

    window_name = str(np.random.randint(np.iinfo(np.int64).max))
    colors = {rid:np.random.randint(255, size=3) for rid in msg.robot_ids}
    prevs = {rid:None for rid in msg.robot_ids}
    distances = {rid:0 for rid in msg.robot_ids}
    max_length_path = max(len(p.data) for p in msg.paths)
    for pidx in range(max_length_path):
        for robot_id, path in zip(msg.robot_ids, msg.paths):
            if pidx < len(path.data):
                color = colors[robot_id]
                prev = prevs[robot_id]
                position = path.data[pidx]
                #img_render = cv2.circle(img_render, scale(position), 5, color, -1)
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
                    distances[robot_id] = distances[robot_id] + sqrt((position.x-prev.x)**2 + (position.y-prev.y)**2)
                prevs[robot_id] = position

        cv2.imshow(window_name, img_render)
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)

    num_positions = {rid:len(path.data) for rid, path in zip(msg.robot_ids, msg.paths)}
    print "\n".join("{0} : {1},{2}".format(k,num_positions[k],d) for k,d in distances.items())

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
