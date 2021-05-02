#!/usr/bin/env python

import rospy
from terple_msgs.msg import RobotExternalCharacteristics, DecentralizedRobotReadings
from threading import Lock

ROS_NODE_NAME = "terple_decentralized_sensing_simulator"

def get_rosparam(name):
    value = None
    if rospy.has_param(name):
        value = rospy.get_param(name)
    return value

def printr(text):
    stdout.write("\r" + text)
    stdout.flush()

# A class to hold the external characteristics heard by the robots
class ExtCharCollection(object):

    # Mutex to access the characteristics in this object safely across threads
    mutex = None

    # A dict where the key is the unique robot ID and the value is the
    # entire external characteristics msg
    characteristics = None

    # Initialize with no entries
    def __init__(self):
        self.mutex = Lock()
        self.characteristics = {}

    # Update the characteristics given the data
    def update(self, updated_chars):
        self.mutex.acquire()
        try:
            self.characteristics[updated_chars.robot_id] = updated_chars
        finally:
            self.mutex.release()

    # Export the collection of characteristics to the ROS msg output
    def to_robot_readings_list(self):
        success = False
        results = DecentralizedRobotReadings()
        self.mutex.acquire()
        try:
            results.data = self.characteristics.values()
            success = True
        finally:
            self.mutex.release()
        return success, results

# Add the updated robot characteristics to the object collecting them
def robot_external_characteristics_callback(msg, collection):
    collection.update(msg)

def main():
    # Init ROS elements and get parameters
    rospy.init_node(ROS_NODE_NAME)
    update_rate = get_rosparam("/terple/decentralized/update_rate")
    assert(update_rate != None)

    # Create collection of robot external characteristics
    ext_char_collection = ExtCharCollection()

    # Init ROS pub and sub
    characteristics_updates_sub = rospy.Subscriber(
        "/terple/decentralized/characteristics_updates",
        RobotExternalCharacteristics,
        lambda m: robot_external_characteristics_callback(m, ext_char_collection),
        queue_size=100
    )
    robot_readings_pub = rospy.Publisher(
        "/terple/decentralized/robot_readings",
        DecentralizedRobotReadings,
        queue_size=1
    )

    # Start spin routine
    ros_rate = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        success, robot_readings = ext_char_collection.to_robot_readings_list()
        if success:
            robot_readings_pub.publish(robot_readings)
        else:
            print "ERROR: failed to collect robot readings."
        ros_rate.sleep()

if __name__ == "__main__":
    main()
