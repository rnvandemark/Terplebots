#!/usr/bin/env python

import rospy
from terple_msgs.msg import RobotExternalCharacteristics, DecentralizedRobotReadings

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

    # A dict where the key is the unique robot ID and the value is the
    # entire external characteristics msg
    characteristics = None

    # Initialize with no entries
    def __init__(self):
        self.characteristics = {}

    # Update the characteristics given the data
    def update(self, updated_chars):
        self.characteristics[updated_chars.robot_id] = updated_chars
        return self.characteristics

# Add the updated robot characteristics to the object collecting them
def robot_external_characteristics_callback(msg, collection, pub):
    update_results = collection.update(msg)
    assert(update_results != None)
    pub.publish(DecentralizedRobotReadings(data=update_results.values()))

def main():
    # Create collection of robot external characteristics
    ext_char_collection = ExtCharCollection()

    # Init ROS elements
    rospy.init_node(ROS_NODE_NAME)
    robot_readings_pub = rospy.Publisher(
        "/terple/decentralized/robot_readings",
        DecentralizedRobotReadings,
        queue_size=1
    )
    characteristics_updates_sub = rospy.Subscriber(
        "/terple/decentralized/characteristics_updates",
        RobotExternalCharacteristics,
        lambda m: robot_external_characteristics_callback(m, ext_char_collection, robot_readings_pub),
        queue_size=100
    )

    # Start spin routine
    rospy.spin()

if __name__ == "__main__":
    main()
