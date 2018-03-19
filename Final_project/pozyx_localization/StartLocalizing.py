#!/usr/bin/env python2
"""
Modified from The Pozyx ready to localize tutorial (c) Pozyx Labs

This script filters the data read in from the Pozyx and prints it.

TODO: Support definition of tags, tag names, and tag labels in an external file

Before running, make sure:
 - Anchors are calibrated, 
 - A tag is connected to the laptop via USB
 - Other tags are powered and in the "tags" list below
"""
from copy import deepcopy
from pypozyx import *
import rospy
from std_msgs.msg import String

import FilterData as fd
import Interaction as interaction
import ReadyToLocalize as rtl

#def main():
class PozyxLocalization:
    def __init__(self):
        # Initialize ROS stuff
        pub = rospy.Publisher('/pozyx_localization', String, queue_size=10)
        rospy.init_node('pozyx_localization_node', anonymous=True)

        # Shortcut to not have to find out the Pozyx port yourself
        #serial_port = get_serial_ports()[0].device #0x6963
        serial_port = get_first_pozyx_serial_port()
        print serial_port

        # (device_id, x, y, z)
        anchors_simple = [(0x6E30, 0, 0, 1220),
                      (0x6969, 3229, 2361, 1100),
                      (0x6E60, 17151, 3480, 1020),
                      (0x6E52, 7350, 0, 810)]
        anchors = []
        for anc_id, x, y, z in anchors_simple:
            anchors.append(DeviceCoordinates(anc_id, 1, Coordinates(x, y, z)))

        #(tag_id, name, category)
        tags = [(0x6E22, "SquirtleBot", "Robot")]
            #(0x6E13, "Pikachu", "Pokemon"), 
            #(0x6E5A, "Dragonite", "Pokemon"),
            #(0x0000, "Mew", "Pokemon"),
            #(0x0000, "Trainer", "Human")]

        algorithm = POZYX_POS_ALG_UWB_ONLY  # Positioning algorithm to use
        dimension = POZYX_3D                # Positioning dimension
        height = 1000                       # Height of device (2.5D positioning)

        # Create a new FilterData instance for each tag
        # Filter using past 10 points; Ignore points 500 mm away from previous point
        filter_data = [fd.FilterData(10, 500) for i in range(len(tags))]
        # Used for determining interaction between tags
        # 200 mm between tags suggests interaction; use the last 10 top interactions
        interact = interaction.Interaction(tags, 200, 10)

        pozyx = PozyxSerial(serial_port)
        r = rtl.ReadyToLocalize(pozyx, tags, anchors, filter_data, interact, pub,
                                        algorithm, dimension, height)
        r.setup()    
    
        rate = rospy.Rate(5) # 5 Hz
        while not rospy.is_shutdown():
            r.loop()
            rate.sleep()

if __name__ == "__main__":
    try:
        PozyxLocalization()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.login("Pypozyx localization terminated.")
