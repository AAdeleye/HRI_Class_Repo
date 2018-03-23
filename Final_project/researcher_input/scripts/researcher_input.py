#!/usr/bin/env python

import rospy, select, sys
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/researcher_input', String, queue_size=10)
    rospy.init_node('researcher_input_node', anonymous=True)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        userinput = select.select([sys.stdin],[],[],0.0)[0]
        if userinput:
            pub.publish("CAUGHT " + userinput[0].readline().rstrip())
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
