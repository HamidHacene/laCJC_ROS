#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def callbackThrottle(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def callbackSteering(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    rospy.init_node('pwm_driver', anonymous=True)

    rospy.Subscriber("throttle_cmd", Float32, callbackThrottle)
    rospy.Subscriber("steering_cmd", Float32, callbackSteering)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()