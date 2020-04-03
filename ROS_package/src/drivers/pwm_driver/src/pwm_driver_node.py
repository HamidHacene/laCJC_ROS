#!/usr/bin/env python
import rospy
import maestro
from std_msgs.msg import Float32

"""
Pololu's Maestro Windows installer sets up the Maestro Control Center, used to configure, test and program the controller. 
Be sure the Maestro is configured for "USB Dual Port" serial mode, which is not the default.
https://github.com/FRC4564/Maestro/blob/master/README.md

------------
Channels
0 : throttle
1 : steering
"""

def callbackThrottle(data):
    """
    Data between -100 and 100
    """
    global servo

    # target has to be between 4000 and 8000
    target = 10*(600 + data*2)

    servo.setTarget(0,target)

def callbackSteering(data):
    """
    Data between -100 and 100
    """
    global servo

    # target has to be between 4000 and 8000
    target = 10*(600 + data*2)

    servo.setTarget(1,target)


def listener():
    global servo

    rospy.init_node('pwm_driver', anonymous=True)
    port = rospy.get_param("port")
    servo = maestro.Controller(port)
    servo.setAccel(0,4)

    rospy.Subscriber("throttle_cmd", Float32, callbackThrottle)
    rospy.Subscriber("steering_cmd", Float32, callbackSteering)

    rospy.spin()


if __name__ == '__main__':
    port = "/dev/ttyACM0"
    servo = 0
    

    listener()
