#!/usr/bin/env python
import rospy

from dynamic_reconfigure.server import Server
from hoverboard_control.cfg import PIDConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {RPM}, {Kp}, {Ki},\
            {Kd}, {size}""".format(**config))

    return config

if __name__ == "__main__":
    rospy.init_node("hoverboard_control", anonymous = False)
    srv = Server(PIDConfig, callback)

    rospy.spin()