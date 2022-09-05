#!usr/bin/env python
import rospy
from simple_pid import PID

from std_msgs.msg import Int64
from std_msgs.msg import Int16
from dynamic_reconfigure.server import Server
from hoverboard_control.cfg import PIDConfig

class Hoverboard_Control:
    started = False
    velocityValue = 0.0
    PWM_Output = 0

    def __init__(self):
        # Publishers
        self.pub_pwm = rospy.Publisher("/PWM_Value", Int16, queue_size = 100)
        self.pub_velocity = rospy.Publisher("/CurrentVelocity", Int64, queue_size = 100)
    def subscribeVelocityValue(self):
        rospy.Subscriber('VelocityValue', Int64, self.velocityCallback)
        timer = rospy.Timer(rospy.Duration(0.01), self.timerCallback)
        rospy.spin()
        timer.shutdown()
    def velocityCallback(self, data):
        print("Velocity Value Received", self.velocityValue)
        self.velocityValue = data.data
        if (self.started == False):
            self.started = True
    def timerCallback(self, event):
        if (self.started == True):
            desiredVelocity = rospy.get_param('/hoverboard_control/RPM')
            
            Kp = rospy.get_param('/hoverboard_control/Kp')
            Ki = rospy.get_param('/hoverboard_control/Ki')
            Kd = rospy.get_param('/hoverboard_control/Kd')

            previousWheelVelocity = self.velocityValue
            pid = PID(Kp, Ki, Kd, setpoint = desiredVelocity,  auto_mode = True)
            pid.output_limits = (-255, 255)
            pid.sample_time = 0.001
            self.PWM_Output = pid(previousWheelVelocity)

            self.pub_pwm.publish(self.PWM_Output)
            print("Desired Velocity", desiredVelocity)
            print("Kp = ", Kp, "Ki = ", Ki, "Kd = ", Kd)
            print("Publishing PWM values", self.PWM_Output)
            print("Current Wheel Velocity", self.velocityValue)

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {RPM}, {Kp}, {Ki},\
            {Kd}, {size}""".format(**config))

    return config   

if __name__ == '__main__':
    print("Running")
    rospy.init_node("hoverboard_control", anonymous=False)
    hoverboard = Hoverboard_Control()
    srv = Server(PIDConfig, callback)
    hoverboard.subscribeVelocityValue()