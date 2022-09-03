#!usr/bin/env python
import rospy
from simple_pid import PID

from std_msgs.msg import Int64
from std_msgs.msg import Int16

class Hoverboard_Control:
    Kp = 0.0
    Ki = 0.0
    Kd = 0.0
    desired_Velocity = 0.0
    started = False
    started1 = False
    velocityValue = 0.0
    PWM_Output = 0

    pub_pwm = rospy.Publisher("/PWM_Value", Int16, queue_size = 100)
    pub_velocity = rospy.Publisher("/CurrentVelocity", Int64, queue_size = 100)
    def __init__(self, kP, kI, kD):
        self.Kp = kP
        self.Ki = kI
        self.Kd = kD
    def subscribeVelocityValue(self):
        rospy.init_node("hoverboard_control", anonymous=True)
        rospy.Subscriber('VelocityValue', Int64, self.velocityCallback)
        rospy.Subscriber('DesiredVelocity', Int64, self.desiredVelocityCallback)
        timer = rospy.Timer(rospy.Duration(0.01), self.timerCallback)
        rospy.spin()
        timer.shutdown()
    def velocityCallback(self, data):
        print("Velocity Value Received", self.velocityValue);
        self.velocityValue = data.data
        if (not self.started):
            self.started = True
    def desiredVelocityCallback(self, data):
        self.desired_Velocity = data.data
        print("Desired_Velocity", self.desired_Velocity)
        if (not self.started1):
            self.started1 = True
    def timerCallback(self, event):
        if (self.started1):
            if (self.started):
                previousWheelVelocity = self.velocityValue
                pid = PID(self.Kp, self.Ki, self.Kd, setpoint = self.desired_Velocity,  auto_mode = True)
                pid.output_limits = (-255, 255)
                pid.sample_time = 0.001
                self.PWM_Output = pid(previousWheelVelocity)

                self.pub_pwm.publish(self.PWM_Output)
                print("Publishing PWM values", self.PWM_Output)
                print("Current Wheel Velocity", self.velocityValue)
    

if __name__ == '__main__':
    print("Running")
    hoverboard = Hoverboard_Control(100, 20, 10)
    hoverboard.subscribeVelocityValue()