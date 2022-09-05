#!/usr/bin/env python

from enum import auto
from os import stat
from RRT_2D import RRT_2D
import numpy as np
import cv2
import math
from nodes.autonomous_rrt import map_img, points_publisher
import rospy
import geometry_msgs.msg
from geometry_msgs import Point, Pose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, GridCells
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import actionlib
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class autonomousRRT():
    def __init__(self, goal_reached = False, robot_rotation = [0, 0, 0], robot_location = [0, 0, 0]):
        self.goal_reached = goal_reached
        self.robot_rotation = robot_rotation
        self.robot_location = robot_location
        self.robot_orientation = []
        self.current_map = np.zeros((384, 384))
    def convertPath(self,path,trans, t):
        '''
            Translate and Rotates the given set of coordinates
        '''
        resultPath = []
        for x in path:
            mat = [x[0], x[1]]
            mat = self.rot2d(mat, t)
            resultPath.append((mat[0] + trans[0], mat[1] + trans[1]))
        
        return resultPath
    @staticmethod
    def computeDistance(pos1, pos2, Type = 'd'):
        '''
            Compute distance between two positions
        '''
        x1 = pos1[0]
        y1 = pos1[1]
        x2 = pos2[0]
        y2 = pos2[1]
        d = ((x1-x2)**2) + ((y1-y2)**2)
        if Type == 'd':
            return math.sqrt(d)
        if Type == 'eu':
            return d
        if Type == 'manhattan':
            return abs(x1-x2)+abs(y1-y2)

    def rot2d(v, t):
        '''
        2D Rotation points
        '''
        x, y = v[0],v[1]
        xr = x*math.cos(t)-y*math.sin(t)
        yr = x*math.sin(t)+y*math.cos(t)
        return [xr,yr]

    def go_to_goal(self, goal):
        '''
            Functions to command robot in ROS stage to go to given goal wrt /odom frame
        ''' 
        d = self.computeDistance(self.robot_location, goal)
        theta = self.robot_rotation[2]
        kl = 1
        ka = 4
        vx = 0
        va = 0
        heading = math.atan2(goal[1]-self.robot_location[1],goal[0]-self.robot_location[0])
        err_theta = heading - theta
        if(d>0.01):
            vx = kl*abs(d)
            vx = 1
        if(abs(err_theta)>0.01):
            va = ka*(err_theta)

        vel_1 = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size=10) # Publish Command to robot_1
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = vx
        cmd.angular.z = va
        vel_1.publish(cmd)
    
    def Follow_path(self, path):
        '''
        Follows a given set of path - Reaches all the points in a list in consecutive order
    '''
        consecutivePath = path
        goal_point = consecutivePath[-1]
        print('Following Path -->',consecutivePath)
        for loc in consecutivePath:
            while(self.computeDistance(self.robot_location,loc)>0.1):
                # goal_location_marker(final_goal_location)
                # points_publisher(consecutivePath)
            
                self.go_to_goal([loc[0]/10,loc[1]/10])
                if(loc==goal_point):
                    self.goal_reached = True
    def callback_odom(self, msg):
        '''
        Obtains Odometer readings and assigns to global Variables
        '''
        location = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.robot_location = location
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation)
        rot = [roll, pitch, yaw]
        self.robot_rotation = rot
        self.robot_orientation = orientation

    def callback_map(self, msg):
        data = np.array(msg.data)
        map_width = msg.info.width
        map_height = msg.info.height
        self.current_map =  np.reshape(data,(map_height,map_width))

    def move_base_client(x, y):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.orientation.w = y

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
           return client.get_result()
    def map_img(self, arr):
        dis_map = np.ones((384,384))*255
        for i in range(arr.shape[0]):
            for j in range(arr.shape[1]):
                if arr[i][j]==-1:
                    dis_map[i][j] = 100
                if arr[i][j] == 100:
                    dis_map[i][j] = 0
        im = np.array(dis_map, dtype = np.uint8)
        return im[::-1]
    
    def points_publisher(self, points_list):
        marker_pub = rospy.Publisher('path_points', Marker,queue_size=1) # Publish Robot Position to RVIZ
        marker_data = Marker()
        marker_data.type = marker_data.POINTS
        marker_data.action = marker_data.ADD
        marker_data.header.frame_id = '/map'

        marker_data.scale.x = 0.1 # width
        marker_data.scale.y = 0.1 # Height

        marker_data.color.a = 1
        marker_data.color.r = 1
        marker_data.color.g = 0
        marker_data.color.b = 0

        for p in points_list:
            marker_data.points.append(Point(p[0],p[1],0))
        marker_pub.publish(marker_data)
        
def main():
    mapPath = "/home/leanhchien/catkin_ws/src/ros_autonomous_slam/media/map.png"
    rospy.init_node('RRT_Explorer')
    rate = rospy.Rate(10.0)
    
    autonomous_rrt = autonomousRRT()
    # Subscribe to /odom
    sub_odom = rospy.Subscriber('/odom', Odometry, autonomous_rrt.callback_odom) # Receive Odom readings
    sub_map = rospy.Subscriber('/map',OccupancyGrid, autonomous_rrt.callback_map) # Receive Map        
    start, goal = ([-2+int(384/2), 0.5+int(384/2)], [5+int(384/2), -0.5+int(384/2)])
    flag =  True
    while not rospy.is_shutdown():
        if flag: 
            rrt = RRT_2D(cv2.cvtColor(autonomous_rrt.map_img(autonomous_rrt.current_map), cv2.COLOR_GRAY2BGR)[::-1], start, goal)
            rrt.RRT()
            flag = False
        points_publisher(autonomous_rrt.convertPath(rrt.path, [-192,-192],0))
        cv2.imshow('Constructed Map', map_img(autonomous_rrt.current_map))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
