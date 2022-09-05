import RRT_2D
import cv2
import numpy as np
mapPath = "/home/leanhchien/catkin_ws/src/ros_autonomous_slam/media/map.png"
mapPath1 = "/home/leanhchien/Navigation_algorithm/RRT_star_algorithm/RRT_map/world1.png"
print ('Loading map... with file \'', mapPath,'\'')
image = cv2.imread(mapPath)
image = np.array(image)

print ('Map is', len(image[0]), 'x', len(image))
start, goal = ([65.0, 248.0], [326.0, 279.0])
print(start,goal)
rrt = RRT_2D.rrt_2D(image, start, goal)
rrt.RRT()
for i in range(len(rrt.graph)):
    cv2.circle(image, (int(rrt.graph[i][0][0]), int(rrt.graph[i][0][1])), 1, (0, 128, 128), thickness = 2, lineType = 8)
if rrt.path != None:
    cv2.circle(image, (int(start[0]), int(start[1])), 2, (255, 0, 0), thickness = 2, lineType = 8)
    cv2.circle(image, (int(goal[0]), int(goal[1])), 2, (0, 255, 0), thickness = 2, lineType = 8)
    for i in range(len(rrt.path)-1):
        cv2.line(image, tuple(rrt.path[i]), tuple(rrt.path[i+1]), (0, 0, 255),thickness = 2, lineType = 8 )
    cv2.imshow("Result Image", image)
    cv2.waitKey()

        
