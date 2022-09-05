import cv2
import numpy as np
import random, math
import sys 
class rrt_2D:
    def __init__(self, image, start, goal, MIN_NUM_VERT = 20, MAX_NUM_VERT = 1500, STEP_DISTANCE = 20, SEED = None):
        self.image = image
        self.grayImage = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        ( _ , self.binaryImage) = cv2.threshold(self.grayImage, 250, 255, cv2.THRESH_BINARY)
        self.height, self.width, _ = self.image.shape
        self.start = start
        self.goal = goal
        self.MIN_NUM_VERT = MIN_NUM_VERT
        self.MAX_NUM_VERT = MAX_NUM_VERT
        self.step_distance = STEP_DISTANCE
        self.seed = SEED
        self.points = [self.start]
        self.graph = [(self.start, [])]
        self.path = []
        
        # phase two values (points 5 step distances around the goal point)
        self.minX =  max(self.goal[0] - 5 * self.step_distance, 0)
        self.maxX = min(self.goal[0] + 5 * self.step_distance, self.width - 1)
        self.minY = max(self.goal[1] - 5 * self.step_distance, 0)
        self.maxY = min(self.goal[1] + 5 * self.step_distance, self.height- 1)
    def RRT(self):
        hundreds =  100
        random.seed(self.seed)
        occupied = True
        phaseTwo = False

        i = 0
        while (self.goal not in self.points) and (len(self.points) < self.MAX_NUM_VERT):
            if (len(self.points) % hundreds) == 0:
                hundreds += 100
            while (occupied):
                if phaseTwo and (random.random() > 0.8):
                    point = [random.randint(self.minX, self.maxX), random.randint(self.minY, self.maxY)]
                else:
                    point = [random.randint(0, self.width-1), random.randint(0, self.height-1)]

                if self.binaryImage[point[1], point[0]] == 255:
                    occupied = False
            occupied = True

            nearest = self.findNearestPoint(point)
            newPoints = self.connectPoints(point, nearest)
            self.addToGraph(newPoints, point)
            newPoints.pop(0)# The first element is already in the points list
            self.points.extend(newPoints)

            i += 1
            if len(self.points) >= self.MIN_NUM_VERT:
                if not phaseTwo:
                    print('Phase Two')
            phaseTwo = True

            if phaseTwo:
                nearest = self.findNearestPoint(self.goal)
                newPoints = self.connectPoints(self.goal, nearest)
                self.addToGraph(newPoints, self.goal)
                newPoints.pop(0)
                self.points.extend(newPoints)
            
            if self.goal in self.points:
                self.path = self.searchPath(self.graph, self.start, [self.start])
                for i in range(len(self.path)):
                    self.path[i][0] = int(round(self.path[i][0],0))
                    self.path[i][1] = int(round(self.path[i][1],0))
            else:
                self.path = None
    def searchPath(self, graph, point, path):
        for i in graph:
            if point == i[0]:
                p = i

        if p[0] == graph[-1][0]:
            return path

        for link in p[1]:
            path.append(link)
            finalPath = self.searchPath(graph, link, path)

            if finalPath != None:
                return finalPath
            else:
                path.pop()

    def addToGraph(self, newPoints, point):
        if len(newPoints) > 1: # If there is anything to add to the graph
            for p in range(len(newPoints) - 1):
                nearest = [ nearest for nearest in self.graph if (nearest[0] == [ newPoints[p][0], newPoints[p][1] ]) ]
                nearest[0][1].append(newPoints[p + 1])
                self.graph.append((newPoints[p + 1], []))

    def connectPoints(self, a, b):
        newPoints = []
        newPoints.append([b[0], b[1]])
        step = [ (a[0] - b[0]) / float(self.step_distance), (a[1] - b[1]) / float(self.step_distance) ]

        # Set small steps to check for walls
        pointsNeeded = int(math.floor(max(math.fabs(step[0]), math.fabs(step[1]))))
        if math.fabs(step[0]) > math.fabs(step[1]):
            if step[0] >= 0:
                step = [ 1, step[1] / math.fabs(step[0]) ]
            else:
                step = [ -1, step[1] / math.fabs(step[0]) ]

        else:
            if step[1] >= 0:
                step = [ step[0] / math.fabs(step[1]), 1 ]
            else:
                step = [ step[0] / math.fabs(step[1]), -1 ]

        blocked = False
        for i in range(pointsNeeded+1): # Creates points between graph and solitary point
            for j in range(self.step_distance): # Check if there are walls between points
                coordX = round(newPoints[i][0] + step[0] * j)
                coordY = round(newPoints[i][1] + step[1] * j)

                if coordX == a[0] and coordY == a[1]:
                    break
                if coordY >= self.height or coordX >= self.width:
                    break
                if self.binaryImage[int(coordY), int(coordX)] == 0:
                    blocked = True
                if blocked:
                    break

            if blocked:
                break
            if not (coordX == a[0] and coordY == a[1]):
                newPoints.append([ newPoints[i][0]+(step[0]*self.step_distance), newPoints[i][1]+(step[1]*self.step_distance) ])

        if not blocked:
            newPoints.append([ a[0], a[1] ])
        return newPoints
    
    def findNearestPoint(self, point):
        best = (sys.maxsize, sys.maxsize, sys.maxsize)
        for p in self.points:
            if p == point:
                continue
            dist = math.sqrt((p[0] - point[0]) ** 2 + (p[1] - point[1]) ** 2)
            if dist < best[2]:
                best = (p[0], p[1], dist)
        return (best[0], best[1])

def main():
    mapPath = "/home/leanhchien/catkin_ws/src/ros_autonomous_slam/media/map.png"
    mapPath1 = "/home/leanhchien/Navigation_algorithm/RRT_star_algorithm/RRT_map/world1.png"
    print ('Loading map... with file \'', mapPath,'\'')
    image = cv2.imread(mapPath)
    image = np.array(image)

    print ('Map is', len(image[0]), 'x', len(image))
    start, goal = ([65.0, 248.0], [326.0, 279.0])
    print(start,goal)
    rrt = rrt_2D(image, start, goal)
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
# if __name__ == '__main__':
#     main() 