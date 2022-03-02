#!/usr/bin/env python3
import rospy
import math
from rospy.client import get_param
from rospy.core import is_shutdown
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
import numpy

currentx = 0
currenty = 0
currenttheta = 0

def path_to_map(pathlist):
    maplist = []

    for item in pathlist:
        x = round((item[0]-8.3), 2)
        y = round((9.3- item[1]),2)
        maplist.append([x,y])
    
    return maplist

def push(heap, val):
    cur = len(heap) 
    heap.append(val)
    while cur > 0:
        parent = (cur - 1) // 2
        if heap[parent] <= heap[cur]: break
        heap[cur], heap[parent] = heap[parent], heap[cur]
        cur = parent
        pass
    
def pop(heap):
    ret = heap[0]
    last = heap.pop()
    size = len(heap)
    if size == 0: return ret
    heap[0] = last
    cur = 0
    while True:
        ch1 = 2 * cur + 1
        if ch1 >= size: return ret
        ch2 = ch1 + 1
        child = ch2 if ch2 < size and heap[ch2] < heap[ch1] else ch1
        if heap[cur] <= heap[child]: return ret
        heap[child], heap[cur] = heap[cur], heap[child]
        cur = child
        pass


def findpath(start, goal):
    
    grid = numpy.array([
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]])
    
    
    opened = [] # set of nodes to be evaluated
    closed = set() # set of nodes already evalated
    

    g_score = dict() # key--> node: value -->corresponding g score  
    f_score = dict() # key--> node: value --> corresponding f score
    origin = dict()  # to get the origin of the node 


    g_score[start] =0  
    f_score[start]= g_score[start]+h_score(start, goal)
    
    #neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    
    neighbors = []
    
    for i in range (-1, 2):
        for j in range(-1, 2):
            neighbors.append([i,j])

    neighbors.remove([0,0])
 
    push(opened, (f_score[start] , start)) #pushing fscore and respecive node
    
    #print("entering while looop")
    while opened:
        
        current = pop(opened)
        
        current_f_score = current[0]
        current_node = current[1]
        
        if current_node == goal:
            
            path = []
            
            while current_node in origin.keys():
                path.append(current_node)
                current_node = origin[current_node]

            #path.reverse()
            
            return path[::-1]
        
        closed.add(current_node)
        
        for i, j in neighbors:
            
            neighbor_x = current_node[0] + i
            neighbor_y = current_node[1] + j
            
            neighbor_node = neighbor_x , neighbor_y
            
            updated_g_score = g_score[current_node] + h_score(current_node, neighbor_node)
            
            
            if 0 <= neighbor_x < grid.shape[1]:
                if 0<=neighbor_y < grid.shape[0]:
                    
                    if grid[neighbor_y][neighbor_x] ==1:
                        continue
                        
                else:
                    continue
            else:
                continue
                
            
            if neighbor_node in closed and updated_g_score >= g_score.get(neighbor_node, 0):
                continue
                
            nodelist = []
            
            for node in opened:
                nodelist.append(node[1])
                
            if updated_g_score < g_score.get(neighbor_node,0) or neighbor_node not in nodelist:
                
                origin[neighbor_node] = current_node
                
                g_score[neighbor_node] = updated_g_score
                f_score[neighbor_node] = updated_g_score + h_score(neighbor_node, goal)
                
                push(opened,(f_score[neighbor_node], neighbor_node))
                
                    
    return False
    
def h_score(a, b):
    
    x = b[0]-a[0]
    y = b[1]-a[1]
    
    distance = math.sqrt((x**2) + (y**2))
    
    return distance


def odom_callback(message):

    global currentx
    global currenty
    global currenttheta

    currentx = message.pose.pose.position.x
    currenty = message.pose.pose.position.y

    rot_q = message.pose.pose.orientation

    (R, P, currenttheta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
   
if __name__ == '__main__':

    rospy.init_node('astar')
    rate = rospy.Rate(5)
    odom_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, odom_callback)
    pub = rospy.Publisher('/cmd_vel' , Twist, queue_size= 1)


    goal = Point()
    #path = [[-6.2, -2.8], [-5.2, -2.8], [-4.2, -2.8], [-3.2, -3.8], [-2.2, -4.8], [-1.2, -4.8], [-0.19999999999999996, -3.8], [0.8, -2.8], [0.8, -1.8], [0.8, -0.8], [0.8, 0.19999999999999996], [1.8, 1.2], [2.8, 2.2], [3.8, 3.2], [3.8, 4.2], [3.8, 5.2], [3.8, 6.2], [3.8, 7.2], [4.8, 8.2]]
    #fullpath = [(2, 12), (3, 12), (4, 12), (5, 13), (6, 14), (7, 14), (8, 13), (9, 12), (9, 11), (9, 10), (9, 9), (10, 8), (11, 7), (12, 6), (12, 5), (12, 4), (12, 3), (12, 2), (13, 1)]
    #halfpath = [(2, 12), (3, 12), (4, 12), (5, 13), (6, 14), (7, 14), (8, 13)]
    

    calculated_path =  findpath((1,12),(13,1))
    mapcoordinates = path_to_map(calculated_path)

 
    if rospy.has_param("goalx") and rospy.has_param("goaly"):
        goal_x = get_param("goalx")
        goal_y = get_param("goaly")

        goal.x = round(goal_x) + 9
        goal.y = 10-round(goal_y)
        #print("Goal coordinates are given")
        #print("Value of goalx is: ", goal.x)
        #print("Value of goaly is: ", goal.y)
        
        if goal_x != 4.5 and goal_y != 9.0:
            newpath = findpath((1,12),(goal.x, goal.y))
            mapcoordinates = path_to_map(newpath)
        


    print("Maplist: " , mapcoordinates)
    while not rospy.is_shutdown():

        
        #goal = Point()
        speed = Twist()
        count = 0
        #print("in first while")

        #print(rospy.has_param("goalx"))
        #if rospy.has_param("goalx"):
        #    print(get_param("goalx"))

        for currentgoal in mapcoordinates:
            count +=1
            print("Current goal:", currentgoal)

            goalreachedflag = False
            goal.x , goal.y = currentgoal

            while not  goalreachedflag:

                errorx = goal.x - currentx
                errory = goal.y - currenty

                angletorotate = math.atan2(errory,errorx)
                angleerror = angletorotate - currenttheta    
                if abs(angletorotate-currenttheta) > math.radians(5):
                    
                    if angleerror < 0:
                        speed.linear.x = 0.0
                        speed.angular.z = -0.5
                    else:
                        speed.linear.x = 0.0
                        speed.angular.z = 0.5


                else:
                    speed.linear.x = 1.0
                    speed.angular.z= 0.0

                if abs(errorx) < 0.2 and abs(errory) < 0.2:
                    speed.linear.x = 0
                    speed.angular.z = 0
                    goalreachedflag = True

                pub.publish(speed)


            if count == len(mapcoordinates):
                print("Pochla re baba")
                break
        break  

    rospy.spin()
