#!/usr/bin/env python3
#importing the required libraries 
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import heapq
from math import dist
import matplotlib.patches as patches
import rospy
from geometry_msgs.msg import Twist
import random

plt.ion()
#first we define the node class which is used for representing nodes generated in the graph 

class Node:

    def __init__(self, x, y, parent, theta_crnt, theta_chng, vel_left, vel_right, cost_to_come, cost_to_go, cost_total):
        
        
        self.x = x
        self.y = y
        self.parent = parent
        self.theta_crnt = theta_crnt
        self.theta_chng = theta_chng
        self.vel_left = vel_left
        self.vel_right = vel_right
        self.cost_to_come = cost_to_come
        self.cost_to_go = cost_to_go
        self.cost_total = cost_total
        
    def __lt__(self, other):

        return self.cost_total < other.cost_total


# Now we make a function to make dubins curve 

def curve_plot(x_start, y_start, angle_start, vel_left, vel_right,c, plot, lst_node, lst_pth):

    t = 0
# Radius of the wheel 
    r = 0.033
#Axle Length    
    L = 0.160
    dt = 0.8
    cost = 0
    
    # initial conditions 
    
    x_goal = x_start
    y_goal= y_start
    
    # angle conversion
    angle_goal = 3.14 * angle_start / 180

    while t < 1:
        
        t = t + dt
        #Using x start 
        X_start = x_goal
        #Using y start 
        Y_start = y_goal
        #For making dubin curve
        #x cord of goal
        x_goal += r*0.5 * (vel_left + vel_right) * math.cos(angle_goal) * dt
        #y cord of goal
        y_goal += r*0.5 * (vel_left + vel_right) * math.sin(angle_goal) * dt
        #Goal orientation
        angle_goal += (r / L) * (vel_right - vel_left) * dt
        
        #Verifying the moves 
        
        if  move_checker(x_goal,  y_goal, r, c):
            
            if plot == 0:
                
                cost_to_go = dist((X_start, Y_start), (x_goal,  y_goal))
                #Final cost = cost to come + cost to go 
                cost = cost + cost_to_go
                # appending the values to nodes list 
                lst_node.append((x_goal, y_goal))
                # appending the values to nodes list 
                lst_pth.append((X_start, Y_start))
                
            #Plotting once goal is reached     
            
            if plot == 1:
            #Plotting red on the map 
                plt.plot([X_start, x_goal], [Y_start, y_goal], color="red")
        else:
            # If not found
            return None
        # converting the angle
    angle_goal = 180 * (angle_goal) / 3.14
    return [x_goal, y_goal, angle_goal, cost, lst_node, lst_pth]


# now we assign unique id to each node 
def u_id(node):
    #Generating u_id by scaling 
    u_id = 1000*node.x + 111*node.y
     
    return u_id

# A star Implementation 
def function_a_star(node_initial, node_goal, rpm1, rpm2, radius, robot_clearance):

    # Condition to check if goal is reached 
    if verify_goal(node_initial, node_goal):
        return 1,None,None
    
    
    node_initial = node_initial
    
    node_initial_id = u_id(node_initial)
    node_goal = node_goal

    lst_node = []  # List for Explored nodes
    
    lst_pth = []  # List for final path from start node to goal node

    closed_node = {}  # Dictionary for closed nodes
    
    robot_open_node = {}  # Dictionary for open nodes
    
    robot_open_node[node_initial_id] = node_initial   # Adding the initial node to the open node dictionary

    priority_list = []  # Priority queue for storing nodes according to the total cost
    
    # Possible robot movements
    
    robot_moves = [[rpm1, 0], [0, rpm1], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]

    # Pushing the start node in priority queue along with total cost
    
    heapq.heappush(priority_list, [node_initial.cost_total, node_initial])

    while (len(priority_list) != 0):

        # Popping the minimum cost node from priority queue
        crnt_nodes = (heapq.heappop(priority_list))[1]
    
        u_id_crnt = u_id(crnt_nodes)

        # Condition to check if popped node is goal  node
    
        if verify_goal(crnt_nodes,node_goal):
            node_goal.parent =  crnt_nodes.parent
            
            node_goal.cost_total =  crnt_nodes.cost_total
            print("Success! The Goal Node has been found")
            #return 1 to plot graph 
            return 1,lst_node,lst_pth
        
        # Adding popped node to Closed node dict
        if u_id_crnt in closed_node:  
            continue
        else:
            closed_node[u_id_crnt] = crnt_nodes
        
        del robot_open_node[u_id_crnt]
        
        # For loop all possible movements of robot
        for robot_move in robot_moves:
            movement = curve_plot(crnt_nodes.x, crnt_nodes.y, crnt_nodes.theta_crnt, robot_move[0], robot_move[1],
                            robot_clearance, 0, lst_node, lst_pth)
           
            # checking if movement is valid
            if (movement != None):
                angle = movement[2]
                
                # Rounding off coordinates and angle
                theta_max = 15
                x = (round(movement[0] * 10) / 10)
                y = (round(movement[1] * 10) / 10)
                theta = (round(angle / theta_max) * theta_max)
                
                # New orientation and cost to go to new node
                theta_crnt = crnt_nodes.theta_chng - theta
                cost_to_go = dist((x,y), (node_goal.x, node_goal.y))
                new_node = Node(x, y, crnt_nodes, theta, theta_crnt, robot_move[0], robot_move[1], crnt_nodes.cost_to_come+movement[3], cost_to_go, crnt_nodes.cost_to_come+movement[3]+cost_to_go)

                robot_id_new_node = u_id(new_node)
                
                # Checking if node is valid and not already visited
                if not move_checker(new_node.x, new_node.y, radius, robot_clearance):
                    continue
                elif robot_id_new_node in closed_node:
                    continue

                # Changing node info if already existing in open list
                if robot_id_new_node in robot_open_node:
                    if new_node.cost_total < robot_open_node[robot_id_new_node].cost_total:
                        
                        robot_open_node[robot_id_new_node].cost_total = new_node.cost_total
                        
                        robot_open_node[robot_id_new_node].parent = new_node

                # Adding to open list if not already present    
                else:
                    robot_open_node[robot_id_new_node] = new_node
                    heapq.heappush(priority_list, [ robot_open_node[robot_id_new_node].cost_total, robot_open_node[robot_id_new_node]])
            
    return 0, lst_node, lst_pth

# Checking for validity of node in obstacle space
def verify_obstacle_space(x, y, radius, robot_clearance):
    
    space = radius + robot_clearance # Buffer space

    rect_1 = ((np.square(x - 4)) + (np.square(y - 1.1)) <= np.square(0.5 + space)) # Rectangle obstacle 
    
    rect_2 = (x >= 1.5 - space) and (x <= 1.625 + space) and (y >= 0.75 - space)   # Rectangle obstacle
    circle = (x >= 2.5 - space) and (x <= 2.625 + space) and (y <= 1.25 + space)   #  Circular obstacle
   
    edge_1 = (x <= 0 + space)     
    edge_2 = (x >= 5.99 - space)
    edge_3 = (y <= 0 + space)
    edge_4 = (y >= 1.99 - space)

    if rect_1 or rect_2 or circle or edge_1 or edge_2 or edge_3 or edge_4:
        return True
    else:
        return False
    
# Checking whether movement is valid
def move_checker(x,y, r,c):
    
    if verify_obstacle_space(x, y, r, c):
        return False
    else:
        return True

# Checking if the goal has been reached 

def verify_goal(current, goal):

    dt = dist((current.x, current.y), (goal.x, goal.y))

    if dt < 0.15:
        return True
    else:
        return False
    

# Path Generation for the robot

def backtrack(node_goal):  

    x_path = []
    y_path = []
    theta_path = []
    x_path.append(node_goal.x)
    y_path.append(node_goal.y)
    theta_path.append(node_goal.theta_crnt)
    parent_node = node_goal.parent
    vel_left_list=[node_goal.vel_left]
    vel_right_list=[node_goal.vel_right]
    

    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        theta_path.append(parent_node.theta_crnt)
        vel_left_list.append(parent_node.vel_left)
        vel_right_list.append(parent_node.vel_right)
        parent_node = parent_node.parent
        
        
    x_path.reverse()
    y_path.reverse()
    theta_path.reverse()
    vel_left_list.reverse()
    vel_right_list.reverse()
    print(vel_left_list)
    print(vel_right_list)

    x = np.asarray(x_path)
    y = np.asarray(y_path)
    theta = np.array(theta_path)
    return x, y, theta, vel_left_list, vel_right_list

if __name__ == '__main__':

    width = 6
    height = 2
    robot_radius  = 0.038
    robot_clearance = int(input("Enter the clearance of robot: "))
    robot_clearance = robot_clearance/100
    robot_clearance = float(robot_clearance)

    RPM1 = int(input("Enter the Low RPM: "))
    RPM2 = int(input("Ente the High RPM: "))

    start_x = int(input("Enter starting X co-ordinate: "))
    start_y = int(input("Enter starting Y co-ordinate: "))
    start_x = start_x/100
    start_y = start_y/100
    start_x = float(start_x)
    start_y = float(start_y)

    start_theta = input("Enter the Orientation of the robot at start node: \n")
    start_theta = int(start_theta)

    #Rounding off the 
    number = int(start_theta)
    remainder = number % 30
    if remainder < 15:
      start_theta = number - remainder
    else:
      start_theta = number + (30 - remainder)
    
    goal_x = int(input("Enter goal X co-ordinates: "))
    goal_y = int(input("Enter goal Y co-ordinates: "))
    goal_x = goal_x/100
    goal_y = goal_y/100
    goal_x = float(goal_x)
    goal_y = float(goal_y)

    if not move_checker(start_x, start_y, robot_radius, robot_clearance):
        print("Start node is either invalid or in obstacle space")
        exit(-1)
        
    if not move_checker(goal_x, goal_y, robot_radius, robot_clearance):
        print("Goal node is either invalid or in obstacle space")
        exit(-1)
    
# Starting Timer to calculate runtime
    timer = time.time()

    cost_to_go = dist((start_x,start_y), (goal_x, goal_y))
    cost_total =  cost_to_go
    node_initial = Node(start_x, start_y,-1,start_theta,0,0,0,0,cost_to_go,cost_total)
    node_goal = Node(goal_x, goal_y, -1,0,0,0,0,cost_to_go,0,cost_total)

    flag, lst_node, lst_pth = function_a_star(node_initial, node_goal,RPM1,RPM2,robot_radius,robot_clearance)
                    
    x_path, y_path, theta_path, vel_left_list, vel_right_list = backtrack(node_goal)
    figure, axes = plt.subplots()
    axes.set(xlim=(0,6), ylim=(0,2))
    

    rect_1 = plt.Circle((4, 1.1), 0.5, fill = 'True', color ='blue')
    rect_2 = patches.Rectangle((1.5, 0.75), 0.15, 1.25, color ='blue')
    circle = patches.Rectangle((2.5, 0), 0.15, 1.25, color ='blue')


    axes.set_aspect('equal')
    axes.add_artist(rect_1)
    axes.add_artist(rect_2)
    axes.add_patch(circle)

    plt.plot(node_initial.x, node_initial.y, "Dw")
    plt.plot(node_goal.x, node_goal.y, "Dg")

    l = 0
    for l in range(len(lst_node)):
        plt.plot([lst_pth[l][0], lst_node[l][0]], [lst_pth[l][1], lst_node[l][1]], color="green")
        l+=1
        #plt.pause(0.00000000000000000000000000001)

    plt.plot(x_path,y_path, ':r')
    timer_stop = time.time()
    
    Count_time = timer_stop - timer
    print("The Total Runtime is:  ", Count_time)

    #plt.show()
    
    #plt.pause(100)
    plt.close('all')
# Initialize the node
rospy.init_node('turtlebot3_publisher', anonymous=True)

# Create a publisher object
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Set the publishing rate at 10 hz
rate = rospy.Rate(10)

# Set the start time
start_time = rospy.Time.now()

#Linear Velocities
linear_velocities = []
#Angular Velocities
angular_velocities = []
#radius of wheel
r=0.033
#Length of the vehicle
L= 0.160
#using for loop 
for i in range(len(vel_left_list)):
    #using the formula 
    vel_left_list[i] = vel_left_list[i] * 0.10472 *10
    vel_right_list[i] = vel_right_list[i] * 0.10472 *10
    #calculating linear velocity 
    linear_velocity = r/2 * (vel_left_list[i] + vel_right_list[i])
    linear_velocities.append(linear_velocity)

for i in range(len(vel_left_list)):
    #using the formula
    vel_left_list[i] = vel_left_list[i] * 0.10472 *10
    vel_right_list[i] = vel_right_list[i] * 0.10472 *10
    #calculating angular velocity
    angular_velocity = -(r/L* (vel_left_list[i] - vel_right_list[i]))
    angular_velocities.append(angular_velocity)


# Publishing the velocity
 
for i in range(len(linear_velocities)):
    linear_vel = linear_velocities[i]
    angular_vel = angular_velocities[i]

    # Define the velocity message
    vel_msg = Twist()
    vel_msg.linear.x = linear_vel
    vel_msg.angular.z = angular_vel

    # Publish the velocity message for one second
    for a in vel_left_list:
        for b in vel_right_list:
            if a == b:
                while (rospy.Time.now() - start_time).to_sec() < 1.6 and not rospy.is_shutdown():
                    pub.publish(vel_msg)
                    rate.sleep()
            
            else:
                while (rospy.Time.now() - start_time).to_sec() < 0.185 and not rospy.is_shutdown(): 
                    pub.publish(vel_msg)
                    rate.sleep()

    start_time = rospy.Time.now()