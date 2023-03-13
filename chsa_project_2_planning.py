#First we import the Libraries
import time
import cv2
import heapq as hq
import copy
import numpy as np


#Now we create function for the movement of robot 
 

def function_move_down(current_node_location,map):

    position = copy.deepcopy(current_node_location)

    if(position[1]+1 < map.shape[0]) and (map[position[1]+1][position[0]][0]<255):
        
        position[1] = position[1] + 1 
        
        return True,tuple(position)
    else:
        
        return False,tuple(position)

def function_move_up(current_node_location,map):

    position = copy.deepcopy(current_node_location)
    
    if(position[1]-1 > 0) and (map[position[1]-1][position[0]][0]<255):
        
        position[1] = position[1] - 1 
        
        return True,tuple(position)
    else:
        
        return False,tuple(position)


def function_move_right(current_node_location,map):
   
    position = copy.deepcopy(current_node_location)

    if(position[0]+1 < map.shape[1]) and (map[position[1]][position[0]+1][0]<255):
        
        position[0] = position[0] + 1 
        return True,tuple(position)
    else:
        
        return False,tuple(position)

def function_move_left(current_node_location,map):

    position = copy.deepcopy(current_node_location)

    if(position[0]-1 > 0) and (map[position[1]][position[0]-1][0]<255):
        
        position[0] = position[0] - 1 
        return True,tuple(position)
    else:
        
        return False,tuple(position)

def function_move_diagonal_down_right(current_node_location,map):

    position = copy.deepcopy(current_node_location)
    if(position[1]+1 < map.shape[0]) and (position[0]+1 <map.shape[1]) and (map[position[1]+1][position[0]+1][0]<255):
        
        position[1] = position[1] + 1
        
        position[0] = position[0] + 1 
        
        return True,tuple(position)
    
    else:
        
        return False,tuple(position)

def function_move_diagonal_up_right(current_node_location,map):
 
    position = copy.deepcopy(current_node_location)
 
    if(position[1]-1 > 0) and (position[0]+1 <map.shape[1]) and (map[position[1]-1][position[0]+1][0]<255):
    
        position[1] = position[1] - 1
    
        position[0] = position[0] + 1 
    
        return True,tuple(position)
    
    else:
    
        return False,tuple(position)



def function_move_diagonal_up_left(current_node_location,map):

    position = copy.deepcopy(current_node_location)
    if(position[1]-1 > 0) and (position[0]-1 >0) and (map[position[1]-1][position[0]-1][0]<255):
    
        position[1] = position[1] - 1
    
        position[0] = position[0] - 1 
    
        return True,tuple(position)
    else:
    
        return False,tuple(position)

def function_move_diagonal_down_left(current_node_location,map):

    position = copy.deepcopy(current_node_location)
    
    if(position[1]+1 < map.shape[0]) and (position[0]-1 >0) and (map[position[1]+1][position[0]-1][0]<255):
    
        position[1] = position[1] + 1
    
        position[0] = position[0] - 1 
    
        return True,tuple(position)
    
    else:
    
        return False,tuple(position)
    
    


#Now we create a function for backtracking

def function_backtracking(point_starting, point_goal, list_close, map):
    
    # getting output as a video in the same folder
    
    out = cv2.VideoWriter('Code_running_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 1000, (map.shape[1], map.shape[0]))
    path_stack = []
    
    for key in list_close.keys():
        
        map[key[1]][key[0]] = [255, 255, 255]
        
        cv2.imshow("Project 2 Chandan Salil", map)
        cv2.waitKey(1)
        out.write(map)
        
    parent_node_current_location = list_close[tuple(point_goal)]
    path_stack.append(point_goal)
    
    while parent_node_current_location != point_starting:
        path_stack.append(parent_node_current_location)
        parent_node_current_location = list_close[tuple(parent_node_current_location)]
    
    #Defining Starting 
    cv2.circle(map, tuple(point_starting), 3, (0, 255, 0), -1)
    
    #Defining end Goal
    cv2.circle(map, tuple(point_goal), 3, (0, 0, 255), -1)
    path_stack.append(point_starting)
    
    while len(path_stack) > 0:
        current_loaction_path = path_stack.pop()
        map[current_loaction_path[1]][current_loaction_path[0]] = [19, 209, 158]
        out.write(map)
        cv2.imshow("Project 2 Chandan Salil", map)
    out.release() 



#First we initialize a blank map 

map = np.ones((250,600,3),dtype="uint8") 

#First Rectangle
points_first_rectangle =[np.array([[100,0],[100,100],[150,100],[150,0],[100,0]])]
cv2.fillPoly(map, points_first_rectangle, color =(0,255, 0))
cv2.polylines(map,points_first_rectangle,True,color=(0,0,255),thickness=5) 

#Second Rectangle
points_second_rectangle = [np.array([[100,150],[100,250],[150,250],[150,150],[100,150]])]
cv2.fillPoly(map, points_second_rectangle, color=(0,255, 0))
cv2.polylines(map,points_second_rectangle,True,color=(0,0,255),thickness=5)

# Define the Hexagon
points_hexagon = [np.array([[235.04, 87.5], [235.05, 162.5], [300, 200], [364.95, 162.5], [364.95, 87.5], [300, 50], [235.04, 87.5]],np.int32)]
cv2.fillPoly(map, points_hexagon, color=(0,255, 0))
cv2.polylines(map,points_hexagon,True,color=(0,0,255),thickness=5)

#Define the Triangle
points_triangle = [np.array([[460,25],[510,125], [460,225], [460,25]])]
cv2.fillPoly(map , points_triangle, color=(0,255, 0)) 
cv2.polylines(map,points_triangle, True ,color=(0,0,255),thickness=5)



 

point_starting = []
point_goal = []

def user_input(type_of_point):
    while True:
        x = input(f"Please enter X Coordinate of {type_of_point} Point: ")
        y = input(f"Please enter Y Coordinate of {type_of_point} Point: ")
        if not (0 <= int(x) < map.shape[1] and 0 <= int(y) < map.shape[0]):
            print("Kindly Enter valid integer coordinates")
            
            #check if overlaps the obstacle 
        elif map[map.shape[0] - int(y) - 1][int(x)][0] == 255:
            print(f"ERROR !!! {type_of_point.lower()} overlaps with obstacle")
        else:
            return [int(x), int(y)]

point_starting = user_input("Starting")
point_goal = user_input("Ending")

point_goal[1] = map.shape[0]-1 - point_goal[1]
point_starting[1] = map.shape[0]-1 - point_starting[1]


time_strt = time.time()

#open list
list_open = []
#close list as dictionary
list_close = {}

# Implementation  

function_back_tracking_flag = False

hq.heapify(list_open)

hq.heappush(list_open,[0,point_starting,point_starting])

while(len(list_open)>0):
    current_node_location = hq.heappop(list_open)
    
    list_close[(current_node_location[2][0],current_node_location[2][1])] = current_node_location[1]
    #initailize current cost
    current_node_location_cost = current_node_location[0]
    
    if list(current_node_location[2]) == point_goal:
        function_back_tracking_flag = True
        print("Back Track")
        break
    
    for motion in [(function_move_up, 1), (function_move_diagonal_up_right, 1.4), (function_move_right, 1), (function_move_diagonal_down_right, 1.4)]:
        flag, position = motion[0](current_node_location[2], map)
        
        if flag and position not in list_close:
            temp = False
        
            for i in range(len(list_open)):
        
                if list_open[i][2] == list(position):
        
                    temp = True
        
                    if (current_node_location_cost + motion[1]) < list_open[i][0]:
                      
                        list_open[i][0] = current_node_location_cost + motion[1]
                      
                        list_open[i][1] = current_node_location[2]
                      
                        hq.heapify(list_open)
                    break
                
            if not temp:
                
                hq.heappush(list_open, [current_node_location_cost + motion[1], current_node_location[2], list(position)])
                
                hq.heapify(list_open)
                
if(function_back_tracking_flag):
    
    function_backtracking(point_starting,point_goal,list_close,map)

else:
    print("ERROR!!! The given path cannot be found")

goal_reached_time = time.time() 

cv2.waitKey(0) 
#end all windows
cv2.destroyAllWindows() 

print("The total time taken is : ",goal_reached_time-time_strt) 