
'''

Team ID: e-YRC+ #1673
Author List: Heethesh Vhavle

Filename: eyrcplus_1673_python_code.py
Theme: eYRC-Plus
Functions:  get_perspective_image, crop_arena, get_marker_color, check_wall,
            arena_mapping, get_largest_contour, get_moments, get_direction,
            check_position, get_position, get_map, __init__, get_node,
            grid_map, get_adjacent, get_heuristic, update_values, path_list,
            a_star_algorithm, get_path, get_marker_pos, get_all_paths, get_turns,
            get_turns_list, get_min_path1, get_min_path2, get_min_path3,
            draw_grid, draw_firebird, draw_path, led_status, led_color,
            encode_north, encode_south, encode_west, encode_east, encode_path,
            simplify_encoding, get_reset_angle, rotation_feedback, xbee_communication
            
Global Variables: The list of global variables is mentioned below

'''


################################ IMPORT MODULES ################################

import numpy as np
import cv2
import serial
import heapq
import math
import time
import timeit
import warnings
from operator import itemgetter
from itertools import groupby

################################## START TIMER #################################

print '\n*Timer Started*'
start_time = timeit.default_timer()

############################### GLOBAL VARIABLES ###############################

# Height and width of image
height = 480
width = 540

# Obstacle Positions
obstacles = []
wall_positions = []
direction_dict = {1:'North',2:'West',3:'South',4:'East'}
green_special = False

# Lists containing all possible paths and their lengths
'''
s -> Start position (Corridor zone)
m -> Service zone
p -> Patient zone

sm -> Start to service zone
mm -> From one provision in the service zone to another
mp -> From service zone to patient zone tables
pp -> From one table in the patient zone to another
pm -> From tables in the patient zone to provisions in the service zone
'''
sm_len = []
mm_len = []
mp_len = []
pm_len = []
pp_len = []

sm_path = []
mm_path = []
mp_path = []
pm_path = []
pp_path = []

# Index values for all correct combinations from the above lists for 2 deliveries
index_2 = [[0,0,1,0,0],
           [0,0,1,1,1],
           [1,1,0,0,0],
           [1,1,0,1,1]]

# Index values for all correct combinations from the above lists for 3 deliveries
index_3 = [[0,0,0,1,0,0,0,1,1,2,2],
           [0,0,0,1,1,1,0,0,1,2,2],
           [0,0,1,2,0,0,1,2,1,1,1],
           [0,0,1,2,2,2,0,0,0,1,1],

           [1,1,0,0,0,0,0,1,1,2,2],
           [1,1,0,0,1,1,0,0,1,2,2],
           [1,1,1,2,1,1,1,2,0,0,0],
           [1,1,1,2,2,2,1,1,0,0,0],

           [2,2,0,0,0,0,1,2,1,1,1],
           [2,2,0,0,2,2,0,0,0,1,1],
           [2,2,1,1,1,1,1,2,0,0,0],
           [2,2,1,1,2,2,1,1,0,0,0]]

request_count = [0,0,0]
request_matched = 0
total_turns = 0

# List which stores the encoded commands to be sent to the robot
motion = []
turns_list = []

# Forward and backward grid position correction values
f_cor = 135
b_cor = 235

# Initial state of LEDs
led1 = 10
led2 = 20

'''
10 -> LED 1 (OFF)     20 -> LED 2 (OFF)
11 -> LED 1 (RED)     21 -> LED 2 (RED)
12 -> LED 1 (YEL)     22 -> LED 2 (YEL)
13 -> LED 1 (BLU)     23 -> LED 2 (BLU)
'''

success_flag = False # Truth value for delivery completion
total_score = 0 # Score of the run


#------------------------------------------------------------------------------#
################################# ARENA MAPPING ################################
#------------------------------------------------------------------------------#


############################### INPUT PERSPECTIVE ##############################

'''

Function Name: get_perspective_image

Input:  frame -> Image captured from the webcam

Output: Returns a perspective corrected and cropped image of the arena within the
        borders and list of the corner points of the this mage
        
Logic:  - Black border from the image is masked
        - Contours are detected around it
        - The biggest polygon is found in it
        - Points of the polygon are remapped to the cropped image
        - Perspective of the image is corrected
        
Example Call:   img, corners = get_perspective_image(frame)
                img is the perspective corrected image

'''

def get_perspective_image(frame):

    t_val = 160
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    lower = np.array([0, 0, 0]) #black color mask
    upper = np.array([t_val, t_val, t_val])
    mask = cv2.inRange(frame, lower, upper)
    
    ret,thresh = cv2.threshold(mask,127,255,cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #cv2.drawContours(frame,contours,-1,(0,255,0),3)
    biggest = 0
    max_area = 0
    min_size = thresh.size/4
    index1 = 0
    for i in contours:
        area = cv2.contourArea(i)
        if area > 10000:
            peri = cv2.arcLength(i,True)
                                 
        if area > max_area: 
            biggest = index1
            max_area = area
        index1 = index1 + 1
    approx = cv2.approxPolyDP(contours[biggest],0.05*peri,True)
    
    #drawing the biggest polyline
    cv2.polylines(frame, [approx], True, (0,255,0), 3)

    x1 = approx[0][0][0]
    y1 = approx[0][0][1]
    x2 = approx[1][0][0]
    y2 = approx[1][0][1]
    x3 = approx[3][0][0]
    y3 = approx[3][0][1]
    x4 = approx[2][0][0]
    y4 = approx[2][0][1]

    raw_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
    min_dist = 10000
    for i in raw_points:
        x,y = i
        dist = get_distance(x,y,0,0)
        if dist < min_dist:
            min_dist = dist
            X1,Y1 = x,y
    min_dist = 10000
    for i in raw_points:
        x,y = i
        dist = get_distance(x,y,0,480)
        if dist < min_dist:
            min_dist = dist
            X2,Y2 = x,y
    min_dist = 10000        
    for i in raw_points:
        x,y = i
        dist = get_distance(x,y,540,0)
        if dist < min_dist:
            min_dist = dist
            X3,Y3 = x,y
    min_dist = 10000
    for i in raw_points:
        x,y = i
        dist = get_distance(x,y,540,480)
        if dist < min_dist:
            min_dist = dist
            X4,Y4 = x,y
    
    #points remapped from source image from camera
    #to cropped image try to match x1, y1,.... to the respective near values
    pts1 = np.float32([[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]]) 
    pts2 = np.float32([[0,0],[0,480],[540,0],[540,480]])
    persM = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(frame,persM,(540,480))
    corner_points = [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]]
    print corner_points
    
    return (dst,corner_points)

'''

Function Name: crop_arena

Input:  frame -> Input image from webcam
        corner_list -> List of the corner points of the arena

Output: Returns a perspective corrected and cropped image of the arena

Logic:  - Corner points of the arena are remapped to the cropped image
        - Perspective of the image is corrected

Example Call:   img = crop_arena(frame,corner_list)
                img is the perspective corrected image

'''

def crop_arena(frame, corner_list):

    #points remapped from source image from camera
    #to cropped image try to match x1, y1,.... to the respective near values
    pts1 = np.float32(corner_list) 
    pts2 = np.float32([[0,0],[0,480],[540,0],[540,480]])
    
    persM = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(frame,persM,(540,480))
    
    return (dst)

def get_distance(x1,y1,x2,y2):

    distance = math.hypot(x2 - x1, y2 - y1)
    return distance

############################### MARKER POSITIONS ###############################

'''

Function Name: get_marker_color

Input:  img -> Image of the arena
        marker_number -> The number of the marker whose color is to be found
        (1-3) are service zone markers; (4-6) are patient zone markers

Output: Returns the color of the specified marker
        Keeps track of the number and type of provision requests

Logic:  - Create a small region of interest image of the marker specified
        - Find the mean red, green and blue values of this roi image
        - Check the range and return the detected color values (red, blue or yellow)
        - If no marker is present, return a white color value

Example Call:   color = get_marker_color(img, 1) // Returns the color of Marker 1 in service zone

'''

def get_marker_color(img, marker_number):

    global request_count

    # Get the x and y values
    if marker_number <= 3:
        x1 = 55
        x2 = 80
    else:
        x1 = 465
        x2 = 490

    y_list = [(70, 95), (230, 260), (390, 415),
              (60, 85), (220, 245), (375, 400)]
        
    y1, y2 = y_list[marker_number - 1]

    # Check marker color
    roi = img[y1:y2, x1:x2, :]
    blue, green, red, ret = cv2.mean(roi)
      
    if blue < 100 and green < 120 and red > 140:
        color = (0, 0, 255)          
        if marker_number > 3:
            request_count[0] += 1
                
    elif blue < 150 and green > 150 and red > 130:
        color = (0, 255, 255)
        if marker_number > 3:
            request_count[1] += 1
                
    elif blue > 140 and green < 200 and red < 140:
        color = (255, 0, 0)           
        if marker_number > 3:
            request_count[2] += 1
            
    else:
        color = (255, 255, 255)

    return(color)

################################ WALL POSITIONS ################################

'''

Function Name: check_wall

Input:  img -> A region of interest image of the wall position 

Output: Returns the truth value if a wall is present or not

Logic:  - Covert the BGR image to HSV image
        - Find the mean hue, saturation and value of the roi image
        - Check if it falls in the green color range
        - If it does, return True indicating a wall is present
        - Else, return False indicating a wall is not present

Example Call:   ret = check_wall(roi) // True if a wall is present

'''

def check_wall(img):

    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    hue, sat, val, ret = cv2.mean(hsv)

    if 65 <= hue <= 90 and sat > 60:
        return True
    else:
        return False

'''

Function Name: arena_mapping

Input:  arena -> Image of the arena

Output: A map of the arena with a grid size of 20 pixels is created and returned

Logic:  - A new image is created for mapping
        - The fixed partitions and the beds are drawn on the map
        - At various wall postions, if a wall present a wall is drawn, if not, then
          a white path is drawn which is traverseble for the robot
        - The service zone and the patient zone markers are drawn
          with their respective colors
        - Rest of the white path which is common for any configuration is drawn
        
Example Call: arena_map = arena_mapping(arena) // Arena mapping

'''

def arena_mapping(arena):
    
    global green_special
    new_condition = [0,0]
    
    grn = (0,140,0)
    wht = (255,255,255)
    blk = (0,0,0)
    gry = (80,80,80)
    grn_special = [0,0,0,0]
    
    grid = np.zeros((480,540,3), np.uint8)
    grid[20:460,20:520] = gry
    
    # Fixed partitions
    grid[140:180,20:80] = grn
    grid[300:340,20:80] = grn

    # Beds
    grid[20:60,420:520] = grn
    grid[160:200,420:520] = grn
    grid[320:360,420:520] = grn
    
    # Wall 1
    wall1 = arena[30:100,178:186,:]
    if(check_wall(wall1)):
        grid[20:120,160:200] = grn
        wall_positions.append(1)
    else:
        grid[60:80,140:240] = wht
        new_condition[0] = 0        
    
    # Wall 2    
    wall2 = arena[150:220,178:186,:]
    if(check_wall(wall2)):
        grid[120:240,160:200] = grn
        wall_positions.append(2)
    else:
        grid[180:200,140:240] = wht
        grn_special[0] = 1
    
    # Wall 3   
    wall3 = arena[260:330,178:186,:]
    if(check_wall(wall3)):
        grid[240:360,160:200] = grn
        wall_positions.append(3)
    else:
        grid[280:300,140:240] = wht
        grn_special[1] = 1
    
    # Wall 4    
    wall4 = arena[380:450,178:186,:]
    if(check_wall(wall4)):
        grid[360:460,160:200] = grn
        wall_positions.append(4)
    else: grid[400:420,140:240] = wht
    
    # Wall 5
    wall5 = arena[30:100,307:315,:]
    if(check_wall(wall5)):
        grid[20:120,300:340] = grn
        wall_positions.append(5)
    else:
        grid[60:80,260:380] = wht
        new_condition[1] = 1
    
    # Wall 6
    wall6 = arena[150:220,307:315,:]
    if(check_wall(wall6)):
        grid[120:240,300:340] = grn
        wall_positions.append(6)
    else:
        grid[180:200,260:360] = wht
        grn_special[2] = 1
    
    # Wall 7
    wall7 = arena[260:330,307:315,:]
    if(check_wall(wall7)):
        grid[240:360,300:340] = grn
        wall_positions.append(7)
    else:
        grid[280:300,260:360] = wht
        grn_special[3] = 1
    
    # Wall 8
    wall8 = arena[380:450,307:315,:]
    if(check_wall(wall8)):
        grid[360:460,300:340] = grn
        wall_positions.append(8)
    else: grid[400:420,260:380] = wht

    '''# Special conditions for certain configurations
    if grn_special[0] is 1 and grn_special[1] is 1:
        grid[240:260,140:240] = wht
    if grn_special[2] is 1 and grn_special[3] is 1:
        grid[240:260,260:360] = wht'''

    if grn_special[0] is 1 and grn_special[1] is 1 and grn_special[2] is 1 and grn_special[3] is 1:
        grid[240:260,140:240] = wht
        grid[240:260,260:360] = wht
        green_special = True

    if new_condition[0]is 1 and new_condition[1] is 1:
        green_special = True

    if green_special:
        mx1 = 240
        mx2 = 260
    else:
        mx1 = 260
        mx2 = 280
    
    # Service zone markers
    grid[60:80,80:100] = get_marker_color(arena,1)
    grid[240:260,80:100] = get_marker_color(arena,2)
    grid[400:420,80:100] = get_marker_color(arena,3)
    
    # Patient zone markers
    grid[100:120,460:480] = get_marker_color(arena,4)
    grid[mx1:mx2,460:480] = get_marker_color(arena,5)
    grid[400:420,460:480] = get_marker_color(arena,6)
    
    # White common path
    grid[60:420,240:260] = wht
    grid[60:420,120:140] = wht
    grid[80:420,360:380] = wht
    
    grid[60:80,100:120] = wht
    grid[240:260,100:120] = wht
    grid[400:420,100:120] = wht
    
    grid[100:120,380:460] = wht
    grid[mx1:mx2,380:460] = wht
    grid[400:420,380:460] = wht

    #cv2.imshow('Grid Mapping',grid)

    return grid

############################# ROTATION OF FIREBIRD #############################

'''

Function Name: get_largest_contour

Input:  contours -> A list of all contours detected

Output: Returns the largest of all the contours from the list

Logic:  - The area of all the contours are found out
        - The contour with the largest area is selected

Example Call: largest_contour = get_largest_contour(contours)

'''

def get_largest_contour(contours):

    max_area = 0
    largest = -1
    
    for i in contours:
        contour_area = cv2.contourArea(i)
        if contour_area > max_area:
            max_area = contour_area
            largest = i

    return largest

'''

Function Name: get_moments

Input:  img -> Image where the moments of a particular color is to be found
        param1 -> HSV values for the lower value of the range of the color
        param2 -> HSV values for the upper value of the range of the color
        
Output: Returns the x and y values of the moment (centre position of the color)

Logic:  - BGR image is converted to HSV image
        - The required color is masked
        - Morphology operations are done to get a clean mask
        - Image is thresholded and contours are detected
        - Moments of the largest contour are found out

Example Call: cx,cy = get_moments(img, [0,200,200], [10,255,255]) // To get centroids of red color

'''

def get_moments(img, param1, param2, mode=0):

    img1 = img.copy()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower = np.array(param1)
    upper = np.array(param2)
    
    mask = cv2.inRange(hsv, lower, upper)
    res  = cv2.bitwise_and(hsv, hsv, mask= mask)

    # Morphology operations are done to get a clean mask
    kernel = np.ones((5,5), np.uint8)
    #res = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
    res = cv2.morphologyEx(res, cv2.MORPH_CLOSE, kernel)
    res  = cv2.GaussianBlur(res, (5,5), 0)

    if mode>0:
        f_name = 'Position/Start'+str(mode)+'.jpg'
        cv2.imwrite(f_name,res)
    
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 10, 255, cv2.THRESH_TOZERO)
        
    # Finding the centroid of the masked region by finding contours
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = get_largest_contour(contours)
    
    M  = cv2.moments(largest_contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    cv2.drawContours(img1, contours, -1, (0,255,0), 1)
    cv2.circle(img1,(cx,cy),5,(0,255,0),-1)

    return cx,cy,img1

'''

Function Name: get_direction

Input:  img -> Image where the robot is present

Output: Returns the direction of the robot
        1 -> North
        2 -> West
        3 -> South
        4 -> East

Logic:  - Finds the centroids of the two color stickers prsent on the robot
        - Calculates the slope (tan inversere of dy/dx) of the line joining
          the two centroids
        - Returns the direction of the robot based on the value of the slope

Example Call: rot = get_direction(arena_image) // Eg: rot = 1 for North

'''

def get_direction(img):

    o1 = [10,120,220] # Orange sticker
    o2 = [30,200,255]
    p1 = [155,100,210] # Pink sticker
    p2 = [180,175,255]

    x1, y1, ret1 = get_moments(img, o1, o2, 1)
    x2, y2, ret2 = get_moments(img, p1, p2, 2)

    cv2.imshow('Orange',ret1)
    cv2.imshow('Pink',ret2)

    dx = x2-x1
    dy = y2-y1
    print dx,dy

    # Calculte the angle
    rads = math.atan2(dy, dx)
    degs = math.degrees(rads)
    print rads,degs
    
    if 50 <= degs <= 130:
        return 1    
    elif -130 <= degs <= -50:
        return 3   
    elif abs(degs) <= 40:
        return 2
    elif abs(degs) >= 140:
        return 4 
        
    else:
        warnings.warn('Error in Direction Detection!')
        return 0

############################# POSITION OF FIREBIRD #############################

'''

Function Name: check_position

Input:  img -> A region of interest image of the start position

Output: Returns a truth value wether the robot is present or not

Logic:  - BGR image is converted to HSV image
        - Get the mean HSV values of the ROI image
        - Check if the robot is present or not

Example Call: ret = check_position(img) // True if robot is present

'''

def check_position(img):

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hue, sat, val, ret = cv2.mean(hsv)

    if(sat > 30):
        return True
    else:
        return False

'''

Function Name: get_position

Input:  img -> Image of the arena
        arena_grid -> Arena map image

Output: The start grid is marked orange in the start position in the arena map
        Returns the start position, the co-ordinates and the start direction of the robot

Logic:  - Create ROI images of the various all the start positions
        - Check at which position the robot is prsent
        - Mark the start position in the arena map
        - Get the orientation of the robot

Example Call: pos, coordinates, rot = get_position(img, arena_grid)

'''

def get_position(img, arena_grid):
    
    start_1 = (12,3)
    start_2 = (12,12)
    start_3 = (12,20)
    
    pos1 = img[25:100, 212:287]  # Start position 1
    pos2 = img[200:280, 212:287] # Start position 2
    pos3 = img[380:450, 212:287] # Start position 3

    if check_position(pos1):
        arena_grid[60:80, 240:260] = (0, 100, 180)
        rotation = get_direction(pos1)
        return (1, start_1, rotation)
    
    elif check_position(pos2):
        arena_grid[240:260, 240:260] = (0, 100, 180)
        rotation = get_direction(pos2)
        return (2, start_2, rotation)
    
    elif check_position(pos3):
        arena_grid[400:420, 240:260] = (0, 100, 180)
        rotation = get_direction(pos3)
        return (3, start_3, rotation)
    
    else:
        return (0,0,0)


#------------------------------------------------------------------------------#
################################# PATH PLANNING ################################
#------------------------------------------------------------------------------#
    

################################# A* ALGORITHM #################################

'''

Function Name: get_obstacle_map

Input:  img -> Arena map image

Output: Creates a list (wall_map) which consists of the co-ordinates of the obstacles

Logic:  - For every grid in the arena map, check if it is an obstacle
        - Appned the co-ordinates of the obstacles in the obstacles list

Example Call: get_obstacle_map(img) // Creates the obstacles list

'''

def get_obstacle_map(img):

    y = 10
    for i in range(0,24):
        x = 10
        if(y < height):
            for j in range (0,27):
                if(x < width):
                    #Detecting the colors
                    blue, green, red = img[y,x]
                    color = np.array([blue, green, red])

                    if blue < 100 and green < 100 and red < 100:
                        obstacles.append((i,j))
                    elif blue < 10 and green > 100 and red < 10:
                        obstacles.append((i,j))

                    x = x + 20
        y = y + 20
        
class Node(object):

    '''

    Function Name: __init__

    Input:  node -> Object of the class Node
            x, y -> Coordinates of the current node
            space -> Truth vale if a grid is a free space or not
            
    Output: Initialise all the parameters for A* algorithm

    Logic:  - Initialising the co-ordinates, parent node, space flag
              and f,g and h values for A* algorithm

    Example Call: Called automatically by the operating system

    '''
    
    def __init__(node, x, y, space):

        #Co-ordinates of the current node
        node.x = x
        node.y = y
        
        #Black space flag and parent node
        node.space = space
        node.parent = None
        
        #f, g, h values for A* algorithm
        node.f = 0
        node.g = 0
        node.h = 0

class PathPlanning(object):

    '''

    Function Name: __init__

    Input:  self -> Object of the class PathPlanning

    Output: Initialise all the parameters for A* path planning

    Logic:  - Initialise the open and closed lists
            - Define the number of rows and columns of the arena grid

    Example Call: Called automatically by the operating system

    '''
    
    def __init__(self):

        #Open list and closed list
        self.open_list  = []
        self.close_list = set()
        
        #Transform open list into a heap (node with lowest f value at top)
        heapq.heapify(self.open_list)
        
        #List of nodes which are free white spaces
        self.nodes = []
        
        #Number of rows and columns of the grid
        self.rows = 24
        self.cols = 27

    '''

    Function Name: get_node

    Input:  self -> Object of the class PathPlanning
            x -> x co-ordinate of the node
            y -> y co-ordinate of the node

    Output: Returns a node based on the position

    Logic:  - Returns a node based on the position from the nodes list

    Example Call: self.start = self.get_node(start_x, start_y)

    '''
    
    def get_node(self, x, y):

        pos = self.nodes[(x * self.cols) + y]
        return pos

    '''

    Function Name: grid_map

    Input:  self -> Object of the class PathPlanning
            start_x -> x co-ordinate of the start node
            start_y -> y co-ordinate of the start node
            end_x -> x co-ordinate of the end node
            end_y -> y co-ordinate of the end node

    Output: Creates a binary grid map (list) and assigns every node with a space flag

    Logic:  - For every node in the arena grid, assign a space flag truth value,
              0 -> Traversable free space
              1 -> Obstacle
            - Set the start and the end node for path planning

    Example Call: a_star.grid_map(start_x, start_y, end_x, end_y)

    '''
    
    def grid_map(self, start_x, start_y, end_x, end_y):

        for i in range(0,self.rows):
            for j in range(0,self.cols):

                if (i,j) in obstacles:
                    space = 0
                else:
                    space = 1
                self.nodes.append(Node(i,j,space))
                
        #Set the positions for start and end node
        self.start = self.get_node(start_x, start_y)
        self.end   = self.get_node(end_x, end_y)

    '''

    Function Name: get_adjacent

    Input:  self -> Object of the class PathPlanning
            node -> Object of the class Node (current node)
            
    Output: Returns a list of adjacent nodes around the parent node (clockwise)

    Logic:  - From the parent node, get the nodes around it in a clockwise manner
              and append it to the adj_nodes list
            - Checks for boundary conditions also

    Example Call: adj_list = self.get_adjacent(node)

    '''
    
    def get_adjacent(self, node):

        adj_nodes = []

        if (node.x < self.cols-1):
            adj_nodes.append(self.get_node(node.x+1, node.y))
        if (node.y > 0):
            adj_nodes.append(self.get_node(node.x, node.y-1))
        if (node.x > 0):
            adj_nodes.append(self.get_node(node.x-1, node.y))
        if (node.y < self.rows-1):
            adj_nodes.append(self.get_node(node.x, node.y+1))        
        
        return adj_nodes

    '''

    Function Name: get_heuristic

    Input:  self -> Object of the class PathPlanning
            node -> Object of the class Node (current node)

    Output: Returns the heuristic value of the current node
            (Cost from current node to the end node)

    Logic:  - The Manhattan distnace is computed as follows:
              |Current_node_x - End_node_x| + |Current_node_y - End_node_y|
            - It is then multiplied by a factor of 10

    Example Call: adj.h = self.get_heuristic(adj)

    '''
    
    def get_heuristic(self, node):

        #H factor value
        H_FACTOR = 24
        dx = abs(node.x - self.end.x)
        dy = abs(node.y - self.end.y)
        h  = H_FACTOR * (dx + dy)
        
        return h

    '''

    Function Name: update_values

    Input:  self -> Object of the class PathPlanning
            adj -> Object of the class Node (adjacent node)
            node -> Object of the class Node (current node)

    Output: Updates the values of f,g and h values for A* algorithm
            The adjacent parent node is updated

    Logic:  f -> Total cost
            g -> Cost from start node to the current node
            h -> Heuristic value - cost from current node to the end node
            - The above costs are updated by adding them with the
              costs of the adjacent nodes
            - The current node is then set as the parent node for the adjacent node

    Example Call: self.update_values(adj, node)

    '''
   
    def update_values(self, adj, node):

        adj.g = node.g + 20
        adj.h = self.get_heuristic(adj)
        adj.f = adj.g + adj.h
        adj.parent = node

    '''

    Function Name: path_list

    Input:  self -> Object of the class PathPlanning
            route_path -> List of the co-ordonates of the nodes in the shortest path

    Output: Update the route_path list with the shortest path computed

    Logic:  - Back trace the path until the start node is encountered
            - Appened the co-ordinates of the nodes to the route_path list

    Example Call: self.path_list(route_path)

    '''
    
    def path_list(self, route_path):

        node = self.end
        
        # Back trace the path until the start node is encountered
        while(node.parent is not self.start):
            node = node.parent
            route_path.append((node.y+1,node.x+1))

    '''

    Function Name: a_star_algorithm

    Input:  self -> Object of the class PathPlanning
            route_path -> List of the co-ordonates of the nodes in the shortest path

    Output: Compute the shortest path using A* algorithm

    Logic:  - open_list -> Heapified to store the nodes that are processsed already
              and store the node with the lowest f value at the top
            - Node with the lowest f value is popped
            - Its adjacent nodes are found
            - Current node is set as the parent node and is added to the closed list
            - Adjacent nodes are then processed to get the lowest f value and then
              added to the open list heap
            - The algorithm continues in the same manner until the end node is reached
            - Starting with the end node, the parent node of the all the nodes is back
              traced until the start node is encountered

    Example Call: a_star.a_star_algorithm(route_path)

    '''
    
    def a_star_algorithm(self, route_path):
        
        #Add start node to open heap queue
        heapq.heappush(self.open_list, (self.start.f, self.start))

        while(len(self.open_list)):
            
            #Pop node from open heap queue
            f,node = heapq.heappop(self.open_list)
            self.close_list.add(node)
            
            #If it is the end node, then append the path list
            if node is self.end:
                self.path_list(route_path)
                break
            
            #To get a list of adjacent nodes
            adj_list = self.get_adjacent(node)
            
            #Get adjacent nodes and compare the f values
            for adj in adj_list:
                if(adj.space and (adj not in self.close_list)):
                    if((adj.f, adj) in self.open_list):
                        if(adj.g > (node.g + 20)):
                            self.update_values(adj, node)
                    else:
                        self.update_values(adj, node)
                        heapq.heappush(self.open_list, (adj.f, adj))

'''

Function Name: get_path

Input:  img -> Arena map image
        start -> Start node co-ordinates
        end -> End node co-ordinates

Output: Returns the shortest path and its length between two points using A* algorithm

Logic:  - Create a list of the obstacle positions
        - Calculate the shortest path using A* algorithm between any two nodes
        - Reverse the path list (as A* backtraces the path)
        - Compute its length
        - Return the route path and its length

Example Call: length,path = get_path(org, m[i], m[j])

'''

def get_path(img, start, end):

    route_path = []
    del obstacles[:]
        
    start_y,start_x = start
    end_y,end_x = end

    if start_x is end_x and start_y is end_y:
        route_length = 0
        route_path = [(end_y+1,end_x+1)]

    else:
        get_obstacle_map(img)
        route_path.append((end_y+1, end_x+1))

        a_star = PathPlanning() 
        a_star.grid_map(start_x, start_y, end_x, end_y)
        a_star.a_star_algorithm(route_path)

        route_path.append((start_y+1, start_x+1))
        route_path.reverse()
        route_length = len(route_path)-1

    return route_length,[route_path]

################################# PATH PLANNING ################################

'''

Function Name: get_marker_pos

Input:  img -> Arena map image
        mode -> Mode of operation of this function
           0 -> To update the request_matched count
           1 -> To return the positions of markers

Output: Matched the provisions in the service and patient zone and returns their positions

Logic:  - The color of the service marker is stored as color1
        - The color of the patient marker is stored as color2
        - If the colors match, then the positions of the markers are appended
          in the service_list and patient_list respectively as matched

Example Call: m,p = get_marker_pos(org, 1)

'''

def get_marker_pos(img, mode):

    global request_matched, green_special

    if green_special:
        mx = 12
    else:
        mx = 13

    service_list = [(-1,-1),(-1,-1),(-1,-1)]
    patient_list = [(-1,-1),(-1,-1),(-1,-1)]

    marker_list = [[(4,3),(4,12),(4,20)],
                [(23,5),(23,mx),(23,20)]]

    count = 0
    for i in range(0,3):
        y1,x1 = marker_list[0][i]
        b1,g1,r1 = img[10+(x1*20),10+(y1*20)]
        color1 = np.array([b1,g1,r1])

        if not(np.all(color1 == 255)):

            for j in range(0,3):
                y2,x2 = marker_list[1][j]
                b2,g2,r2 = img[10+(x2*20),10+(y2*20)]
                color2 = np.array([b2,g2,r2])
                
                if np.all(color1 == color2):
                    service_list[count] = (y1,x1)
                    patient_list[count] = (y2,x2)
                    count += 1

    request_matched = count

    if mode is 0:
        return None
    
    elif mode is 1:
        return service_list, patient_list

'''

Function Name: get_all_paths

Input:  org -> Arena map image
        start -> Start point co-ordinates
        index -> Number of provision deliveries to be made

Output: Calculates the path and length from start position to each provision
        in the service zone, from each provision in the service zone to the patient
        tables and also from each patient table to the service zone

Logic:  s -> Start position (Corridor zone)
        m -> Service zone
        p -> Patient zone

        sm -> Start to service zone
        mm -> From one provision in the service zone to aonther
        mp -> From service zone to patient zone tables
        pp -> From one table in the patient zone to another
        pm -> From tables in the patient zone to provisions in the service zone

        - All possible paths are computed between two points using A* algorithm
          in the given configuration as mentined above
        - These individual path lists and lengths and stored in the corresponding
          lists based on their zones as mentioned above

Example Call: get_all_paths(org, pos_cord, 2)

'''

def get_all_paths(org, start, index):

    m,p = get_marker_pos(org, 1)

    print '\nService Markers:',m
    print 'Patient Tables:',p  

    # Start to service zone
    for i in range(0,index):
        length,path = get_path(org, start, m[i])
        sm_len.append(length)
        sm_path.append(path)

    # Service zone to service zone
    for i in range(0,index):
        temp1 = []
        temp2 = []
        for j in range(0,index):
            if i is not j:
                if index is 2:
                    temp1,temp2 = get_path(org, m[i], m[j])
                elif index is 3:
                    length,path = get_path(org, m[i], m[j])
                    temp1.append(length)
                    temp2.append(path)       
        mm_len.append(temp1)
        mm_path.append(temp2)

    # Service zone to patient zone
    for i in range(0,index):
        temp1 = []
        temp2 = []
        for j in range(0,index):
            length,path = get_path(org, m[i], p[j])
            temp1.append(length)
            temp2.append(path)
        mp_len.append(temp1)
        mp_path.append(temp2)

    # Patient zone to service zone
    if index is 3:
        for i in range(0,3):
            temp1 = []
            temp2 = []
            for j in range(0,3):
                if i is not j:
                    length,path = get_path(org, p[i], m[j])
                    temp1.append(length)
                    temp2.append(path)
            pm_len.append(temp1)
            pm_path.append(temp2)

    # PAtient zone to patient zone
    for i in range(0,index):
        temp1 = []
        temp2 = []
        for j in range(0,index):
            if i is not j:
                if index is 2:
                    temp1,temp2 = get_path(org, p[i] ,p[j])
                elif index is 3:
                    length,path = get_path(org, p[i], p[j])
                    temp1.append(length)
                    temp2.append(path)
        pp_len.append(temp1)
        pp_path.append(temp2)

    #print '\nSM:',sm_len
    #print 'MM:',mm_len
    #print 'MP:',mp_len
    #print 'PM:',pm_len
    #print 'PP:',pp_len

'''

Function Name: get_turns

Input:  path -> List containing the path co-ordinates

Output: Computes the number of turns in a given path

Logic:  - Based on the change in differnce of the x and y values of
          two nodes in the path, turns are computed

Example Call: a.append(get_turns(path))

'''

def get_turns(path):

    turns = 0
    for i in path:
        arr = np.array(i)
        turns += ((np.abs(arr[2:] - arr[:-2])>0).sum(axis=1)==2).sum()

    return turns

'''

Function Name: get_min_path1

Input:  org -> Arena map image
        start -> Start point co-ordinates

Output: Computes the shortest path for only 1 delivery

Logic:  - Since only one delivery takes place, the master path is
          the shortest path between the start to the service zone to
          the patient table
        - The shortest master path, its length and numner of turns
          are computed and returned

Example Call: master_len,master_path,master_index,master_turns = get_min_path1(org,pos_cord)

'''

def get_min_path1(org, start):
    
    m,p = get_marker_pos(org,1)
    #print '\nService Markers:',m[0]
    #print 'Patient Tables:',p[0]
    
    length1,path1 = get_path(org, start, m[0])
    length2,path2 = get_path(org, m[0], p[0])

    min_len = length1 + length2
    min_path = path1 + path2
    min_index_list = [0,0]
    min_turns = get_turns(path1) + get_turns(path2)
    
    return min_len, min_path, min_index_list, min_turns

'''

Function Name: get_min_path2

Input:  index_list -> The list of indices of the different path length lists betwwen
        variuos zones for the six possible master paths while delivering two provisions

Output: The shortest path with least turns of the six possible master path is selected
        while delivering only 2 provisions in the arena

Logic:  - Calculate the length of all the 6 master paths
        - Select the path with the least overall length
        - If two paths have the same length, select the one with least number of turns
        - Return the master path, its length and the number of turns

Example Call: master_len,master_path,master_index,master_turns = get_min_path2(index_2)

'''

def get_min_path2(index_list):

    length = 0
    path = []
    sub_len = []

    # Get all 6 path lengths
    for a in index_list:
        length = sm_len[a[0]] + mm_len[a[1]] + mp_len[a[2]][a[3]] + pp_len[a[4]]
        path = sm_path[a[0]] + mm_path[a[1]] + mp_path[a[2]][a[3]] + pp_path[a[4]]

        a.append(length)
        a.append(get_turns(path))
        sub_len.append(length)

    # Choose the shortest path
    index_list = sorted(index_list, key=itemgetter(5))
    sub_len = sorted(sub_len)

    min_turns = 1000
    min_len = sub_len[0]

    # Choose the one with least turns
    for a in index_list:
        if a[5] is min_len and a[6] < min_turns:
            min_turns = a[6]
            min_len = a[5]
            min_index_list = a
            
    #print '\nAll Lengths:',sub_len
    print '\nAll Index:'
    for i in index_list: print i

    # Shortest path
    a = min_index_list
    min_path = []
    min_path = sm_path[a[0]] + mm_path[a[1]] + mp_path[a[2]][a[3]] + pp_path[a[4]]
    
    return min_len,min_path,min_index_list,min_turns

'''

Function Name: get_min_path3

Input:  index_list -> The list of indices of the different path length lists betwwen
        variuos zones for the 12 possible master paths while delivering three provisions

Output: The shortest path with least turns of the 12 possible master paths is selected
        while delivering only 3 provisions in the arena

Logic:  - Calculate the length of all the 12 possible master paths
        - Select the path with the least overall length
        - If two paths have the same length, select the one with least number of turns
        - Return the master path, its length and the number of turns

Example Call: master_len,master_path,master_index,master_turns = get_min_path3(index_3)

'''

def get_min_path3(index_list):

    length = 0
    path = []
    sub_len = []

    # Get all 12 path lengths
    for a in index_list:
        length = sm_len[a[0]] + mm_len[a[1]][a[2]] + mp_len[a[3]][a[4]]
        length += pp_len[a[5]][a[6]] + pm_len[a[7]][a[8]] + mp_len[a[9]][a[10]]

        path = sm_path[a[0]] + mm_path[a[1]][a[2]] + mp_path[a[3]][a[4]]
        path += pp_path[a[5]][a[6]] + pm_path[a[7]][a[8]] + mp_path[a[9]][a[10]]

        a.append(length)
        a.append(get_turns(path))
        sub_len.append(length)

    # Choose the shortest path
    index_list = sorted(index_list, key=itemgetter(11))
    sub_len = sorted(sub_len)

    min_turns = 1000
    min_len = sub_len[0]

    for a in index_list:
        if a[11] is min_len and a[12] < min_turns:
            min_turns = a[12]
            min_len = a[11]
            min_index_list = a
            
    #print '\nAll Lengths:',sub_len
    print '\nAll Index:'
    for i in index_list: print i

    # Shortest path
    a = min_index_list
    min_path = []
    min_path = sm_path[a[0]] + mm_path[a[1]][a[2]] + mp_path[a[3]][a[4]]
    min_path += pp_path[a[5]][a[6]] + pm_path[a[7]][a[8]] + mp_path[a[9]][a[10]]
    
    return min_len,min_path,min_index_list,min_turns

################################ DRAW FUNCTIONS ################################

'''

Function Name: draw_grid

Input:  img -> Image to draw the grid on
        grid_x, grid_y -> Number of rows and columns in the grid
        g_val -> Gray color value of the grid

Output: A grid of specified parameters is drwan on the image

Logic:  - Draw equally spaced vertical lines
        - Draw equally spaced horizintal lines

Example Call: arena_grid = draw_grid(arena_grid, 20, 20, 140)

'''

def draw_grid(img, grid_x, grid_y, g_val):
    
    for i in range(0,54):
        cv2.line(img, (grid_x*i,0), (grid_x*i, height), (g_val,g_val,g_val), 1)
    for i in range(0,48):
        cv2.line(img, (0,grid_y*i), (width, grid_y*i), (g_val,g_val,g_val), 1)

    return(img)

'''

Function Name: draw_firebird

Input:  img -> Image of the arena
        pos -> Start position of the robot
        rot -> Start orientation of the robot

Output: Draws the outline of the robot in the arena and direction is indicated
        by a circle drawn towards the front of the robot

Logic:  - Based on the start position, draw an outline circle of the robot
        - Indicate the direction by a circle drawn towards the front of the robot

Example Call: draw_firebird(img, pos, rot)

'''

def draw_firebird(img, pos, rot):

    if pos is 1:
        x = 250
        y = 60
    elif pos is 2:
        x = 250
        y = 240
    elif pos is 3:
        x = 250
        y = 420

    cv2.circle(arena,(x,y),35,(0,255,0),2)

    if rot is 1: y = y-15
    elif rot is 2: x = x-15
    elif rot is 3: y = y+15
    elif rot is 4: x = x+15

    cv2.circle(arena,(x,y),8,(0,255,0),-1)

'''

Function Name: draw_path

Input:  flag -> To keep a track of the path number
        img -> Image of the arena where path is to be drawn
        path -> The path list
        ex, ey -> Co-ordinates of end points in a individual path

Output: Draws the path on the image

Logic:  - All the individual paths are drwan with seperate colors as follows:
        Path 1: Blue
        Path 2: Cyan
        Path 3: Green
        Path 4: Yellow
        Path 5: Red
        Path 6: Purple

        - The end of each individual paths of the master path are marker by
          a circle of the corressponding color

Example Call: draw_path(col_flag, image, i, ex, ey)

'''

def draw_path(flag, img, path, ex, ey):

    # Set the color
    pb = pg = pr = 0
    if 1 <= flag <=2 or flag is 6: pb = 255
    if 2 <= flag <= 4: pg = 255
    if 4 <= flag <= 6: pr = 255

    # Draw the path
    length = len(path)-4
    for i in range(0,length):
        y1,x1 = path[i]
        y2,x2 = path[i+1]   
        cv2.line(img,((y1*20)-10,(x1*20)-10),((y2*20)-10,(x2*20)-10),(pb,pg,pr),3)
    cv2.circle(img,((ey*20)-10,(ex*20)-10),8,(pb,pg,pr),-1)
    

#------------------------------------------------------------------------------#
################################# COMMUNICATION ################################
#------------------------------------------------------------------------------#
    

################################### ENCODING ###################################

'''

Function Name: led_status

Input:  y -> y co-ordinate value of the markers
        L1 -> Color command of LED 1 
        L2 -> Color command of LED 2
        led1 -> Color command assigned to LED 1
        led2 -> Color command assigned to LED 2
        flag1 -> Status of LED 1 (ON/OFF)
        flag2 -> Status of LED 2 (ON/OFF)

Output: Updates the status of the two leds

Logic:  - Based on the status (ON/OFF) of the two LEDs,
          they are assigned the LED color command
          
        10 -> LED 1 (OFF)       20 -> LED 2 (OFF)
        11 -> LED 1 (RED)       21 -> LED 2 (RED)
        12 -> LED 1 (YEL)       22 -> LED 2 (YEL)
        13 -> LED 1 (BLU)       23 -> LED 2 (BLU)

Example Call: led1,led2,flag1,flag2 = led_status(y,11,21,led1,led2,flag1,flag2)

'''

def led_status(y, L1, L2, led1, led2, flag1, flag2):

    if flag1 is 'OFF' and flag2 is 'OFF':
        if y<10:
            led1 = L1
            flag1 = 'ON'
            
    elif flag1 is 'ON' and flag2 is 'OFF':
        if y<10:
            led2 = L2
            flag2 = 'ON'
        if y>10:
            led1 = 10
            flag1 = 'OFF'
            
    elif flag2 is 'ON' and flag1 is 'OFF':
        if y>10:
            led2 = 20
            flag2 = 'OFF'
        if y<10:
            led1 = L1
            flag1 = 'ON'
            
    elif flag1 is 'ON' and flag2 is 'ON':
        if y>10:
            if led1 is L1:
                led1 = 10
                flag1 = 'OFF'
            elif led2 is L2:
                led2 = 20
                flag2 = 'OFF'

    return(led1, led2, flag1, flag2)    

'''

Function Name: led_color

Input:  path -> The master path list
        org -> Arena map image

Output: The LED commands are appended to each individual path list at the end

Logic:  - The end point of each indivudual path is found
        - The color of the end point (marker) is found out from the arena map
        - Based on the color, variuos LED commands are appended after the end point in the list
        - The two LEDs are provided with commands based on their status (ON/OFF) availablity

        10 -> LED 1 (OFF)       20 -> LED 2 (OFF)
        11 -> LED 1 (RED)       21 -> LED 2 (RED)
        12 -> LED 1 (YEL)       22 -> LED 2 (YEL)
        13 -> LED 1 (BLU)       23 -> LED 2 (BLU)

Example Call: led_color(master_path, arena_map)

'''

def led_color(path, org):

    global led1,led2
    flag1 = 'OFF'
    flag2 = 'OFF'
    
    print '\nLED Info:'
    for i in path:
        y,x = i[-2]
        blue, green, red = org[((x-1)*20)+10,((y-1)*20)+10]
        
        if blue < 10 and green < 10 and red > 120:
            led1,led2,flag1,flag2 = led_status(y,11,21,led1,led2,flag1,flag2)

        elif blue < 10 and green > 120 and red > 120:
            led1,led2,flag1,flag2 = led_status(y,12,22,led1,led2,flag1,flag2)

        elif blue > 120 and green < 10 and red < 10:
            led1,led2,flag1,flag2 = led_status(y,13,23,led1,led2,flag1,flag2)
                
        print led1,led2
        i.extend((led1,led2))

'''

Function Name: encode_north

Input:  a -> Individual path list

Output: The path is analysed and encoded into commands for the robot

Logic:  - This function is used for encoding when the robot is facing North
        - Based on the difference of x and y values of two consecutive set
          of co-ordinates in the list, the direction change is detected
        - Based on the new direction, the encoding function for that particular
          direction is then called and the path starting from that point is sent to that function
        - If the end is encountered (indicated by (-1,-1)), then encoding is stopped

Example Call: ret = encode_north(path)

'''

def encode_north(a):

    global f_cor,b_cor
    for i in range(1,len(a)):

        if a[i][0] is -1: return True
        
        elif a[i][1]-a[i-1][1] is -1:
            motion.append(1)

        elif a[i][1]-a[i-1][1] is 1:
            motion.append(3)
            
        elif a[i][0]-a[i-1][0] is -1:
            temp = a[i-1:]
            motion.extend((f_cor,2,b_cor))
            ret = encode_west(temp)
            del temp
            if (ret): return True
            
        elif a[i][0]-a[i-1][0] is 1:            
            temp = a[i-1:]
            motion.extend((f_cor,4,b_cor))
            ret = encode_east(temp)
            del temp
            if (ret): return True

'''

Function Name: encode_west

Input:  a -> Individual path list

Output: The path is analysed and encoded into commands for the robot

Logic:  - This function is used for encoding when the robot is facing West
        - Based on the difference of x and y values of two consecutive set
          of co-ordinates in the list, the direction change is detected
        - Based on the new direction, the encoding function for that particular
          direction is then called and the path starting from that point is sent to that function
        - If the end is encountered (indicated by (-1,-1)), then encoding is stopped

Example Call: ret = encode_west(path)

'''

def encode_west(a):

    global f_cor,b_cor
    for i in range(1,len(a)):

        if a[i][0] is -1: return True
        
        elif a[i][0]-a[i-1][0] is -1:
            motion.append(1)

        elif a[i][0]-a[i-1][0] is 1:
            motion.append(3)
            
        elif a[i][1]-a[i-1][1] is 1:          
            temp = a[i-1:]
            motion.extend((f_cor,2,b_cor))
            ret = encode_south(temp)           
            del temp
            if (ret): return True
            
        elif a[i][1]-a[i-1][1] is -1:
            temp = a[i-1:]
            motion.extend((f_cor,4,b_cor))
            ret = encode_north(temp)
            del temp
            if (ret): return True
            
'''

Function Name: encode_south

Input:  a -> Individual path list

Output: The path is analysed and encoded into commands for the robot

Logic:  - This function is used for encoding when the robot is facing South
        - Based on the difference of x and y values of two consecutive set
          of co-ordinates in the list, the direction change is detected
        - Based on the new direction, the encoding function for that particular
          direction is then called and the path starting from that point is sent to that function
        - If the end is encountered (indicated by (-1,-1)), then encoding is stopped

Example Call: ret = encode_south(path)

'''

def encode_south(a):

    global f_cor,b_cor
    for i in range(1,len(a)):

        if a[i][0] is -1: return True

        elif a[i][1]-a[i-1][1] is 1:
            motion.append(1)

        elif a[i][1]-a[i-1][1] is -1:
            motion.append(3)
            
        elif a[i][0]-a[i-1][0] is 1:
            temp = a[i-1:]
            motion.extend((f_cor,2,b_cor))
            ret = encode_east(temp)
            del temp
            if (ret): return True
            
        elif a[i][0]-a[i-1][0] is -1:
            temp = a[i-1:]
            motion.extend((f_cor,4,b_cor))
            ret = encode_west(temp)
            del temp
            if (ret): return True

'''

Function Name: encode_east

Input:  a -> Individual path list

Output: The path is analysed and encoded into commands for the robot

Logic:  - This function is used for encoding when the robot is facing East
        - Based on the difference of x and y values of two consecutive set
          of co-ordinates in the list, the direction change is detected
        - Based on the new direction, the encoding function for that particular
          direction is then called and the path starting from that point is sent to that function
        - If the end is encountered (indicated by (-1,-1)), then encoding is stopped

Example Call: ret = encode_east(path)

'''

def encode_east(a):

    global f_cor,b_cor
    for i in range(1,len(a)):

        if a[i][0] is -1: return True
        
        elif a[i][0]-a[i-1][0] is 1:
            motion.append(1)

        elif a[i][0]-a[i-1][0] is -1:
            motion.append(3)
            
        elif a[i][1]-a[i-1][1] is -1:           
            temp = a[i-1:]
            motion.extend((f_cor,2,b_cor))
            ret = encode_north(temp)
            del temp
            if (ret): return True
            
        elif a[i][1]-a[i-1][1] is 1:
            temp = a[i-1:]
            motion.extend((f_cor,4,b_cor))
            ret = encode_south(temp)
            del temp
            if (ret): return True

'''

Function Name: encode_path

Input:  path -> The master path list
        rotation -> The start oreintation of the robot

Output: The entire path of the robot in the arena is encoded into commands
        which is to be sent to the robot

        NOTE: PLEASE REFER TO THE ENCODED COMMANDS LIST PDF FILE FOR
              THE COMPLETE LIST OF COMMANDS

Logic:  - Based on the start direction, the respective function for encoding is called
        - When in the service zone, west encoding function is called
        - When in the patient zone, east encoding function is called
        - Buzzer command (5) is appended to the list at the end

Example Call: encode_path(master_path, rot)

'''

def encode_path(path, rotation):
    
    length = len(path)

    # Start position encoding
    if rotation is 1:
        ret = encode_north(path[0])
        
    elif rotation is 2:
        ret = encode_west(path[0])
        
    elif rotation is 3:
        ret = encode_south(path[0])
        
    elif rotation is 4:
        ret = encode_east(path[0])
    
    motion.extend((path[0][-2],path[0][-1]))

    # Encoding rest of the path
    for i in range(1,length):
        if length is 2:
            ret = encode_west(path[i])
        elif 1<=i<=2:
            ret = encode_west(path[i])
        elif 3<=i<=4:
            ret = encode_east(path[i])
        elif i is 5:
            ret = encode_west(path[i])
        motion.extend((path[i][-2],path[i][-1]))
        
    motion.append(5)

'''

Function Name: simplify_encoding

Input:  None

Output: The encoding is simplified and repeating commands such
        as the forward/backward commands are added up to travel at once
        instead of grid by grid movement

Logic:  - Repeating commands are combined to one command (forward/backward)
          Eg: [1,1,1,1] is simplified as [1,[104]]

        - Certian logical errors in the encoding is fixed
        - Feedback commands are added for longer lengths
        
        NOTE: PLEASE REFER TO THE ENCODED COMMANDS LIST PDF FILE FOR
              THE COMPLETE LIST OF COMMANDS
              
Example Call: simplify_encoding()

'''

def simplify_encoding(rot):

    global motion, total_turns, direction_dict
    temp_list = []
    
    for k, g in groupby(motion):
        repetition = sum(1 for var in g)

        # Combine forward commands
        if k is 1:
            if repetition > 8:
                rem = repetition - 8
                if rem>=2:
                    temp_list.extend((1,[108],100,1,[rem+100],99)) # 99 -> Feedback command
                else:
                    temp_list.extend((1,[repetition+100],99))
            else:
                temp_list.extend((1,[repetition+100],99))

        # Combine backward commands     
        elif k is 3:
            if repetition > 8:
                rem = repetition - 8
                temp_list.extend((3,[208],100,3,[rem+200],99))
            else:
                temp_list.extend((3,[repetition+200],99))
        elif k is 2 or k is 4:
            total_turns += 1
            temp_list.append(k)
        else:
            temp_list.append(k)

    print '\nSimplified Encoding:',temp_list

    # Pick-up/delivery feedback removal
    break_flag = False
    i = 0
    while True:
        if temp_list[i+1] is 5 or temp_list[i+1] is 6:
            break_flag = True
        elif temp_list[i] is 99:
            if 10<=temp_list[i+1]<=13:
                del temp_list[i]
        i += 1
        if break_flag:
            break
    
    # U-turn logic correction
    u_flag = False
    for i in range(0,len(temp_list)):
            try:
                if temp_list[i] is 3:
                    if temp_list[i+4] is 3:
                        u_flag = True
                        temp_list[i+4] = 1
                        [rep] = temp_list[i+5]
                        temp_list[i+5] = [rep-100]
                        if temp_list[i+6] is 100:
                            temp_list[i+7] = 1
                            [rep] = temp_list[i+8]
                            temp_list[i+8] = [rep-100]
                        
                    elif temp_list[i+6] is 3:
                        temp_list[i+6] = 1
                        [rep] = temp_list[i+7]
                        temp_list[i+7] = [rep-100]
                        if temp_list[i+8] is 100:
                            temp_list[i+9] = 1
                            [rep] = temp_list[i+10]
                            temp_list[i+10] = [rep-100]
            
                if u_flag:
                    if temp_list[i] is 2:
                        temp_list[i] = 4
                        u_flag = False
                    elif temp_list[i] is 4:
                        temp_list[i] = 2
                        u_flag = False
                    
            except IndexError:
                warnings.warn('\n\nENCODING U-TURN CORRECTION ERROR!')

    '''# Reducing position feedback
    i = 0
    direction = rot
    print '\nDirections:'
    print direction_dict[direction]
    while True:
        try:
            if temp_list[i] is 2:
                if 1<=direction<=3:
                    direction += 1
                elif direction is 4:
                    direction = 1
                print direction_dict[direction]

            elif temp_list[i] is 4:
                if 2<=direction<=4:
                    direction -= 1
                elif direction is 1:
                    direction = 4
                print direction_dict[direction]

            if (temp_list[i] is 99) and (direction is 1 or direction is 3):
                del temp_list[i]
          
            i += 1

        except IndexError:
            break'''
                
    motion = temp_list

############################## XBEE COMMUNICATION ##############################

def get_turns_list(path):

    global turns_list
    
    for i in path:
        direction = -1
        
        for j in range(1,len(i)):
            if i[j][0] is -1:
                break
            current_dir = 0 if i[j][0]-i[j-1][0] != 0 else 1
            if direction != -1:
                if current_dir != direction:
                    turns_list.append((i[j-1][0],i[j-1][1]))
            direction = current_dir

'''

Function Name: get_reset_angle

Input:  offset -> The offset angle detected in degrees

Output: Based on the offset angle, the command for the correction
        angle (reset) is computed and returned

        NOTE: PLEASE REFER TO THE ENCODED COMMANDS LIST PDF FILE FOR
              THE COMPLETE LIST OF COMMANDS

Logic:  - If offset is gretaer than 0, turn the robot left
        - If offset is lesser than 0, turn the robot right
        - The command is computed based on the magnitude of offset

Example Call: angle_command = get_reset_angle(true_degs)

'''

def get_reset_angle(offset):

    if offset>0: base = 30
    elif offset<0: base = 60
    reset = 30
    
    if abs(offset)<=6: reset = base+1
    
    elif 7<=abs(offset)<20:
        for i in range(6, 19, 2):
            if i<=abs(offset)<i+2:
                reset = ((i-2)/2)+base
                
    elif abs(offset)>=20:
        for i in range(20, 181, 20):
            if i<=abs(offset)<i+20:
                reset = int((i-18)/20)+base+8
                    
    return reset

def feedback_capture():

    global corners
    
    timer = 0
    while(1):
        ret, frame = cap.read()
        timer += 1 
        if timer is 10:  
            break        

    o1 = [10,130,220] # Orange sticker
    o2 = [25,200,255]
    p1 = [155,100,220] # Pink sticker
    p2 = [176,170,255]

    img = crop_arena(frame, corners)
    img1 = img.copy()
    #cv2.imshow('Feedback Capture',img)

    # Calculate angle 
    x1,y1,img2 = get_moments(img, o1, o2)
    x2,y2,img3 = get_moments(img, p1, p2)

    return x1,y1,x2,y2,img,img2
    
'''

Function Name: rotation_feedback

Input:  turns_index -> Feedback count number

Output: The offset angle is detected and the correction feedback is provoded

Logic:  - The image of the arena is captured
        - The colored stickers are masked and the direction is found out
        - The offset angle for that particular direction is found
        - The command for the correction angle is returned

Example Call: txData = chr(rotation_feedback(turns_count))

'''

def rotation_feedback(turns_index):

    x1,y1,x2,y2,img,img2 = feedback_capture()
    file_name = 'Rotation Feedback/Rotation-'+str(turns_index)+'.jpg'
    cv2.imwrite(file_name,img2)

    dy = y2-y1
    dx = x2-x1
    rads = math.atan2(dy,dx)
    degs = int(math.degrees(rads))

    print '\nFeedback Turn %d:'%turns_index
    print 'Angle Detected:',degs
    
    true_degs = None
    
    if (abs(degs) is 0) or (abs(degs) is 90) or (abs(degs) is 180) or (abs(dx)<1) or (abs(dy)<1):
        print 'Straight Angle'
        true_degs = 0
        angle_command = 30

    elif 40<abs(degs)<50 or 130<abs(degs)<140:
        print'Direction Unknown! (45 degrees error)'
        true_degs = 45
        angle_command = 30

    # Get the angle with respect to the direction   
    else:
        if 50 <= degs <= 130:
            print 'North'
            true_degs = degs-90 
            
        elif -130 <= degs <= -50:
            print 'South'
            true_degs = degs+90
            
        elif abs(degs) <= 40:
            print 'West'
            true_degs = degs

        elif abs(degs) >= 140:
            print 'East'
            if degs<0: true_degs = degs+180
            elif degs>0: true_degs = degs-180

        print 'Offset angle:',true_degs
        angle_command = get_reset_angle(true_degs)

    print 'Feedback command:',angle_command,'\n'
    
    return angle_command

def get_pos_command(offset):

    if offset>0: base = 130
    elif offset<0: base = 230
    reset = 60

    if 7<=abs(offset)<12: reset = base
    elif 12<=abs(offset)<16: reset = base+1
    elif 16<=abs(offset)<95:
        for i in range(16, 95, 4):
            if i<=abs(offset)<i+4:
                reset = ((i/4)-3)+base

    return reset 

def path_alignment(pos_index):

    path_offset = 0
    reset = 60
    flag = False
    
    x1,y1,x2,y2,img,img2 = feedback_capture()
    avg_x = int((x2+x1)/2)
    avg_y = int((y2+y1)/2)
    avg = (avg_x,avg_y)
    
    file_name = 'Position Feedback/Position-'+str(pos_index)+' '+str(avg)+'.jpg'
    cv2.imwrite(file_name,img2)

    print '\nFeedback Position:',pos_index
    
    if get_direction(img) is 2:
        print 'West'
        flag = True
        if 50 < avg_x < 190:
            path_offset = avg_x - 110
            print 'Path 1'
            flag = True
        elif 180 < avg_x < 320:
            path_offset = avg_x - 242
            print 'Path 2'
            flag = True
        elif 300 < avg_x < 440:
            path_offset = avg_x - 367
            print 'Path 3'
            flag = True
        else: print 'Not in range'

    elif get_direction(img) is 4:
        print 'East'
        if 50 < avg_x < 190:
            path_offset = 125 - avg_x
            print 'Path 1'
            flag = True
        elif 180 < avg_x < 320:
            path_offset = 253 - avg_x
            print 'Path 2'
            flag = True
        elif 300 < avg_x < 440:
            path_offset = 388 - avg_x
            print 'Path 3'
            flag = True
        else: print 'Not in range'

    elif get_direction(img) is 1:
        print 'North'
        if 0 < avg_y < 85 and 0 < avg_x < 320:
            path_offset = avg_y - 45
            print 'Top Path'
            flag = True
        elif 0 < avg_y < 50 and avg_x > 320:
            path_offset = avg_y - 45
            print 'Top Path 3'
            flag = True
        else: print 'Not in range'

    elif get_direction(img) is 3:
        print 'South'
        if 350 < avg_y < 480:
            path_offset = 435 - avg_y
            print 'Bottom Path'
            flag = True
        else: print 'Not in range'

    else:
        print 'Not in direction'

    print 'Current position:',avg
    if flag:
        if abs(path_offset)>8:
            reset = get_pos_command(int(path_offset*2.4)) # Range: 8mm -> 92mm
            
        print 'Offset:',path_offset,'px',int(path_offset*2.4),'mm'
        print 'Feedback command:',reset

    print '\n'

    return reset

'''

Function Name: xbee_communication

Input:  None

Output: The encoded commands are sent to the robot one by one
        and feedback is done at every turn of the robot

Logic:  - Serial communication is used to send the command one by one
        - The robot and the computer are first synced before communication
        - When a data of '1' is recieved from the robot, the next command is sent
        - When a data of '2' is recieved from the robot, feedback is processed (after every turn)
        - After the buzzer command(5) is sent, communication is stopped

Example Call: xbee_communication()

'''

def xbee_communication():

    global motion, success_flag
    end_flag = False

    xbee = serial.Serial('COM7', 9600, timeout = None)
    print '\nXBee Transmitter found at:',xbee.name

    array_index = 0
    turns_count = 1
    pos_count = 1
    repetition = 0

    print 'Searching for robot... (Ctrl+C to abort communication at any time)\n'

   # Sync robot and computer
    while True:
        if (xbee.inWaiting()>0):            
            start_flag = xbee.read()
            if start_flag is '0':
                print 'Robot found!'
                xbee.write(chr(255))
                break
        
    print '*Communication Initiated*\n'
    while True:

        # Start live communication
        try:
            command = motion[array_index]
            txData = chr(command)

            if (xbee.inWaiting()>0):            
                rxData = xbee.read()
                #print 'Data In:',rxData

                # Send next command
                if rxData is '1':
                    
                    if command is 1 or command is 3:
                        [repetition] = motion[array_index+1]
                        txData = chr(repetition)
                        
                        xbee.write(txData)
                        print 'Command Sent:',ord(txData)

                        array_index += 2
                        
                    else:
                        xbee.write(txData)
                        print 'Command Sent:',ord(txData)
                        
                        array_index += 1

                # Feedback process
                elif rxData is '2':
            
                    txData = chr(rotation_feedback(turns_count))
                    xbee.write(txData)
                    print 'Command Sent:',ord(txData)
                    time.sleep(0.3)

                    turns_count += 1

                elif rxData is '3':
            
                    txData = chr(path_alignment(pos_count))
                    xbee.write(txData)
                    print 'Command Sent:',ord(txData)
                    time.sleep(0.3)

                    pos_count += 1
                    
                # End after buzzer command is sent
                if command is 5 or command is 6:
                    end_flag = True
                    success_flag = True
                    break

            if end_flag: 
                break
        
        except KeyboardInterrupt:
            break
            
    xbee.close()

def get_run_time():

    global motion
    speed = 160 # Speed in mm/sec
    run_time = 5

    i = 0
    while True:

        command = motion[i]

        if command is 5 or command is 6:
            break
        
        elif command is 1:
            [repetition] = motion[i+1]
            run_time += ((repetition-100)*48)/speed + 0.1
            i += 1
        elif command is 3:
            [repetition] = motion[i+1]
            run_time += ((repetition-200)*48)/speed + 0.1
            i += 1
            
        elif command is 135 or command is 235:
            run_time += 24/speed + 0.1
        elif command is 2 or command is 4:
            run_time += 0.4 + 0.5 + 0.5 + 0.1 + 0.1 + 1
        elif command is 76:
            run_time += 0.4 + 0.5 + 1 + 0.1 + 0.1 + 1
        elif command is 100:
            run_time += 0.4 + 0.5 + 0.1 + 0.1 + 1

        i += 1

    return run_time
    
    
#------------------------------------------------------------------------------#
################################# MAIN PROGRAM #################################
#------------------------------------------------------------------------------#


'''

Function Name: Main Program

Input:  Captures image of the arena from the webcam

Output: Delivers the provision using XBee serial communication
        Show the webcam input image and the arena mapping done with the planned path

Logic:  - Captures the image of the arena
        - Maps the arena and detects provisions and obstacles
        - Plans the shortest possible path for all deliveries
        - Encodes the path into commands for communication
        - Sends command using serial communication via XBee
        - Shows the webcam input image and arena map with the planned path and robot position
        - Total run time is timed
        - Calculates the score of the run

Example Call: Called automatically by the Operating System

'''


########################### CAPTURE ARENA FOR MAPPING ##########################

master_mode = 0

# 10 frame delay after starting the capture
if master_mode is 1:
    cap = cv2.VideoCapture(1)
    timer = 0
    while(1):
        ret, arena_input = cap.read()
        timer += 1 
        if timer is 10:  
            break

    # Correct perspective      
    arena, corners = get_perspective_image(arena_input)
    cv2.imshow('Input',arena_input)
    arena_crop = arena.copy()

elif master_mode is 0:
    arena_input = cv2.imread('Test Image-01.jpg') ## INPUT IMAGE ##
    arena = get_perspective_image(arena_input)
    cv2.waitKey(0)
    
else: arena = cv2.imread('Testing/Arena Testing.jpg')

img = arena.copy()

################################# ARENA MAPPING ################################

# Arena mapping
arena_grid = arena_mapping(arena)

# Get the robot position and orientation
pos, pos_cord, rot = get_position(img, arena_grid)
if pos is 0:
    warnings.warn('\n\nRobot Not Found!\nStart Position: 2')
else:
    print '\nRobot Position:',pos
    print 'Robot Direction:',direction_dict[rot]

org = arena_grid
image = org.copy()

# Match the provisions
get_marker_pos(org, 0)
print '\nPatient Requests:',request_count
print 'Provision Deliveries:',request_matched

################################# PATH PLANNING ################################

path_truth = False # Truth value if a path is computed without errors

if request_matched is 0:
    print '\nNo Patient Present'

# Delivery of 1 provision    
if request_matched is 1:        
    master_len, master_path, master_index, master_turns = get_min_path1(org, pos_cord)
    path_truth = True

# Delivery of 2 provisions
if request_matched is 2:
    get_all_paths(org, pos_cord, 2)
    master_len, master_path, master_index, master_turns = get_min_path2(index_2)
    path_truth = True

# Delivery of 3 provisions         
if request_matched is 3:
    get_all_paths(org, pos_cord, 3)
    master_len, master_path, master_index, master_turns = get_min_path3(index_3)
    path_truth = True

################################# PATH ENCODING ################################

if(path_truth):
    
    # Encode Path
    for i in master_path:
        i.append((-1,-1))

    # Encode LED commands
    led_color(master_path,org)
    encode_path(master_path,rot)

    # Simplify encoding
    simplify_encoding(rot)

    get_turns_list(master_path)
    pos_points = [(7,4),(7,13),(7,21),(13,4),(13,10),(13,15),(13,21),(19,6),
                  (19,13),(19,21),(7,10),(7,15),(19,4),(19,10),(19,15)]
    pos_feedback = list(set(pos_points).intersection(turns_list))
    #print '\n',pos_feedback

    # Print master path details
    print '\nRoute Length =',master_len
    print 'Route Turns =',total_turns
    #print 'Route Index =',master_index
    #print '\nRoute Path:'
    #for i in master_path: print i,'\n'
    print '\nEncoded Commands:',motion

############################### DRAW ALL ELEMENTS ##############################

    # Draw Path
    col_flag = 1
    for i in master_path:
        ex,ey = i[-4]

        draw_path(col_flag, image, i, ey, ex)
        draw_path(col_flag, arena, i, ey, ex)
        col_flag += 1
        del i[0]

# Draw grids
arena_grid = draw_grid(arena_grid, 20, 20, 140)
image = draw_grid(image, 20, 20, 160)

# Draw firebird
if pos is not 0: draw_firebird(0, pos, rot)

# Show Images
cv2.imshow('Arena Live',arena)
cv2.imshow('Master Path',image)

################################# COMMUNICATION ################################

print '\nPath Planning Successful!\nPress any key to start communication'
cv2.waitKey(0)

# Start XBee communication
if master_mode is 1:
    xbee_communication()
    
if success_flag:
    print '\n*All Deliveries Complete*'
else:
    print'\nDelivery Failed!'

######################## STOP TIMER / CLOSE ALL WINDOWS ########################

# Stop timer
stop_time = timeit.default_timer()
total_time = int(stop_time - start_time)
print '\n*Timer Stopped*'
  
# Close all windows
print '\nPress any key to calculate score'
cv2.waitKey(0)
if master_mode is 1: cap.release()
cv2.destroyAllWindows()

# Calculte time taken
EST_time = int(get_run_time())
EST_min = int(EST_time/60)
EST_sec = int(EST_time%60)

time_min = int(total_time/60)
time_sec = int(total_time%60)

print("\nEstimated Run Time: %d min %d sec" %(EST_min, EST_sec))
print("Total Run Time: %d min %d sec" %(time_min, time_sec))

################################ CALCULATE SCORE ###############################

# Calculate the score
if master_mode is 1 and success_flag:
    score_b = 0
    score_c = int(raw_input('Provisions Delivered: '))
    score_w = int(raw_input('Provisions Delivered Incorrectly: '))
    score_p = int(raw_input('Obstacles Displaced: '))

    if (score_w + score_p) is 0 and total_time<600 and score_c is not 0:
        score_b = 100
        
    total_score = ((600-total_time)+(score_c*100)-(score_w*70)+score_b-(score_p*30))
    print '\nTOTAL SCORE: %d (BONUS: %d)'%(total_score, score_b)

    efficiency = float(EST_time)/float(total_time)
    print 'Efficiency:',efficiency*100.0,'%'

################################## PROGRAM ENDS ################################
   
print '\n\nThank you for using Caretaker #1673'
