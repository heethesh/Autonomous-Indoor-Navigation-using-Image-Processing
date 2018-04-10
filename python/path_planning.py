################################################################################
################################################################################

                              ## IMPORTANT NOTE ##

# 1. Please set appropiate mode of operation towards the END OF THIS SCRIPT

# Mode = 0 --> For single image and specify the filename
# Mode = 1 --> For multiple images and specify the range


################################################################################
################################################################################

                               ## TEAM DETAILS ##
'''
e-Yantra 2014
Task 2 - Path Planning (Practice)

eYRC+
Team ID: eYRC+#1673
File name: eYRC+#1673_PathPlanning.py
Version: 1.0.0
Author: Heethesh Vhavle
Date: December 15th, 2014
'''

                            ## PROGRAM DESCRIPTION ##
'''
This is a program that calculates the shortest path from the start and end point
using A* algorithm.

An image is read and processed to get the positions of the start and end points
and also detect the obstacles.

The length and the coordinates of the shortest path detected is displayed along
with the path drawn on the output image.
'''

################################################################################
################################################################################


# Importing Modules
import numpy as np
import cv2
import math
import heapq

img = cv2.imread('test_images/test_image1.png')


#============================ HELPER FUNCTIONS HERE ===========================#


## Global Variables ##
h,w,c = img.shape

count = 1
imn = 'img'+str(count)

#Cordinates for Start/End nodes
start_x = 0
start_y = 0
end_x = 0
end_y = 0

#List of positions of the obstacles
wall_map = []

############################ GET START/END FUNCTION ############################

## Function which processes the image and returns the coordinates of the
## start and end point
## Also creates a list with the coordinates of the obstacles present

def get_start_end(img2):

    y = 20
    for i in range(0,10):
        x = 20
        if(y<h):
            for j in range (0,10):
                if(x<w):                    
                    #Detecting the colors
                    roi = img2.copy()
                    roi = roi[y-5:y+5, x-5:x+5, :]
                    hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
                    hue,sat,val,ret = cv2.mean(hsv)
                    #If red, set the position for start node
                    if(hue==0 and sat==255 and val==255):
                        start_x = i
                        start_y = j
                    #If green, set the position for end node
                    elif(hue==60 and sat==255 and val==255):
                        end_x = i
                        end_y = j
                    #If black, add the obstacle position into the list
                    elif (val==0):
                        wall_map.append((i,j))
                
                    x = x+40
        y = y+40

    return (start_x,start_y,end_x,end_y)

################################## NODE CLASS ##################################

class Node(object):

    def __init__(node, x, y, space):

        #Coordinates of the current node
        node.x = x
        node.y = y
        #Black spce flag and parent node
        node.space = space
        node.parent = None
        #f, g, h values for A* algorithm
        node.f = 0
        node.g = 0
        node.h = 0
        
############################## A* ALGORITHM CLASS ##############################

class path_algorithm(object):

############################## INITIALISE FUNCTION #############################

## Function to intialise the algorithm parameters
        
    def __init__(a):

        #Open list and closed list
        a.open_list  = []
        a.close_list = set()
        #Transform open list into a heap (node with lowest f value at top)
        heapq.heapify(a.open_list)
        #List of nodes which are free white spaces
        a.nodes = []
        #Number of rows and columns of the grid
        a.rows = 10
        a.cols = 10

############################ NODE POSITION FUNCTION ############################

## Function which returns a node based on the position
    
    def get_pos(a, x, y):

        pos = a.nodes[(x*a.rows)+y]
        return pos

############################### GRID MAP FUNCTION ##############################

## Function to create a list which stores the position of white spaces and
## obstacles and to initialise the start and end node

    def grid_map(a, sx, sy, ex, ey):

        for i in range(0,a.rows):
            for j in range(0,a.cols):

                if (i,j) in wall_map:
                    space = 0
                else:
                    space = 1
                a.nodes.append(Node(i,j,space))
                
        #Set the positions for start and end node
        a.start = a.get_pos(sx,sy)
        a.end   = a.get_pos(ex,ey)

############################ ADJACENT NODES FUNCTION ###########################

## Function which returns a list of adjacent nodes around the parent node (clockwise)

    def get_adjacent(a, node):

        adj_nodes = []

        if (node.x < a.cols-1):
            adj_nodes.append(a.get_pos(node.x+1, node.y))
        if (node.y > 0):
            adj_nodes.append(a.get_pos(node.x, node.y-1))
        if (node.x > 0):
            adj_nodes.append(a.get_pos(node.x-1, node.y))
        if (node.y < a.rows-1):
            adj_nodes.append(a.get_pos(node.x, node.y+1))        
        
        return adj_nodes

######################### GET HEURISTIC VALUE FUNCTION #########################

## Function which calcultes the heuristic value and returns it
    
    def get_h(a, node):

        #H factor value
        h_factor = -20
        dx = abs(node.x - a.end.x)
        dy = abs(node.y - a.end.y)
        h  = h_factor * (dx + dy)
        
        return h

############################ UPDATE VALUE FUNCTION #############################

## Function to update the values of the selected adjacent node
    
    def update_values(a, adj, node):

        adj.g = node.g + 40
        adj.h = a.get_h(adj)
        adj.f = adj.g + adj.h
        adj.parent = node

########################### ROUTE PATH LIST FUNCTION ###########################

## Function which adds the final coordinates of the path in the route_path list
## in the reverse order (excluding the start and end points)

    def path_list(a, route_path):

        node = a.end
        # Back trace the path until the start node is encountered
        while(node.parent is not a.start):
            node = node.parent
            route_path.append((node.y+1,node.x+1))

############################# PATH DETECT FUNCTION #############################

## Function to find the shortest path usinf A* algorithm
     
    def path_detect(a, route_path):
        
        #Add start node to open heap queue
        heapq.heappush(a.open_list, (a.start.f, a.start))

        while(len(a.open_list)):
            
            #Pop node from open heap queue
            f,node = heapq.heappop(a.open_list)
            a.close_list.add(node)
            
            #If it is the end node, then append the path list
            if node is a.end:
                a.path_list(route_path)
                break
            
            #To get a list of adjacent nodes
            adj_list = a.get_adjacent(node)
            
            #Get adjacent nodes and compare the f values
            for adj in adj_list:
                if(adj.space and (adj not in a.close_list)):
                    if((adj.f, adj) in a.open_list):
                        if(adj.g > (node.g + 40)):
                            a.update_values(adj, node)
                    else:
                        a.update_values(adj, node)
                        heapq.heappush(a.open_list, (adj.f, adj))

############################## DRAW PATH FUNCTION ##############################

## Function to draw the shortest path and display the final output image
## Also calculates the total path length and returns it
                        
def draw_path(path, ex, ey):

    #Path length
    length = len(path)-1

    #Draw the colored path
    imx = imn.copy()
    for i in range(0,length):
        y1,x1 = path[i]
        y2,x2 = path[i+1]
        if(i>0):
            cl = (int)((255/(length)*i)) 
            cv2.rectangle(imx,((y1*40)-38,(x1*40)-38),((y1*40)-2,(x1*40)-2),(0,cl,255-cl),-1) 
    opacity = 0.2
    cv2.addWeighted(imx, opacity, imn, 1 - opacity, 0, imn)

    #Draw the path lines
    for i in range(0,length):
        y1,x1 = path[i]
        y2,x2 = path[i+1]   
        cv2.line(imn,((y1*40)-20,(x1*40)-20),((y2*40)-20,(x2*40)-20),(255,0,0),3)
    
    #Mark the end node
    cv2.circle(imn,((ey*40)-20,(ex*40)-20),8,(255,0,0),-1)
    
    #Display the output image
    name = 'Final Output '+str(count)
    cv2.imshow(name,imn)

    return length


#========================== HELPER FUNCTIONS HERE END =========================#


## Function which take an image as its arguement, calculates the shortest path
## and returns the path length and a list of the route path

def play(img):

    #Reseting the lists
    route_path = []
    del wall_map[:]

    #Function to get the coordinates for start and end points
    start_x,start_y,end_x,end_y = get_start_end(imn)

    #Adding the end point to the list first (reverse order)
    route_path.append((end_y+1, end_x+1))

    #Creating object 'a' of the algorithm class
    a = path_algorithm()

    #Function to create a grid map (space = 1(white space), space = 0(obstacle))
    a.grid_map(start_x, start_y, end_x, end_y)

    #Function to detect the shortest path
    a.path_detect(route_path)

    #Adding the start point to the list at the last (reverse order) 
    route_path.append((start_y+1, start_x+1))

    #Reversing the route_path list
    route_path.reverse()

    #Function to calculate the path length and to draw the path on the image
    route_length = draw_path(route_path, end_x+1, end_y+1)

    #Removing the end point as per the required output
    del route_path[0]

    #Returns the path length and list of the route path
    return route_length, route_path


#============================ PLEASE SET MODE HERE ============================#

'''
Please Enter the Mode here:

MODE = 0 --> To check the output for a Single Image
             And also enter the required file name below

MODE = 1 --> To check the output for Multiple Images in a range
             And also set the range value in the 'for loop' below
'''
#SET THE MODE HERE
mode = 1

#FOR SINGLE IMAGE (mode = 0), PLEASE ENTER FILENAME BELOW:
image_name = 'test_images/test_image1.png'

#FOR MULTIPLE IMAGES, PLEASE ENTER THE RANGE:
range_start = 1
range_end   = 5  ##Eg: Displays images from 1 to 5 (including 5)


#============================== CHECK OUTPUT HERE =============================#


if __name__ == "__main__":
    
    #Checking output for single image
    if (mode==0):
        img1 = cv2.imread(image_name)    ##Enter the File Name above
        imn  = img1.copy()
        route_length, route_path = play(imn)
        print 'Route Length =', route_length
        print 'Route Path =', route_path
        
    #Checking output for all images
    elif (mode==1):
        route_length_list = []
        route_path_list   = []
        for file_number in range(range_start, range_end+1): ##Enter the Range above
            file_name = 'test_images/test_image'+str(file_number)+'.png'
            pic = cv2.imread(file_name)
            imn = pic.copy()
            route_length, route_path = play(imn)
            count = count+1
            route_length_list.append(route_length)
            route_path_list.append(route_path)
        print 'Route Lenth List: ',route_length_list
        print '\nRoute Path List: '
        for i in range(len(route_path_list)):
            print '\nImage',i+1,':\n',route_path_list[i]

cv2.waitKey(0)
cv2.destroyAllWindows()
