#! /usr/bin/env python

import random
import rospy
import cv2 as cv
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

#Only road lines are white, other features are black
BINARY_THRESH = 250

#Canny edge detector parameters
CANNY_H = 200
CANNY_L = 25

#Probabilistic Hough Transform parameters
HOUGH_THRESH = 75
MIN_LINE = 20
MAX_GAP = 75

#Direction Thesholds
#Forward Threshold must be high >=3 so that turns are executed completely.
FWD_THRESH = 3
LEFT_THRESH = 1
RIGHT_THRESH = 1
#FWD_THRESH = 2
#LEFT_THRESH = 2
#RIGHT_THRESH = 2
MID_THRESH = 1

#Codes
IDK = -1
FWD = 0
LEFT = 1
RIGHT = 2
LEFTRIGHT = 3

#PID parameters
INTEGRAL_WINDUP = 0.5
previous_error = 0
integral_response = 0

#Misc Global Variables
lock_movement = False
movement = FWD
move = Twist()
n_turns = 0

def edge_detector(cv_img_binary, blank_image, frame_width, draw = True):

    #Use Probabilistic Hough Transform to find edges
    edges = cv.Canny(cv_img_binary, CANNY_L, CANNY_H, apertureSize = 3)
    lines = cv.HoughLinesP(edges, 1, np.pi/180, HOUGH_THRESH,\
                             minLineLength=MIN_LINE,maxLineGap=MAX_GAP)
    
    line_coords = []
    if lines is not None:
        for line in lines:
            line_coords.append(line[0])

    #Convert set of lines into information about the vehicle position
    #Raw information is 'points' indicating a certain direction is available
    gradients = []
    fwd_pt = 0             #forward line is detected
    left_pt = 0            #line on the left is detected
    right_pt = 0           #line on the right is detected
    middle_pt = 0          #line in the middle is detected
    idk_pt = 0             #unsure
    for i in range(len(line_coords)):
        x1, y1, x2, y2 = line_coords[i]
        if x2 - x1 != 0:
            gradient = abs(((float) (y2 - y1))/((float) (x2 - x1)))
        else:
            gradient = 1 #definitely vertical

        gradients.append(gradient)
        if gradient > 0.5:
            if draw == True:
                cv.line(blank_image, (x1,y1), (x2,y2),\
                         (random.randint(0,255),255,random.randint(0,255)),2)
            fwd_pt = fwd_pt + 1
        elif gradient < 0.3:
            if draw == True:
                cv.line(blank_image, (x1,y1), (x2,y2), (255,0,0),2)
            
            #if x1 or x2 is on the left half, turn is on the left
            if max(x1,x2) < (frame_width)/2:
                left_pt = left_pt + 1
            #if x1 or x2 is on the right half, turn is on the right
            elif min(x1,x2) > (frame_width)/2:
                right_pt = right_pt + 1
            #if one coordinate is on the left and another on the right
            else:
                middle_pt = middle_pt + 1
        else:
            if draw == True:
                cv.line(blank_image, (x1,y1), (x2,y2), (0,0,255),2)
            idk_pt = idk_pt + 1

    #print("f:{},l:{},r:{},m:{},idk:{}".format(fwd_pt,left_pt,right_pt,middle_pt,idk_pt))

    return ((fwd_pt, left_pt, right_pt, middle_pt, idk_pt), blank_image) 

def determine_motion(edge_points):
 
    #Determine what movements are currently available 
    forward = False  
    left = False
    right = False
    if edge_points[0] >= FWD_THRESH:
        forward = True
    if edge_points[1] >= LEFT_THRESH:
        left = True    
    if edge_points[2] >= RIGHT_THRESH:
        right = True
    if edge_points[3] >= MID_THRESH:
        forward = False
     
    #   Forward     Left        Right
    #   T           T           T      entering a lr junction or lrf junction
    #   T           T           F      entering a l junction or lf junction
    #   T           F           T      entering a r junction or rf junction 
    #   T           F           F      straight road
    #   F           F           F      confused, this is not a possible state
    #   F           F           T      the forward road is blocked
    #   F           T           F      the forward road is blocked
    #   F           T           T      the forward road is blocked
   
    return (forward, left, right)

def driver(data):

    global lock_movement
    global movement
    global n_turns
    global move

    #Apply filters to raw image
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    cv_img = cv_img[-100:, :, :]
    cv_img_gray = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)
    kernel = np.ones((5,5), np.float32)/25
    cv_img_gray = cv.filter2D(cv_img_gray,-1,kernel)
    retval, cv_img_binary = cv.threshold(cv_img_gray, BINARY_THRESH, 255, cv.THRESH_BINARY)

    cv_redmask = cv.inRange(cv_img, np.array([0,0,200]), np.array([0,0,255]))

    frame_height, frame_width = cv_img_gray.shape
    blank_image = np.zeros((frame_height, frame_width, 3), np.uint8)
    
    #Obtain edges to determine current position on board
    edge_points, blank_image = edge_detector(cv_img_binary, blank_image, frame_width)

    #Obtain possible movements
    forward, left, right = determine_motion(edge_points)
    
    #if the vehicle is not in the middle of a turn, determine new movement
    if lock_movement == False:
 
        if forward == True and left == False and right == False:
            # The only way is forward.
            movement = FWD
            print("F")

        elif forward == True and left == True and right == True:
            # Open a decision to turn left, right or continue forwards
            movement = RIGHT 
            print("Left, Right, Forward Positions Available")
            print("Begin RIGHT movement")
    
        elif forward == True and left == True and right == False:
            # Open a decision to turn left or continue forwards
            movement = LEFT
            print("Left, Forward Positions Available")
            print("Begin LEFT movement")

        elif forward == True and left == False and right == True:
            # Open a decision to turn right or continue forwards
            movement = RIGHT
            print("Right, Forward positions available")
            print("Begin RIGHT movement")

        elif forward == False:
            # Stop the car. Mission failed...for now
            movement = IDK
            print("ERROR: Unknown Position.")
            print("Forward: {} Left: {} Right: {}".format(forward,left,right))

    #if the vehicle is in the middle of a turn 
    else:
       
        #if the vehicle has completed a turn, unlock movement 
        if forward == True and left == False and right == False:


            #As the Hough Transform is probabilistic, repeat the Hough Transform 5x
            # to average out the random errors.
            
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.angular.z = move.angular.z * 0.5
            pub_vel.publish(move)

            fwd_total = 0
            idk_total = 0
            mid_total = 0
            for i in range(5):

                edge_points, retval = edge_detector(cv_img_binary, blank_image,\
                                                     frame_width, draw = False)
                fwd_total = fwd_total + edge_points[0] 
                mid_total = mid_total + edge_points[3]
                idk_total = idk_total + edge_points[4]

            redmask_sum = cv_redmask.sum()
            
            
            if fwd_total >= (20) and idk_total == 0 and mid_total == 0 and redmask_sum < 500000:
                print("Turn {} Complete".format(n_turns))
                print("fwd_total: {}, redmask_sum: {}".format(fwd_total, redmask_sum))
                cv.imshow("turn_completed_{}_edges".format(n_turns), blank_image)
                cv.imshow("turn_completed_{}_color".format(n_turns), cv_img)
                movement = FWD
                lock_movement = False
                n_turns = n_turns + 1
            else:
                print("random: fwd: {}, red: {}, idk: {} mid: {}".format(\
                        fwd_total, redmask_sum, idk_total, mid_total))

    if movement == FWD:

        #sum the thresholded image to reduce random error
        row = []
        for i in range(frame_width):
            vert = 0
            for j in range(frame_height):
                vert = vert + cv_img_binary[j][i]
            
            row.append(vert)
        
        #find the lower bound
        lower = frame_width/2
        c = frame_width/2
        lower_found = False
        while(c > -1):
            
            if row[c] > 10*255:
                lower = c
                lower_found = True
                break
            c = c - 1
        
        if lower_found == False:
            lower = 0
        
        #find the upper bound
        upper = frame_width/2
        c = frame_width/2
        upper_found = False
        while (c < frame_width - 1):
            
            if row[c] > 10*255:
                upper = c
                upper_found = True
                break
            c = c + 1
        
        if upper_found == False:
            upper = frame_width - 2
 
        cv.circle(blank_image, (upper, frame_height - 10), 10, (0,0,255), -1)
        cv.circle(blank_image, (lower, frame_height - 10), 10, (0,255,0), -1)
        
        error = frame_width/2 -  int((upper + lower)/2)
        
        #print("Upper: {} Lower: {} Err: {}".format(upper, lower, error))
        
        #Do PID
        move = Twist()
        move.linear.x = 0.20
        move.linear.y = 0.0
            
        kp = 3.0/frame_width
        ki = 0
        kd = 10.0/frame_width

        #if error > 0, proportional_response > 0
        #if error < 0, proportional_response < 0
        proportional_response = kp * error
        global integral_response
        global previous_error
        integral_response = integral_response + ki * error
        if integral_response > INTEGRAL_WINDUP:
            integral_response = INTEGRAL_WINDUP
        elif integral_response < -INTEGRAL_WINDUP:
            integral_response = -INTEGRAL_WINDUP
        differential_response = kd * (error - previous_error)
        
        move.angular.z = proportional_response + differential_response + integral_response
            
        pub_vel.publish(move)
            
        previous_error = error
        
    elif movement == LEFT:
        #Do a preprogrammed sharp left turn

        #If it is the first movement to the left,
        if lock_movement == False:
            
            print("Entering Junction: ")

            cv.imshow("before entering {}".format(n_turns), cv_img)

            #Fully enter the junction by waiting x seconds
            time.sleep(1.5)

            #Stop the car
            move = Twist()
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.angular.z = 0.0

            pub_vel.publish(move)

            print("Junction Entered. Beginning LEFT TURN")

            #lock the vehicle state
            lock_movement = True

        #If the vehicle is in the middle of a turn
        else:

            #Send appropriate turning messages
            move = Twist()
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.angular.z = 0.35

            pub_vel.publish(move)
         
    elif movement == RIGHT:
        #Do a preprogammed sharp right turn

        #If it is the first movement to the right,
        if lock_movement == False:

            print("Entering Junction: ")
            
            cv.imshow("before entering {}".format(n_turns), cv_img)

            #Fully enter the junction by waiting x seconds
            time.sleep(1.5)
            
            #Stop the car 
            move = Twist()
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.angular.z = 0.0

            pub_vel.publish(move)

            print("Junction Entered. Beginning RIGHT TURN")

            #lock the vehicle state
            lock_movement = True

        #If the vehicle is in the middle of a turn
        else:

            #Send appropriate turning messages
            move = Twist()
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.angular.z = -0.35

            pub_vel.publish(move)

    elif movement == IDK:
        #shutdown

        move = Twist()
        move.linear.x = 0.0
        move.linear.y = 0.0
        move.angular.z = 0.0
        pub_vel.publish(move)
        
        print("Vehicle has been stopped.")
        cv.imshow('error', blank_image)
        cv.waitKey(1)
        time.sleep(50)

    cv.imshow('img', blank_image)
    cv.waitKey(1)


def observer(data):

    global lock_movement
    global movement

    #Apply filters to raw image
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    cv_img = cv_img[-100:, :, :]
    cv_img_gray = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)
    kernel = np.ones((5,5), np.float32)/25
    cv_img_gray = cv.filter2D(cv_img_gray,-1,kernel)
 
    retval, cv_img_binary = cv.threshold(cv_img_gray, BINARY_THRESH, 255, cv.THRESH_BINARY)

    cv_redmask = cv.inRange(cv_img, np.array([0,0,200]), np.array([0,0,255]))

    frame_height, frame_width = cv_img_gray.shape
    blank_image = np.zeros((frame_height, frame_width, 3), np.uint8)
    
    #Obtain edges to determine current position on board
    edge_points, blank_image = edge_detector(cv_img_binary, blank_image, frame_width)

    #Obtain possible movements
    forward, left, right = determine_motion(edge_points)

    #print("forward: {} left: {} right: {}".format(forward,left,right))

    #print("fwd_pt: {} left_pt: {} right_pt: {} mid_pt: {}".format(\
    #        edge_points[0], edge_points[1], edge_points[2], edge_points[3]))

    cv.imshow("observer", blank_image)
    cv.waitKey(1)

#Initialize the node
rospy.init_node('driver')
rospy.loginfo('driver node started')

#Publish a movement topic
pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

time.sleep(10)

#Initialize the movement
move = Twist()
move.linear.x = 0.35
move.linear.y = 0.0
move.linear.z = 0.0
move.angular.x = 0.0
move.angular.y = 0.0
move.angular.z = 0.3

pub_vel.publish(move)

time.sleep(0.3)

move.linear.x = 0.00
move.linear.y = 0.0
move.linear.z = 0.0
move.angular.x = 0.0
move.angular.y = 0.0
move.angular.z = 0.0

pub_vel.publish(move)

#Subscribe to camera
sub_image = rospy.Subscriber("/rrbot/camera1/image_raw", Image, observer)

#Bridge ros and opencv
bridge = CvBridge()

rospy.loginfo('robot movement initialized')

#TEMP: keep running
rospy.spin()
