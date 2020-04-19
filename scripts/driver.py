#! /usr/bin/env python

# ENPH 353 final project: Self-driving vehicle with license-plate reading
# driver.py: script which handles the vehicle driving
# Authors:
#   Miles Justice
#   David Widjaja 18950063

import random
import time
import sys

import cv2 as cv
import numpy as np

#ROS-specific libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

#local modules
import sift_based_car_finder
import navigator
#import hough_based_car_finder

#Robot Camera Parameters
IMG_WIDTH = 640
IMG_HEIGHT = 100        #Image Height has to be less than 480
 
#Only road lines are white, other features are black
BINARY_THRESH = 250

#Canny edge detector parameters
CANNY_H = 200
CANNY_L = 25

#Probabilistic Hough Transform parameters
#100 gives too little lines
#85 gives ~ the right number of lines
#75 gives too many lines
HOUGH_THRESH_P = 85
MIN_LINE = 25
MAX_GAP = 75

#Direction Thesholds
FWD_THRESH_P = 3        #to confirm a way forward, 3 vertical lines are needed
LEFT_THRESH_P = 1
RIGHT_THRESH_P = 1
MID_THRESH_P = 1
CROSSWALK_THRESHOLD = 7

#Color codes
BLUE = 0
GREEN = 1
YELLOW = 2

#Color Filters
RED_LOW = np.array([0,0,200])
RED_HIGH = np.array([0,0,255])
YG_LOW = np.array([0,0,0])
YG_HIGH = np.array([40,150,150])
BLUE_LOW = np.array([0,0,0])
BLUE_HIGH = np.array([240,40,40])

#Color Thresholds
#YG_THRESHOLD = 3000000
BLUE_THRESHOLD = 4500000
#YG_THRESHOLD_LOW = 1000000
BLUE_THRESHOLD_LOW = 500000
#BLUE_THRESHOLD = 5500000

YG_THRESHOLD_1 = 4500000
YG_THRESHOLD_LOW_1 = 4000000
YG_THRESHOLD_2 = 3000000
YG_THRESHOLD_LOW_2 = 1000000

# Movement Codes
IDK = -1
FWD_LOOK_LEFT = 0
FWD_LOOK_RIGHT = 1
FWD_LOOK_NONE = 2
FWD_LOOK_CROSS = 3
TURN_LEFT = 4
TURN_RIGHT = 5
TURN_AROUND = 6
FWD_CROSS_WITHOUT_KILL = 7
TURN_FORWARD = 8

#PID parameters
INTEGRAL_WINDUP = 0.5
previous_error = 0
integral_response = 0

#Misc Global Variables
lock_movement = False
evaluation_lock = -1
fwd_total = 0
idk_total = 0
mid_total = 0
movement = FWD_LOOK_RIGHT
previous_movement = FWD_LOOK_RIGHT
move = Twist()
n_turns = 0
car_found = False
previous_image = np.zeros((480, 640), np.uint8)

turn_thresh = [20, 15, 20, 15]

#Counts the number and type of edges in the given image
# cv_img_binary: greyscale opencv image, bgr color
# blank_image: image to draw lines on
# draw: True if observed lines should be drawn on blank_image
#           blue represent horizontal lines
#           green represent vertical lines
#           red are unclassified lines
# Returns: ((fwd_pt, left_pt, right_pt, middle_pt, idk_pt), blank_image)
#           fwd_pt is the number of forward-pointing lines
#           left_pt is number of horizontal lines on the left half of the image
#           right_pt is number of horizontal lines on the right half of the image
#           middle_pt is number of horizontal lines on the middle of the image
#           idk_pt is the number of unclassified lines
def edge_detector_p(cv_img_binary, blank_image, draw = True):

    #Use Probabilistic Hough Transform to find edges
    edges = cv.Canny(cv_img_binary, CANNY_L, CANNY_H, apertureSize = 3)
    lines = cv.HoughLinesP(edges, 1, np.pi/180, HOUGH_THRESH_P,\
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

        #Calculate the gradient of the lines
        x1, y1, x2, y2 = line_coords[i]
        if x2 - x1 != 0:
            gradient = abs(((float) (y2 - y1))/((float) (x2 - x1)))
        else:
            gradient = 1 #definitely vertical

        #If it is a vertical line, add to the number of vertical lines
        gradients.append(gradient)
        if gradient > 0.5:
            if draw == True:
                cv.line(blank_image, (x1,y1), (x2,y2), (0,255,0),2)
            fwd_pt = fwd_pt + 1

        #If it is a horizontal line, add to the number of horizontal lines
        elif gradient < 0.3:
            if draw == True:
                cv.line(blank_image, (x1,y1), (x2,y2), (255,0,0),2)
            
            #if x1 or x2 is on the left half, turn is on the left
            if max(x1,x2) < (IMG_WIDTH)/2:
                left_pt = left_pt + 1
            #if x1 or x2 is on the right half, turn is on the right
            elif min(x1,x2) > (IMG_WIDTH)/2:
                right_pt = right_pt + 1
            #if one coordinate is on the left and another on the right
            else:
                middle_pt = middle_pt + 1

        #If it is an IDK line
        else:
            if draw == True:
                cv.line(blank_image, (x1,y1), (x2,y2), (0,0,255),2)
            idk_pt = idk_pt + 1
   
    #TESTING 
    #print("f:{},l:{},r:{},m:{},idk:{}".format(fwd_pt,left_pt,right_pt,middle_pt,idk_pt))
    
    return ((fwd_pt, left_pt, right_pt, middle_pt, idk_pt), blank_image) 


#Determines the possible directions of motion
# edge_points: (fwd_pt, left_pt, right_pt, middle_pt)
#           fwd_pt is the number of forward-pointing lines
#           left_pt is number of horizontal lines on the left half of the image
#           right_pt is number of horizontal lines on the right half of the image
#           middle_pt is number of horizontal lines on the middle of the image
# Returns: (forward, left, right)
#           forward: True if the forward path is available
#           left: True if the left path is available
#           right: True if the right path is available
def determine_motion(edge_points):
 
    #Determine what movements are currently available based on observations
    forward = False  
    left = False
    right = False
    if edge_points[0] >= FWD_THRESH_P:
        forward = True
    if edge_points[1] >= LEFT_THRESH_P:
        left = True    
    if edge_points[2] >= RIGHT_THRESH_P:
        right = True
    if edge_points[3] >= MID_THRESH_P:
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


#Check if a particular frame convincingly has a car in it
# img: a bgr image
# returns True or False 
def check_for_car(img, car_colour):

    #Mask and sum images to check if a car is available

    if car_colour == BLUE:
        
        blue_mask = cv.inRange(img, BLUE_LOW, BLUE_HIGH)
        blue_sum = blue_mask.sum()

        #TESTING
        print("Checking for BLUE car: {}".format(blue_sum))
        if blue_sum > BLUE_THRESHOLD:
            print("There is a BLUE car here")
            return True
        elif blue_sum < BLUE_THRESHOLD_LOW:
            print("there is NO BLUE car here")
            return False
        else:
            print("BLUE not detected sufficiently")
            return None
    
    else:
    
        yg_mask = cv.inRange(img, YG_LOW, YG_HIGH)
        yg_sum = yg_mask.sum()
    
        #TESTING
        print("Checking for YELLOWGREEN car: {}".format(yg_sum))

        if navigator.get_current_node() == 14 or navigator.get_current_node() == 12:
            YG_THRESHOLD = YG_THRESHOLD_1
            YG_THRESHOLD_LOW = YG_THRESHOLD_LOW_1
        else:
            YG_THRESHOLD = YG_THRESHOLD_2
            YG_THRESHOLD_LOW = YG_THRESHOLD_LOW_2
            
        if yg_sum > YG_THRESHOLD:
            print("There is a YELLOWGREEN car here")
            return True
        elif yg_sum < YG_THRESHOLD_LOW:
            print("There is NO YELLOWGREEN car here")
            return False
        else:
            print("YELLOWGREEN not detected sufficiently")
            return None


#Controls the model vehicle
# data: raw ROS image file
def driver(data):

    global lock_movement
    global evaluation_lock
    global fwd_total
    global mid_total
    global idk_total
    global movement
    global n_turns
    global move
    global car_found
    global previous_image
    global previous_movement

    #Obtain and crop the raw image
    cv_img_raw = bridge.imgmsg_to_cv2(data, "bgr8")
    cv_img = cv_img_raw[-IMG_HEIGHT:, :, :]

    #Apply greyscale, averaging, and thresholding to get binary image
    cv_img_gray = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)
    kernel = np.ones((5,5), np.float32)/25
    cv_img_gray = cv.filter2D(cv_img_gray,-1,kernel)
    retval, cv_img_binary = cv.threshold(cv_img_gray, BINARY_THRESH, 255, cv.THRESH_BINARY)

    #Instantiate a blank array for illustration purposes
    blank_image = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), np.uint8)
    
    #Obtain edges to determine current position on board
    edge_points, blank_image = edge_detector_p(cv_img_binary, blank_image)

    #Obtain possible movements from the edges
    forward, left, right = determine_motion(edge_points)
    
    #if the vehicle is not in the middle of a turn, determine new movement
    if lock_movement == False:
      
        #If there appears to be only a left turn, or only a right turn, 
        if left != right:
            #there is a possibility that the vehicle is misaligned so only 1
            # horizontal edge is visible. This can be solved by temporarily increasing
            # the size of the cropped image, exposing the hidden edges. If all goes 
            # properly, this statement should only be reached at intersections.
           
            #TESTING 
            print("Evaluating correctness of left: {} right: {}".format(left, right))
       
            #Get an enlarged version of the target image 
            #10 is too little to detect the new edge
            cv_img_mod = cv_img_raw[-(IMG_HEIGHT + 25):, :, :]
            cv_img_mod_gray = cv.cvtColor(cv_img_mod, cv.COLOR_BGR2GRAY)
            cv_img_mod_gray = cv.filter2D(cv_img_mod_gray,-1,kernel)
            retval, cv_img_mod_binary = cv.threshold(cv_img_mod_gray, BINARY_THRESH,\
                                                     255, cv.THRESH_BINARY)
            
            #Obtain a new set of edges
            edge_points_mod, cv_img_mod = edge_detector_p(cv_img_mod_binary, cv_img_mod)
            retval, left_new, right_new = determine_motion(edge_points_mod)
            
            #TESTING 
            #cv.imshow("VARIATION", cv_img_mod)
            
            #If the first reading is incorrect and the variation corrected it, 
            #And also if it did not make it worse, 
            if left_new == right_new and left_new == True:
                left = left_new
                right = right_new
                
                #TESTING
                print("New edge found: left: {} right: {}".format(left, right))

            else:

                #TESTING
                print("No new edge found: left: {} right: {}".format(left, right))

        #Note: By this point, all external input has been received. Its time to 
        # consult the state machine and find out where to go

        #The only available way is forward
        if forward == True and left == False and right == False:

            #Note: For a car to continue moving forward, no change in state, no
            # updating or locking, is required.

            if previous_movement == TURN_FORWARD or \
               previous_movement == FWD_LOOK_CROSS: #Note this will only happen if unlocked
                navigator.update_state()
                navigator.lock_current_state()

            movement = navigator.get_current_behavior()

        #Note: If the car is not going forward, the finite state machine has reached 
        #      a new node. So update the finite state machine, get the new behavior, and
        #      lock the fsm in the new state.

        #The car can go left, right or backwards
        elif forward == True and left == True and right == True:

            #Use the fsm for navigation
            navigator.update_state()
            navigator.lock_current_state()
            movement = navigator.get_current_behavior()

        #The car can go left, back, or (maybe) forwards
        elif forward == True and left == True and right == False:

            #Use the fsm for navigation
            navigator.update_state()
            navigator.lock_current_state()
            movement = navigator.get_current_behavior()
            
        #The car can go right, back or (maybe) forwards 
        elif forward == True and left == False and right == True:

            #Use the fsm for navigation
            navigator.update_state()
            navigator.lock_current_state()
            movement = navigator.get_current_behavior()

        #The car thinks it cannot go forwards, but there may be exceptions
        elif forward == False:

            #As long as left is still false and right is still false,
            #   a way forward might still be possible. PID will then bring it
            #   back to the centre.
            if edge_points[0] >= 1 and left == False and right == False:

                #TESTING
                print("Warning: Vehicle moving forward but veering. ")

                forward = True
    
                if previous_movement == TURN_FORWARD or \
                   previous_movement == FWD_LOOK_CROSS:
                    navigator.update_state()
                    navigator.lock_current_state()

                movement = navigator.get_current_behavior()

            #Same as forward == True and left == True and right == False
            elif edge_points[0] >= 1 and left == True and right == False:

                #TESTING
                print("Warning: Vehicle detects left but is veering. ")

                #Use the fsm for navigation
                navigator.update_state()
                navigator.lock_current_state()
                movement = navigator.get_current_behavior()
               
            #Same as forward == True and left == False and right == True
            elif edge_points[0] >= 1 and left == False and right == True:

                #TESTING
                print("Warning: Vehicle detects right but is veering. ")

                #Use the fsm for navigation
                navigator.update_state()
                navigator.lock_current_state()
                movement = navigator.get_current_behavior()
       
                #TESTING 
                #movement = RIGHT 

            #Same as forward == True and left == True and right == True
            elif edge_points[0] >= 1 and left == True and right == True:

                #TESTING
                print("Warning: Vehicle detects leftright but is veering. ")

                #Use the fsm for navigation
                navigator.update_state()
                navigator.lock_current_state()
                movement = navigator.get_current_behavior()

            #The car really doesnt know where it is.
            else:

                if previous_movement == FWD_LOOK_CROSS:
                    navigator.update_state()
                    navigator.lock_current_state()
                    movement = navigator.get_current_behavior()
                elif previous_movement == FWD_CROSS_WITHOUT_KILL:
                    movement = navigator.get_current_behavior()
                elif previous_movement == FWD_LOOK_NONE and \
                     navigator.get_current_node() == 10:
                    movement = navigator.get_current_behavior()
                elif previous_movement == FWD_LOOK_NONE and \
                     navigator.get_current_node() == 9:
                    navigator.update_state()
                    navigator.lock_current_state()
                    movement = navigator.get_current_behavior()
                else:
                    print("previous_movement: {}".format(previous_movement))
                    print("Unknown Position. Stopping Vehicle")
                    movement = IDK

        #Cheating a bit to ensure that the car doesnt fall off
        #Some parts of the track are a bit awkward
        if navigator.is_state_locked() == False and navigator.get_current_node() == 5:
            navigator.update_state()
            navigator.lock_current_state()
            movement = navigator.get_current_behavior()

        previous_movement = movement

    #if the vehicle is in the middle of a turn 
    else:
       
        #if the vehicle has completed a turn, unlock movement.
        if (forward == True and left == False and right == False) or evaluation_lock != -1:

            #Note: The Probabilistic Hough Transform actually produces self-consistent 
            #   results, as long as the same image is being used. This means that most of the
            #   error comes from the source image. So to average out random errors, different
            #   images from the same position must be used.
             
            #if the vehicle is not evaluating a turn yet, begin evaluating the turn
            if evaluation_lock == -1:
           
                #TESTING 
                print("Evaluation unlocked. Begin evaluating turn {}".format(n_turns))

                evaluation_lock = 0
                fwd_total = 0
                idk_total = 0
                mid_total = 0

            #Take like 5 different images to be sure.
            elif evaluation_lock < 5:
                
                #TESTING
                print("Evaluating Turn, Trial {}".format(evaluation_lock))

                fwd_total = fwd_total + edge_points[0]
                mid_total = mid_total + edge_points[3]
                idk_total = idk_total + edge_points[4]

                evaluation_lock = evaluation_lock + 1
 
            elif evaluation_lock == 5:
   
                #Extract only the red part of the image
                cv_redmask = cv.inRange(cv_img_raw[-(IMG_HEIGHT+40):, :, :], RED_LOW, RED_HIGH)

                redmask_sum = cv_redmask.sum()
           
                #TESTING 
                print("Evaluation Data for turn {} obtained.".format(n_turns))
            
                #evaluate whether the turn is really complete with stricter conditions 
                #For Turn 0, the threshold seems to be 20
                #For Turn 1, the threshold seems to be 15

                #Note: If the current turn is 9, don't redmask sum, because you actually
                #      want to turn into the crosswalk
                # TODO: Adapt this for future turns
                if (fwd_total >= turn_thresh[n_turns % 4] or \
                     (navigator.get_current_node() == 9 and redmask_sum > 2000000))\
                     and idk_total == 0 and mid_total == 0 and \
                     (navigator.get_current_node() == 9 or redmask_sum < 500000):
           
                    #TESTING 
                    print("Turn {} Complete".format(n_turns))
                    print("fwd_total: {}, redmask_sum: {}".format(fwd_total, redmask_sum))

                    #TESTING
                    #cv.imshow("turn_completed_{}_edges".format(n_turns), blank_image)
                    #cv.imshow("turn_completed_{}_color".format(n_turns), cv_img)
                    
                    #TESTING
                    #movement = FWD

                    lock_movement = False
                    n_turns = n_turns + 1

                    #Note: At this point, the turn is complete. Unlock the state lock on the 
                    # finite state machine
                    navigator.unlock_current_state()

                    #Now that the vehicle is unlocked, it is free to move forward until
                    # it reaches the next node
                    movement = navigator.get_current_behavior()

                    #Note: If the current node is 6, then back up a bit. This will allow
                    #      the car at 7 to be seen properly

                    #TODO: Decide if this is right or not
                    if navigator.get_current_node() == 6:

                        move = Twist()
                        move.linear.x = -0.5
                        move.linear.y = 0.0
                        move.angular.z = 0.0
                        pub_vel.publish(move)

                        rospy.sleep(1.0)

                        move = Twist()
                        move.linear.x = 0.0
                        move.linear.y = 0.0
                        move.linear.z = 0.0
                        pub_vel.publish(move)

                    if navigator.get_current_node() == 0:

                        move = Twist()
                        move.linear.x = -0.5
                        move.linear.y = 0.0
                        move.angular.z = 0.0
                        pub_vel.publish(move)

                        #1.0 might be too much
                        rospy.sleep(0.5)

                        move = Twist()
                        move.linear.x = 0.0
                        move.linear.y = 0.0
                        move.linear.z = 0.0
                        pub_vel.publish(move)

                else:
                    
                    #TESTING
                    print("Turn Incomplete: fwd: {}, red: {}, idk: {} mid: {}".format(\
                    fwd_total, redmask_sum, idk_total, mid_total))

                #release the evaluation lock, allowing the car to continue turning
                evaluation_lock = -1

    #If the vehicle should move forward, apply PID on the edges of the line
    #Also actively search the left and right for cars
    if movement == FWD_LOOK_LEFT or movement == FWD_LOOK_RIGHT or \
        movement == FWD_LOOK_NONE or movement == FWD_LOOK_CROSS or \
        movement == FWD_CROSS_WITHOUT_KILL:

        #Note: A car moving forward will either encounter a turn, a car or a crosswalk
        # If it is a crosswalk, then a new flag system must be devised to handle it

        #sum the thresholded image to reduce random error
        row = []
        for i in range(IMG_WIDTH):
            vert = 0
            for j in range(IMG_HEIGHT):
                vert = vert + cv_img_binary[j][i]
            row.append(vert)
        
        #find the lower bound
        lower = IMG_WIDTH/2
        c = IMG_WIDTH/2
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
        upper = IMG_WIDTH/2
        c = IMG_WIDTH/2
        upper_found = False
        while (c < IMG_WIDTH - 1):
            
            if row[c] > 10*255:
                upper = c
                upper_found = True
                break
            c = c + 1
          
        if upper_found == False:
            upper = IMG_WIDTH - 2

        #Obtain the error
        error = IMG_WIDTH/2 - int((upper + lower)/2)

        #TESTING          
        cv.circle(blank_image, (upper, IMG_HEIGHT - 10), 10, (0,0,255), -1)
        cv.circle(blank_image, (lower, IMG_HEIGHT - 10), 10, (0,255,0), -1)
       
        #Use PID to correct the forward movement of the vehicle 
        move = Twist()
        move.linear.x = 1.0
        move.linear.y = 0.0
        
        kp = 6.0/IMG_WIDTH
        ki = 0
        kd = 20.0/IMG_WIDTH

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
             
        previous_error = error

        #Note: Now that the PID is settled, search for features on the track
        if movement == FWD_LOOK_LEFT:

            pub_vel.publish(move)

            check_img = cv_img_raw[:, 0:200, :]
            
            #TESTING
            cv.imshow("check_img", check_img)
            cv.waitKey(1)

            img_colour = navigator.get_car_colour(next_node = True)
            current_car_found = check_for_car(check_img, img_colour)
            if current_car_found is True and car_found == False:

                print("A car was detected on the left")

                #This means that the vehicle is in a position to read for a car
                new_state = navigator.update_state()
                navigator.lock_current_state()

                #Apply SIFT to extract a sub-image from the main image
                check_img_gray = cv.cvtColor(check_img, cv.COLOR_BGR2GRAY)
                img_colour = navigator.get_car_colour()
                edges = car_finder.match_car(check_img_gray, img_colour)

                if edges is not None:

                    if edges[0][1] - 40 > 0:
                        min_y = edges[0][1] - 40
                    else: 
                        min_y = 0

                    if edges[1][1] + 40 > len(check_img):
                        max_y = edges[1][1] + 40
                    else:
                        max_y = len(check_img)
                        
                    check_img = check_img[min_y:max_y, edges[0][0]:edges[1][0], :]
    
                    #TESTING
                    print("Image successfully obtained at node {}".format(new_state))
                    cv.imshow("SIFT output at node {}".format(new_state), check_img)
                    cv.waitKey(1)

                else:

                    #TESTING
                    print("Image could not be obtained at node {}".format(new_state))
                    cv.imshow("Failed SIFT at node {}".format(new_state), check_img)
                    cv.waitKey(1)
                 

                #TODO: Call NN code to read the license plate

                #TODO: Check that the output of the NN is satisfactory before moving on
                #       Otherwise, repeat the steps above
                navigator.unlock_current_state()

                #Note: Now that a car was found, the same car should not be found again
                #      This flag will prevent the same car from being observed twice.
                car_found = True

            else:
                
                #If no car was found, that means that the old car is gone
                if current_car_found is False:

                    #TESTING
                    print("car is gone")

                    car_found = False
                
        elif movement == FWD_LOOK_RIGHT:

            pub_vel.publish(move)

            check_img = cv_img_raw[:, -200:, :]

            #TESTING
            cv.imshow("check_img", check_img)
            cv.waitKey(1)

            img_colour = navigator.get_car_colour(next_node = True)
            current_car_found = check_for_car(check_img, img_colour)
            if current_car_found is True and car_found is False:

                print("A car was detected on the right")

                #This means that the vehicle is in a position to read for a car
                new_state = navigator.update_state()
                navigator.lock_current_state()

                #Apply SIFT to extract a sub-image from the main image
                check_img_gray = cv.cvtColor(check_img, cv.COLOR_BGR2GRAY)
                img_colour = navigator.get_car_colour()
                edges = car_finder.match_car(check_img_gray, img_colour)

                if edges is not None:

                    if edges[0][1] - 40 > 0:
                        min_y = edges[0][1] - 40
                    else: 
                        min_y = 0

                    if edges[1][1] + 40 > len(check_img):
                        max_y = edges[1][1] + 40
                    else:
                        max_y = len(check_img)
                        
                    cropped_image = check_img[min_y:max_y, edges[0][0]:edges[1][0], :]
    
                    #TESTING
                    print("Image successfully obtained at node {}".format(new_state))
                    cv.imshow("SIFT output at node {}".format(new_state), cropped_image)
                    cv.waitKey(1)

                else:

                    #TESTING
                    print("Image could not be obtained at node {}".format(new_state))
                    cv.imshow("Failed SIFT at node {}".format(new_state), check_img)
                    cv.waitKey(1)
                    
                 
                #TODO: Call NN code to read the license plate

                #TODO: Check that the output of the NN is satisfactory before moving on
                #        Otherwise, repeat the steps above
                navigator.unlock_current_state()

                #Note: Now that a car was found, the same car should not be found again
                #      This flag will prevent the same car from being observed twice.
                car_found = True

            else:

                if current_car_found is False:
                    
                    #TESTING
                    print("car is gone")

                    car_found = False

        elif movement == FWD_LOOK_CROSS:

            #Note: At ths point, we are moving forward, and we need to find a 
            # crosswalk right in front of us. 

            #If a crosswalk is actually here
            cv_redmask = cv.inRange(cv_img_raw[-(IMG_HEIGHT+40):, :, :], RED_LOW, RED_HIGH)
            redmask_sum = cv_redmask.sum()
            if redmask_sum > 500000:
                
                #TESTING
                print("A crosswalk directly ahead is detected!")
                
                rospy.sleep(0.5)
               
                #TESTING
                print("Stopping car due to crosswalk....") 

                move = Twist()
                move.linear.x = 0.0
                move.linear.y = 0.0
                move.angular.z = 0.0
                pub_vel.publish(move)

                #Sleep for 1 second just to be sure
                rospy.sleep(1.0)
                
                #TESTING
                print("Car stopped due to crosswalk")

                navigator.unlock_current_state()

                previous_image = cv.cvtColor(cv_img_raw, cv.COLOR_BGR2GRAY)

            else:
                
                #TESTING
                print("Moving forward slowly..")
               
                #move forward slowly
                move = Twist()
                move.linear.x = 0.3
                move.linear.y = 0.0
                move.angular.z = 0.0
                pub_vel.publish(move)

                #OLD: publish old PID

        elif movement == FWD_LOOK_NONE:

            #Publish the old PID
            pub_vel.publish(move)

            #Signify that the old car is no longer needed
            car_found = False

        elif movement == FWD_CROSS_WITHOUT_KILL:
           
            #pub_vel.publish(move)

            car_found = False
            
            #Note: At this point, the car is ready to enter the junction.
            #      Use the difference between the current and previous frame
            #      to determine if someone is on the track. 

            current_img = cv.cvtColor(cv_img_raw, cv.COLOR_BGR2GRAY)
            
            img_diff = cv.subtract(current_img, previous_image)
            img_diff_sum = img_diff.sum()
            print("DIFF: {}".format(img_diff_sum))
           
            #cv.imshow("current", current_img)
            #cv.waitKey(1)
            #cv.imshow("previous", previous_image) 
            #cv.waitKey(1)
            cv.imshow("diff", img_diff)
            cv.waitKey(1)
           
            if img_diff_sum > 300000: 
            #if img_diff_sum > 50000 and img_diff_sum < 150000:
            
                #TESTING
                print("GUY ON ROAD! Car is stopped")
                
                move = Twist()
                move.linear.x = 0.0
                move.linear.y = 0.0
                move.angular.z = 0.0
                pub_vel.publish(move)
            
            else:
                
                #TESTING 
                print("Guy is not on road. Move along")

                #Publish the old PID
                move = Twist()
                move.linear.x = 1.0
                move.linear.y = 0.0
                move.angular.z = 0.0
                pub_vel.publish(move) 

                #1.5 seems too short
                #2.0 is alright               
                #2.3 was a bit too long
                #rospy.sleep(1.75)
                #rospy.sleep(1.5)
                rospy.sleep(1.0)
               
                #TODO 
                #I see the problem. THe car exits state 5. 
                #It does not immediately go to state 6.
                #First, it goes to the edge between 5 and 6, where it first
                #tries to detect a turn, which will be difficult to succeed
                #Cheat the system, if the car is supposed to go FWD_LOOK_NONE, but the
                #current state is 5, just update it directly to 6.
 
                #move = Twist()
                #move.linear.x = 0.0
                #move.linear.y = 0.0
                #move.angular.z = 0.0
                #pub_vel.publish(move)

                print("Crosswalk crossed")
            
                #Unlock state
                navigator.unlock_current_state()


            previous_image = current_img
             
    #Execute semi-hardcoded subroutine to turn left.        
    elif movement == TURN_LEFT:

        #If it is the first movement to the left,
        if lock_movement == False:

            #TESTING 
            print("Entering Junction: ")
            
            #TESTING
            #cv.imshow("before entering {}".format(n_turns), cv_img)

            #Fully enter the junction by waiting x seconds
            rospy.sleep(1.5)

            #Stop the car
            move = Twist()
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.angular.z = 0.0
            pub_vel.publish(move)

            #TESTING
            print("Junction Entered. Beginning LEFT TURN")

            #lock the vehicle state
            lock_movement = True

        #If the vehicle is in the middle of a turn
        elif evaluation_lock == -1:

            #Send appropriate turning messages
            move = Twist()
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.angular.z = 1.75
            pub_vel.publish(move)
        
        #If the vehicle is evaluating a turn
        else:
 
            #Stop the car
            move = Twist()
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.angular.z = 0.0
            pub_vel.publish(move)        

    #Execute semi-hardcoded subroutine to turn right         
    elif movement == TURN_RIGHT:

        #If it is the first movement to the right,
        if lock_movement == False:

            #TESTING 
            print("Entering Junction: ")
           
            #TESTING 
            #cv.imshow("before entering {}".format(n_turns), cv_img)

            #Fully enter the junction by waiting x seconds
            rospy.sleep(1.5)
            
            #Stop the car 
            move = Twist()
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.angular.z = 0.0
            pub_vel.publish(move)

            #TESTING
            print("Junction Entered. Beginning RIGHT TURN")

            #lock the vehicle state
            lock_movement = True

        #If the vehicle is in the middle of a turn
        elif evaluation_lock == -1:

            #Send appropriate turning messages
            move = Twist()
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.angular.z = -1.75
            pub_vel.publish(move)
        
        #If the vehicle is evaluating a turn
        else:

            #Stop the car
            move = Twist()
            move.linear.x = 0.0
            move.linear.y = 0.0
            move.angular.z = 0.0
            pub_vel.publish(move)        
        
    elif movement == TURN_FORWARD:
        
        print("Entering Junction: ")
        
        #Fully enter the junction by waiting x seconds 
        rospy.sleep(1.25)
        
        #Stop the car
        move = Twist()
        move.linear.x = 0.0
        move.linear.y = 0.0
        move.angular.z = 0.0
        pub_vel.publish(move)

        print("Junction has been entered")

        #Unlock the state of the car
        navigator.unlock_current_state()

    elif movement == TURN_AROUND:

        #Note: This is not necessary
        
        #TESTING
        print("Supposed to turn around here")
    
    #Vehicle is in an unknown position; Stop the vehicle
    elif movement == IDK:

        move = Twist()
        move.linear.x = 0.0
        move.linear.y = 0.0
        move.angular.z = 0.0
        pub_vel.publish(move)
         
        #TESTING 
        print("Vehicle has been stopped for safety reasons. Dumping state...")
        print("f: {} l: {} r: {} m: idk: {}".format(edge_points[0], edge_points[1], \
                    edge_points[2], edge_points[3], edge_points[4]))
        
        while True:
            continue

    cv.imshow('img', blank_image)
    cv.waitKey(1)


#Function to test various aspects of the vehicle without controlling it
# data: raw ROS image file
def observer(data):
    
    global lock_movement
    global movement
    global previous_image
    
    #Obtain and crop the raw image
    cv_img_raw = bridge.imgmsg_to_cv2(data, "bgr8")
    cv_img = cv_img_raw[-IMG_HEIGHT:, :, :]

    #Apply greyscale, averaging, and thresholding to get binary image
    cv_img_gray = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)
    kernel = np.ones((5,5), np.float32)/25
    cv_img_gray = cv.filter2D(cv_img_gray,-1,kernel)
    retval, cv_img_binary = cv.threshold(cv_img_gray, BINARY_THRESH, 255, cv.THRESH_BINARY)

    #Instantiate a blank array for illustration purposes
    blank_image = np.zeros((IMG_HEIGHT, IMG_WIDTH, 3), np.uint8)
    
    #Obtain edges to determine current position on board
    edge_points, blank_image = edge_detector_p(cv_img_binary, blank_image)

    #cv_img_side = cv_img_upper[:, 0:200, :]
    #cv_img_side = cv_img_upper[:, -200:, :]
    #cv_img_side = cv_img_upper[:, -200:, :]
    
    #check_for_car(cv_img_side)
    cv.imshow("edges", blank_image)
    cv.waitKey(1)
    #cv.imshow("side", cv_img_side)
    #cv.waitKey(1)
     
    #Note, I'm going to try taking 2 greyscale images and doing bitwise and
    #If the bitwise and is sufficiently different, then clearly something is moving
    
    #difference = cv.subtract(cv_img_gray, previous_image)
    #summa = difference.sum()
    
    #if (summa > 70000 and summa < 150000):
    #    print("guy is here")
    
    #cv.imshow("img", cv_img_gray)
    #cv.waitKey(1)
    
    #cv.imshow("diff", difference) 
    #cv.waitKey(1)

    #previous_image = cv_img_gray


#Initialize the node
rospy.init_node('driver')
rospy.loginfo('driver node started')

#Publish a movement topic
pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

#Initialize sift-based car finder
car_finder = sift_based_car_finder.SIFTBasedCarFinder()

#Initialize edge-based car finder
#car_finder = hough_based_car_finder.EdgeBasedCarFinder()

#Initialize the finite state machine navigator
navigator = navigator.FiniteStateNavigator()

#Delay for observation purposes
rospy.sleep(10)

#Initialize the movement with a slight angle to demonstrate PID in action
move = Twist()
move.linear.x = 0.25
move.linear.y = 0.0
move.linear.z = 0.0
move.angular.x = 0.0
move.angular.y = 0.0
move.angular.z = 0.0
pub_vel.publish(move)

rospy.sleep(0.30)

move.linear.x = 0.00
move.linear.y = 0.0
move.linear.z = 0.0
move.angular.x = 0.0
move.angular.y = 0.0
move.angular.z = 0.0
pub_vel.publish(move)

#Subscribe to camera
sub_image = rospy.Subscriber("/rrbot/camera1/image_raw", Image, driver)

#Bridge ros and opencv
bridge = CvBridge()

#Stops driver node from dying
rospy.spin()

