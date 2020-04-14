#! usr/bin/env/python

# ENPH 353 final project: Self-driving vehicle with license-plate reading
# navigator.py: script which handles the navigation of the vehicle around the track
# Authors:
#   Miles Justice
#   David Widjaja 18950063

import numpy as np

#Possible routes
ROUTE_1 = [0, 1, 2, 3, 4, 12, 11]

#Currently Impossible routes
ROUTE_2 = [0, 1, 2, 3, 4, 12, 11, 13, 4, 12, 11]
ROUTE_3 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
ROUTE_4 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 4, 13, 11]

IDK = -1
FWD_LOOK_LEFT = 0
FWD_LOOK_RIGHT = 1
FWD_LOOK_NONE = 2
FWD_LOOK_CROSS = 3
TURN_LEFT = 4
TURN_RIGHT = 5
TURN_AROUND = 6

class FiniteStateNavigator:

    #Initializes the navigator class
    def __init__(self):

        #Instantiate the route
        self.index = 0
        self.route = ROUTE_1
        self.state_lock = False

        #Note: When the route is initialized, the car is at node 0, but unlocked
        # This means that the car between node 0 and 1
        # So when the car goes forward, it will immediately encounter node 1
    
    #Gets the next node
    def get_next_node(self):

        return self.route[(self.index + 1) % len(self.route)]

    #Get the current node
    def get_current_node(self):
        
        return self.route[self.index]   
    
    #Gets the previous node
    def get_previous_node(self):

        if self.index == 0:
            self.index = len(self.route) - 1
        
        return self.route[self.index - 1] 

    #lock the current state the finite state machine
    def lock_current_state(self):

        self.state_lock = True
   
    #unlock the current state of the finite state machine 
    def unlock_current_state(self):

        self.state_lock = False

    #Update the position of the node
    def update_state(self):

        if self.state_lock == True:
            self.curr = self.curr + 1
            return self.curr
        else:
            return -1
        
    #Get the current behavior of the node from the finite state machine
    def get_current_behavior(self):

        nex == self.get_next_node(curr)
        previous = self.get_previous_node(curr)
        
        #Note: FML, if only I knew a bit of graph theory

        if ((curr == 0 and nex == 1) or \
           (curr == 1 and nex == 2) or \
           (curr == 6 and nex == 7) or \
           (curr == 7 and nex == 8)) and self.state_lock == False:

            #Tell the driver to look right for a car while moving forward
            movement = FWD_LOOK_RIGHT

        elif ((curr == 11 and nex == 13) or \
             (curr == 4 and nex == 12)) and self.state_lock == False:

            #Tell the driver to look left for a car while moving forward
            movement = FWD_LOOK_LEFT

        elif ((curr == 2 and nex == 3) or \
             (curr == 3 and nex == 0) or \
             (curr == 3 and nex == 4) or \
             (curr == 4 and nex == 3) or \
             (curr == 0 and nex == 11) or \
             (curr == 11 and nex == 0) or \
             (curr == 5 and nex == 14) or \
             (curr == 5 and nex == 4) or \
             (curr == 10 and nex == 11) or \
             (curr == 10 and nex == 15) or \
             (curr == 8 and nex == 9) or \
             (curr == 9 and nex == 6) or \
             (curr == 5 and nex == 6) or \
             (curr == 10 and nex == 9) or \
             (curr == 12 and nex == 11) or \
             (curr == 13 and nex == 4)) and self.state_lock == False:

            #Tell the driver to not look for anything while moving forward
            movement = FWD_LOOK_NONE

        elif ((curr == 4 and nex == 5) or \
             (curr == 6 and nex == 5) or \
             (curr == 9 and nex == 10) or \
             (curr == 11 and nex == 10) or \
             (curr == 14 and nex == 5) or \
             (curr == 15 and nex == 10)) and self.state_lock == False:

            #Tell the driver to look forward for a crosswalk
            movement = FWD_LOOK_CROSS

        elif ((prev == 2 and curr == 3 and nex == 4) or \
             (prev == 11 and curr == 0 and nex == 1) or \
             (prev == 12 and curr == 11 and nex == 0) or \
             (prev == 13 and curr == 4 and nex == 14) or \
             (prev == 13 and curr == 4 and nex == 5) or \
             (prev == 5 and curr == 6 and nex == 7) or \
             (prev == 8 and curr == 9 and nex == 10) or \
             (prev == 3 and curr == 4 and nex == 12) or \
             (prev == 10 and curr == 11 and nex == 13) or \
             (prev == 10 and curr == 15 and nex == 13)) and self.state_lock == True:

            #Illegal: 4-14-12
            #         15-11-13
 
            #Tell the driver to turn right
            movement = TURN_RIGHT
        
        elif ((prev == 4 and curr == 3 and nex == 0) or \
             (prev == 3 and curr == 0 and nex == 11) or \
             (prev == 13 and curr == 4 and nex == 3) or \
             (prev == 12 and curr == 11 and nex == 15) or \
             (prev == 12 and curr == 11 and nex == 10) or \
             (prev == 0 and curr == 11 and nex == 13) or \
             (prev == 14 and curr == 4 and nex == 12) or \
             (prev == 5 and curr == 14 and nex == 12) or \
             (prev == 10 and curr == 9 and nex == 6) or \
             (prev == 9 and curr == 6 and nex == 5)) and self.state_lock == True
             
            #Illegal: 5-4-12, 
            #         11-15-13

            #Tell the driver to turn left
            movement = TURN_LEFT

        elif ((prev == 13 and curr == 4 and nex == 12) or \
             (prev == 12 and curr == 11 and nex == 13)) and self.state_lock == True
               
            #Tell the driver to do a 180 turn 
            movement = TURN_AROUND

        else:
           
            #Note: This should be an impossible state. Curl up and cry
            movement = IDK

        return movement
