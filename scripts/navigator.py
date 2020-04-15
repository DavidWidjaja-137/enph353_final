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
ROUTE_2 = [0, 1, 2, 3, 4, 14, 5, 6, 7, 8, 9, 10, 15, 11, 0]

#Movement Codes
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

#Colour codes
BLUE = 0
GREEN = 1
YELLOW = 2

class FiniteStateNavigator:

    #Initializes the navigator class
    def __init__(self):

        #Instantiate the route
        self.index = 0
        self.route = ROUTE_2
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
            return self.route[len(self.route) - 1]
        else:
            return self.route[self.index - 1]
        
    #lock the current state the finite state machine
    def lock_current_state(self):
        
        #TESITNG
        print("state locked")

        self.state_lock = True
   
    #unlock the current state of the finite state machine 
    def unlock_current_state(self):

        #TESTING
        print("state unlocked")

        self.state_lock = False

    #Update the position of the node
    def update_state(self):

        #State can only be updated when it is not locked in place
        if self.state_lock == False:
            self.index = self.index + 1

            #overflow, so loop back
            if self.index == len(self.route):
               self.index = 0
            
            #TESTING
            print("state updated to {}".format(self.route[self.index]))

            return self.route[self.index]

        else:

            #TESTING
            print("State update from {} failed; state lock still active".format( \
                   self.route[self.index]))

            return -1

    #Get the colour of the current car
    def get_car_colour(self, next_node = False):

        if next_node == True:
            curr = self.get_next_node()
        else:
            curr = self.get_current_node()
        
        if curr == 2 or curr == 1:
            
            #TESTING
            print("Look for a BLUE car")
        
            return BLUE
        
        elif curr == 12 or curr == 7:
            
            #TESTING
            print("Look for a GREEN car")
        
            return GREEN 
        
        elif curr == 13 or curr == 8:

            #TESTING
            print("Look for a YELLOW car")
        
            return YELLOW
        
    #Get the current behavior of the node from the finite state machine
    def get_current_behavior(self):

        curr = self.route[self.index]
        nex = self.get_next_node()
        prev = self.get_previous_node()
        
        #Note: FML, if only I knew a bit of graph theory

        if ((curr == 0 and nex == 1) or \
           (curr == 1 and nex == 2) or \
           (curr == 6 and nex == 7) or \
           (curr == 7 and nex == 8)) and self.state_lock == False:

            #Tell the driver to look right for a car while moving forward
            movement = FWD_LOOK_RIGHT

            #TESTING
            print("FWD_LOOK_RIGHT curr: {} nex: {}".format(curr, nex))

        elif ((curr == 11 and nex == 13) or \
             (curr == 4 and nex == 12)) and self.state_lock == False:

            #Tell the driver to look left for a car while moving forward
            movement = FWD_LOOK_LEFT

            #TESTING
            print("FWD_LOOK_LEFT curr: {} nex: {}".format(curr, nex))

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

            #TESTING
            print("FWD_LOOK_NONE curr: {} nex: {}".format(curr, nex))

        elif ((curr == 4 and nex == 5) or \
             (curr == 6 and nex == 5) or \
             (curr == 9 and nex == 10) or \
             (curr == 11 and nex == 10) or \
             (curr == 14 and nex == 5) or \
             (curr == 15 and nex == 10)) and self.state_lock == True:

            #Problem:
            # State x-9-10-True is the same as state 9-10-True. 

            #Tell the driver to look forward for a crosswalk
            movement = FWD_LOOK_CROSS

            #TESTING
            print("FWD_LOOK_CROSS curr: {} nex: {}".format(curr, nex))
    
        elif (curr == 5 or curr == 10) and self.state_lock == True:

            movement = FWD_CROSS_WITHOUT_KILL

            #TESTING
            print("FWD_CROSS_WITHOUT_KILL curr: {} nex: {} prev: {}".format( \
                    curr, nex, prev))

        elif ((prev == 3 and curr == 4 and nex == 14) or \
              (prev == 5 and  curr == 14 and nex == 4) or \
              (prev == 0 and curr == 11 and nex == 15) or \
              (prev == 10 and curr == 15 and nex == 11) or \
              (prev == 14 and curr == 4 and nex == 3) or \
              (prev == 5 and curr == 4 and nex == 3) or \
              (prev == 15 and curr == 11 and nex == 0) or \
              (prev == 10 and curr == 11 and nex == 0)) and self.state_lock == True:
            
            movement = TURN_FORWARD

            #TESTING
            print("TURN_FORWARD curr: {} nex: {} prev: {}".format(curr, nex, prev))

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

            #TESTING
            print("TURN_RIGHT curr: {} nex: {} prev: {}".format(curr, nex, prev))
        
        elif ((prev == 4 and curr == 3 and nex == 0) or \
             (prev == 3 and curr == 0 and nex == 11) or \
             (prev == 13 and curr == 4 and nex == 3) or \
             (prev == 12 and curr == 11 and nex == 15) or \
             (prev == 12 and curr == 11 and nex == 10) or \
             (prev == 0 and curr == 11 and nex == 13) or \
             (prev == 14 and curr == 4 and nex == 12) or \
             (prev == 5 and curr == 14 and nex == 12) or \
             (prev == 10 and curr == 9 and nex == 6) or \
             (prev == 9 and curr == 6 and nex == 5)) and self.state_lock == True:
             
            #Illegal: 5-4-12, 
            #         11-15-13

            #Tell the driver to turn left
            movement = TURN_LEFT

            #TESTING
            print("TURN_LEFT curr: {} nex: {} prev: {}".format(curr, nex, prev))

        elif ((prev == 13 and curr == 4 and nex == 12) or \
             (prev == 12 and curr == 11 and nex == 13)) and self.state_lock == True:
               
            #Tell the driver to do a 180 turn 
            movement = TURN_AROUND

            #TESTING
            print("TURN_AROUND curr: {} nex: {} prev: {}".format(curr, nex, prev))

        else:
           
            #Note: This should be an impossible state. Curl up and cry
            movement = IDK

            #TESTING
            print("IDK curr: {} nex: {} prev: {} lock: {}".format(curr, nex, \
                                                            prev, self.state_lock))

        return movement
