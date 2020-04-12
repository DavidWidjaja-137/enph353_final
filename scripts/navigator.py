#! usr/bin/env/python

# ENPH 353 final project: Self-driving vehicle with license-plate reading
# navigator.py: script which handles the navigation of the vehicle around the track
# Authors:
#   Miles Justice
#   David Widjaja 18950063

import numpy as np

#Possible routes
ROUTE_1 = [0, 1, 2, 3, 4, 12, 11, 0]

#Currently Impossible routes
ROUTE_2 = [0, 1, 2, 3, 4, 12, 11, 13, 4, 12, 11, 0]
ROUTE_3 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 0]
ROUTE_4 = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 4, 13, 11, 0]

class Navigator:

    #Initializes the navigator class
    def __init__(self):

        #Instantiate the route
        self.index = 0
        self.route = ROUTE_1
    
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

    #Update the position of the node
    def update_node(self):
            
        self.curr = self.curr + 1
        
        return self.curr

    #Get the current behavior of the node
    def get_current_behavior(self):

        nex == self.get_next_node(curr)
        previous = self.get_previous_node(curr)
        
        if (curr == 0 and nex == 1) or \
           (curr == 1 and nex == 2) or \
           (curr == 6 and nex == 7) or \
           (curr == 7 and nex == 8):

            #TODO: tell the driver to look right for a car while moving forward

            pass

        elif (curr == 11 and nex == 13) or \
             (curr == 4 and nex == 12):

            #TODO: tell the driver to look left for a car while moving forward

            pass

        elif (curr == 4 and nex == 5) or \
             (curr == 6 and nex == 5) or \
             (curr == 9 and nex == 10) or \
             (curr == 11 and nex == 10):

            #TODO: tell the driver to look forward for a crosswalk

            pass

        elif (prev == 2 and curr == 3 and nex == 4) or \
             (prev == 11 and curr == 0 and nex == 1) or \
             (prev == 12 and curr == 4 and nex == 5) or \
             (prev == 10 and curr == 11 and nex == 13) or \
             (prev == 5 and curr == 6 and nex == 7) or \
             (prev == 8 and curr == 9 and nex == 10) or \
             (prev == 3 and curr == 4 and nex == 12) or \
             (prev == 13 and curr == 11 and nex == 0):
        
            #TODO: tell the driver to turn right
        
            pass
        
        elif (prev == 4 and curr == 3 and nex == 2) or \
             (prev == 1 and curr == 0 and nex == 11) or \
             (prev == 5 and curr == 4 and nex == 12) or \
             (prev == 13 and curr == 11 and nex == 10) or \
             (prev == 7 and curr == 6 and nex == 5) or \
             (prev == 10 and curr == 9 and nex == 8) or \
             (prev == 12 and curr == 4 and nex == 3) or \
             (prev == 0 and curr == 11 and nex == 13):

            #TODO: tell the driver to turn left

            pass

        else:
            
            #TODO: tell the driver to continue moving forward


        #TODO: Add the ability to turn 180 when it is needed
        
        return None
