#David's Remaining Tasks for today

###Design a system of knowing where the vehicle is at any moment

Use a sytem of checkpoints
Checkpoints can include:
- turns
- cars
- crosswalks

If a vehicle just completed a turn, advance the location in the data structure
If a vehicle just finished extracting from a car, advance the location in the data structure
If a vehicle just crossed a crosswalk, advance the location in the data structure

Data structure: A directed graph between all the key points of the track. 

Implement the directed graph as a matrix. The start and final positions of any transition together represents a directed edge. In the code, specific behavior can be turned for various edges.


3->4: turn to turn
turn 3 has been executed
turn 4 is found.
Node 4 is entered

In Node 4:
    Lock in position until the node-specific behavior is executed.
    4->5, 4->12,4->3 is possible
    Lookup the place to go using an array
    Execute different behavior based on which turn is selected.
    Node-specific behavior has been executed, unlock the node
    When the turn is executed.
    
represent graph using a matrix

##############################################################
When do we update the state of the state machine?
##############################################################

- when the car finishes a turn, unlock the state lock, but do not change state^^
- when the car finishes reading a car, unlock the state lock, but do not change state^^
- when the car finishes crossing the crosswalk, unlock the state lock, but do not change state
Final: locked = False

- when the car is processing a state, don't change anything

- when the car detects a turn, update the state, and lock the state^^
- when the car detects a car, update the state, and lock the state^^^
- when the car detects a crosswalk, update the state, and lock the state
UPDATE STATE
Final: locked = True

- when the car is on the road, don't change anything

The lock tells us where the car is inside or outside a node 



#############################################################
How do we handle crosswalks?
############################################################

- car detects right turn
- state machine tells car to go forward
- car moves forward
- car detects right turn
- state machine tells car to go forward again(Note: must change state machine)
- car detects crosswalk
- state machine tells car to stop, check for guy, go forward, stop.

Note, make sure the state machine can do both
right turn -> right turn -> crosswalk
right turn -> crosswalk

crosswalk -> right turn -> right turn
crosswalk -> right turn
