#David's Remaining Tasks for today

- Do full integration testing of all the components, minus the NN
- Write a script to handle traversing the crosswalks
- Write a script to handle turnbacks
- Integrate crosswalks and turnback into the whole


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

