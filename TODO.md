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



