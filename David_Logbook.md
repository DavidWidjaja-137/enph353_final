# David's Project Log

### Project Log: Start of Project

**Tasks:**
- implement the ability to stop for license plates, take images of license plates, and seperate the junk from the license plate.                                                                         
- retune PID parameters to reflect new timing changes                   
- allow for some recovery when the robot veers of course AND hits a junction, otherwise it will enter the unknown state
- method of variations is still not working to read changes   

### Project Log: 06 May 2020

**Tasks:**
- Implement the ability to stop for license plates, take images of license plates, and seperate the junk from the license plate.                                                                         

**Method 1: Quick and dirty**
1. Color Mask for blue, yellow and green features.
2. Check the right side of the masked images to see if a car is there.
3. Stop if a car is there and take a picture
4. ...
5. No this will be barbaric, because then I will have to extract the pixels of the license plate manually

**Method 2: Slightly fancier**
1. Obtain an image of the front of a car
2. Produce a Homography of the image using SIFT
3. Match using SIFT features on the track until a car is there.
4. Stop the car if a good position is available to do SIFT.
5. From the output of SIFT, extract the corners of the image which make up the license plates
6. Extract from the raw image the pixels bounded by the corners of the sift algorithm

Conclusion, Method 2 is better for this case.

### Project Log: 10 May 2020

**Method 3: Lazy**
1. Threshold the image to reveal only the license plates
2. Use Hough Transform to get a set of lines.
3. Use geometry to find the corners of the lines. 
4. Knowing how the license plates are arranged, deduce what sections of the image are
    license plate
5. Extract

I will give SIFT another try. If that doesnt play out, then I will use Method 3.

Note: SIFT, or the way I am using it, is too finicky to detect the corners of the license plates reliably. So, I will use Method 3 instead.

Turns out Method 3 is a pain. Method 2 it is then.

### Project Log: 13 May 2020

**Design a system of knowing where the vehicle is at any moment**

Use a sytem of checkpoints
Checkpoints can include:
- turns
- cars
- crosswalks

If a vehicle just completed a turn, advance the location in the data structure
If a vehicle just finished extracting from a car, advance the location in the data structure
If a vehicle just crossed a crosswalk, advance the location in the data structure

**Data structure: A directed graph between all the key points of the track.**

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

**When do we update the state of the state machine?**

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

**How do we handle crosswalks?**

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

**How to prevent the same car from being detected twice?**

If there appears to be a car, set a flag to True
Subsequently, check whether there appears to be a car
If the car is still there
    Keep the flag True. As long as the flag is True, the car cannot use SIFT again
If the car is no longer there
    Keep the flag to False. The car can now use SIFT again

### Project Log: 19 April 2020

**Remaining Issues**

- Sometimes the car just momentarily slips on the track. Not entirely sure why this behavior arises. Its minor and infrequent at best
- Vehicle sometimes has trouble distinguishing between Node 12 and Node 16 due to the same color and close proximity. Might need fixing later on
- Vehicle sometimes has trouble distinguishing between Node 7 and Node 8. Might need some fixing later on.


