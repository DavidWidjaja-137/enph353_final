#David's Remaining Tasks for today

##Main Task
- Implement the ability to stop for license plates, take images of license plates, and seperate t    he junk from the license plate.                                                                         
###Breakdown

Method 1: Quick and dirty
1. Color Mask for blue, yellow and green features.
2. Check the right side of the masked images to see if a car is there.
3. Stop if a car is there and take a picture
4. ...
5. No this will be barbaric, because then I will have to extract the pixels of the license plate manually

Method 2: Slightly fancier
1. Obtain an image of the front of a car
2. Produce a Homography of the image using SIFT
3. Match using SIFT features on the track until a car is there.
4. Stop the car if a good position is available to do SIFT.
5. From the output of SIFT, extract the corners of the image which make up the license plates
6. Extract from the raw image the pixels bounded by the corners of the sift algorithm

Method 3: Lazy
1. Threshold the image to reveal only the license plates
2. Use Hough Transform to get a set of lines.
3. Use geometry to find the corners of the lines. 
4. Knowing how the license plates are arranged, deduce what sections of the image are
    license plate
5. Extract

I will give SIFT another try. If that doesnt play out, then I will use Method 3.

Note: SIFT, or the way I am using it, is too finicky to detect the corners of the license plates reliably. So, I will use Method 3 instead.
