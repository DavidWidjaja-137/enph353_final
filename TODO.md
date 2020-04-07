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

Conclusion, Method 2 is better for this case.
