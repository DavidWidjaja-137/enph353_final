#! /usr/bin/env python

# ENPH 353 final project: Self-driving vehicle with license-plate reading
# car_finder.py: script which handles the finding of cars and extracting license plate
#                images from said cars.
# Authors:
#   Miles Justice
#   David Widjaja 18950063

import math

import cv2 as cv
import numpy as np

FILTER_MATCH_THRESHOLD = 0.6
HOMOGRAPHY_THRESHOLD = 5

def match_car(grayframe):

    #use sift to get keypoints and descriptors in the frame 
    kp_grayframe, desc_grayframe = sift.detectAndCompute(grayframe, None)

    #match the descriptors of the target and the descriptors of the frame
    #matches is a list of matching points in the target and descriptor.
    matches = flann.knnMatch(desc_image, desc_grayframe, k=2) 

    #filter only for good matches
    #this is done by cutting out points with abnormally large distances    
    good_pts = []
    for m, n in matches:
        if m.distance < FILTER_MATCH_THRESHOLD*n.distance:
            good_pts.append(m)
   
    #draw the found matches of keypoints from two images 
    output_matches = cv.drawMatches(img, kp_image, grayframe, kp_grayframe, \
                                     good_pts, grayframe)
    cv2.imshow("output", output_matches) 
    
	#Homography
    if len(good_pts) > HOMOGRAPHY_THRESHOLD:
        #if there are this many points, draw homography

        #query index gives position of the points in the query image
        #this extracts those points and reshapes it
        query_pts = np.float32([kp_image[m.queryIdx].pt for m in good_pts]).reshape(-1,1,2)        
        train_pts = np.float32([kp_grayframe[m.trainIdx].pt for m in good_pts]).reshape(-1,1,2)

        #obtains the perspective transformation between two sets of points 
        matrix, mask = cv.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)

        #if no homography can be found, keep going
        if matrix is None:
            cv.imshow("Homography", grayframe)
 
        else:
            #do a perspective transform to change the orientation of the homography
            # with respect to the original image
            h, w = img.shape
            pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1,1,2)
            dst = cv.perspectiveTransform(pts, matrix)            

            #draw the homography and show it 
            homography = cv.polylines(grayframe, [np.int32(dst)], True, (255, 0, 0), 3)
            cv.imshow("Homography", homography)

    else:
        cv.imshow("Homography", grayframe)

    cv.waitKey(1)

#TODO: Obtain suitable target image
img = cv.imread("image.jpg", cv2.IMREAD_GRAYSCALE)
cv2.imshow('target', img)

#instantiate the sift class, and uses sift to get the keypoints and descriptors of target
sift = cv.xfeatures2d.SIFT_create()
kp_image, desc_image = sift.detectAndCompute(img, None)

#instantiate the image matcher, which uses the flann algorithm
index_params = dict(algorithm=0, trees=5) #wtf
search_params = dict()
flann = cv.FlannBasedMatcher(index_params, search_params)

