#! /usr/bin/env python

# ENPH 353 final project: Self-driving vehicle with license-plate reading
# sift_based_car_finder.py: script which handles the finding of cars and extracting license plate
#                images from said cars.
# Authors:
#   Miles Justice
#   David Widjaja 18950063

import math
import os
import sys

import cv2 as cv
import numpy as np

#0.8 is too high
#0.6 is too high
#0.5 is okay
#0.45 is too low
#0.4 is too low
FILTER_MATCH_THRESHOLD = 0.5

#5 is too low
#10 is still pretty bad
HOMOGRAPHY_THRESHOLD = 3

class SIFTBasedCarFinder:

    def __init__(self):
        
        path = os.path.dirname(os.path.realpath(__file__)) + "/"

        #Obtain suitable target image
        self.source_img = cv.imread(path+"target_image_1_v6.jpg")
        self.source_img = cv.cvtColor(self.source_img, cv.COLOR_BGR2GRAY)
        self.h, self.w = self.source_img.shape
        
        #instantiate the sift class, and uses sift to get the keypoints and descriptors of source
        self.sift = cv.xfeatures2d.SIFT_create()
        self.kp_source, self.desc_source = self.sift.detectAndCompute(self.source_img, None)
        
        #instantiate the image matcher, which uses the flann algorithm
        self.index_params = dict(algorithm=0, trees=5)
        self.search_params = dict()
        self.flann = cv.FlannBasedMatcher(self.index_params, self.search_params)

    def match_car(self, target_gray):
        
        #use sift to get keypoints and descriptors in the frame 
        kp_target, desc_target = self.sift.detectAndCompute(target_gray, None)

        #match the descriptors of the target and the descriptors of the frame
        #matches is a list of matching points in the target and descriptor.
        matches = self.flann.knnMatch(self.desc_source, desc_target, k=2) 

        #filter only for good matches
        #this is done by cutting out points with abnormally large distances    
        good_pts = []
        for m, n in matches:
            if m.distance < FILTER_MATCH_THRESHOLD*n.distance:
                good_pts.append(m)
   
        #draw the found matches of keypoints from two images 
        output_matches = cv.drawMatches(self.source_img, self.kp_source, target_gray, kp_target, \
                                     good_pts, target_gray)

	    #Homography
        if len(good_pts) > HOMOGRAPHY_THRESHOLD:
            #if there are this many points, draw homography

            #query index gives position of the points in the query image
            #this extracts those points and reshapes it
            query_pts = np.float32([self.kp_source[m.queryIdx].pt \
                                                 for m in good_pts]).reshape(-1,1,2)        

            train_pts = np.float32([kp_target[m.trainIdx].pt \
                                                 for m in good_pts]).reshape(-1,1,2)

            #obtains the perspective transformation between two sets of points 
            matrix, mask = cv.findHomography(query_pts, train_pts, cv.RANSAC, 5.0)

            #if no homography can be found, keep going
            if matrix is None:
                return (target_gray, output_matches)
 
            else:
                #do a perspective transform to change the orientation of the homography
                # with respect to the original image
                pts = np.float32([[0, 0], [0, self.h], [self.w, self.h], \
                                 [self.w, 0]]).reshape(-1,1,2)
                dst = cv.perspectiveTransform(pts, matrix)            
                
                print("Good Points: {}".format(len(good_pts)))
                #draw the homography and show it 
                homography = cv.polylines(target_gray, [np.int32(dst)], True, (255, 0, 0), 3)
                
                return (homography, output_matches)

        else:
            return (target_gray, output_matches)


