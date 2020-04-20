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
#FILTER_MATCH_THRESHOLD = 0.5
FILTER_MATCH_THRESHOLD = 0.7

#0.8 too high for yellow
#0.7 is okay-ish
#0.5 too low for yellow
FILTER_MATCH_THRESHOLD_YELLOW = 0.7

#yellow is weak af
HOMOGRAPHY_THRESHOLD = 3
HOMOGRAPHY_THRESHOLD_YELLOW = 1

#Color codes
BLUE = 0
GREEN = 1
YELLOW = 2

#Homography Thresholds
HOM_LOW = 25
HOM_HIGH = 175

class SIFTBasedCarFinder:

    #instantiate the SIFTBasedCarFinder class
    def __init__(self):
        
        path = os.path.dirname(os.path.realpath(__file__)) + "/"
        
        #Obtain suitable target images
        self.source_green = cv.imread(path+"green_v0.jpg")
        self.source_blue = cv.imread(path+"blue_v0.jpg")
        self.source_yellow = cv.imread(path+"yellow_v2.jpg")
         
        self.source_green = cv.cvtColor(self.source_green, cv.COLOR_BGR2GRAY)
        self.source_blue = cv.cvtColor(self.source_blue, cv.COLOR_BGR2GRAY)
        self.source_yellow = cv.cvtColor(self.source_yellow, cv.COLOR_BGR2GRAY)
        self.h_g, self.w_g = self.source_green.shape
        self.h_b, self.w_b = self.source_blue.shape
        self.h_y, self.w_y = self.source_yellow.shape
        
        #instantiate the sift class, and uses sift to get the keypoints and descriptors of source
        self.sift = cv.xfeatures2d.SIFT_create()
        self.kp_source_g, self.desc_source_g = self.sift.detectAndCompute(self.source_green, None)
        self.kp_source_b, self.desc_source_b = self.sift.detectAndCompute(self.source_blue, None)
        self.kp_source_y, self.desc_source_y = self.sift.detectAndCompute(self.source_yellow, None)
        
        #instantiate the image matcher, which uses the flann algorithm
        self.index_params = dict(algorithm=0, trees=5)
        self.search_params = dict()
        self.flann = cv.FlannBasedMatcher(self.index_params, self.search_params)
       
    #Use the SIFT algorithm to check if a valid license plate is available at the particular
    # location. This algorithm is computationally intensive, so run it sparingly. Other
    # preliminary methods should first be used to check if a license plate may be available at
    # a location.
    # returns: ( (x0, y0), (x1, y1) )
    # x0, y0 represent the top-left corner of the homography
    # x1, y1 represent the bottom-right corner of the homography
    def match_car(self, target_gray, colour):
        
        if colour == GREEN:
            self.source = self.source_green
            self.h = self.h_g
            self.w = self.w_g
            self.kp_source = self.kp_source_g
            self.desc_source = self.desc_source_g
            homography_threshold = HOMOGRAPHY_THRESHOLD
            filter_match_threshold = FILTER_MATCH_THRESHOLD
        elif colour == BLUE:
            self.source = self.source_blue
            self.h = self.h_b
            self.w = self.w_b
            self.kp_source = self.kp_source_b
            self.desc_source = self.desc_source_b
            homography_threshold = HOMOGRAPHY_THRESHOLD
            filter_match_threshold = FILTER_MATCH_THRESHOLD
        else:
            self.source = self.source_yellow
            self.h = self.h_y
            self.w = self.w_y
            self.kp_source = self.kp_source_y
            self.desc_source = self.desc_source_y
            homography_threshold = HOMOGRAPHY_THRESHOLD_YELLOW
            filter_match_threshold = FILTER_MATCH_THRESHOLD_YELLOW
         
        #use sift to get keypoints and descriptors in the frame 
        kp_target, desc_target = self.sift.detectAndCompute(target_gray, None)
         
        #match the descriptors of the target and the descriptors of the frame
        #matches is a list of matching points in the target and descriptor.
        matches = self.flann.knnMatch(self.desc_source, desc_target, k=2) 
        
        #filter only for good matches
        #this is done by cutting out points with abnormally large distances    
        good_pts = []
        for m, n in matches:
            if m.distance < filter_match_threshold*n.distance:
                good_pts.append(m)
         
        #draw the found matches of keypoints from two images 
        #output_matches = cv.drawMatches(self.source_img, self.kp_source, target_gray, kp_target, \
        #                             good_pts, target_gray)
        
	    #Homography
        if len(good_pts) > homography_threshold:
            #if there are this many points, draw homography
             
            #query index gives position of the points in the query image
            #this extracts those points and reshapes it
            query_pts = np.float32([self.kp_source[m.queryIdx].pt \
                                                 for m in good_pts]).reshape(-1,1,2)        
        
            train_pts = np.float32([kp_target[m.trainIdx].pt \
                                                 for m in good_pts]).reshape(-1,1,2)
        
            #obtains the perspective transformation between two sets of points 
            matrix, mask = cv.findHomography(query_pts, train_pts, cv.RANSAC, 5.0)
        
            #if no homography can be found
            if matrix is None:
                
                print("SCENARIO 1 HAPPENED")
                print("DUMP: {}".format(query_pts))

                return None
                #TESTING
                #return (target_gray, output_matches)
 
            else:
                #do a perspective transform to change the orientation of the homography
                # with respect to the original image
                pts = np.float32([[0, 0], [0, self.h], [self.w, self.h], \
                                 [self.w, 0]]).reshape(-1,1,2)
                dst = cv.perspectiveTransform(pts, matrix)            
                dst = np.int32(dst) 

                #Find the maximum and minimum x and y points
                max_x = dst[0][0][0]
                min_x = dst[0][0][0]
                
                max_y = dst[0][0][1]
                min_y = dst[0][0][1]
                
                for i in range(len(dst)):
                    
                    if max_x < dst[i][0][0]:
                        max_x = dst[i][0][0]
                    if min_x > dst[i][0][0]:
                        min_x = dst[i][0][0]

                    if max_y < dst[i][0][1]:
                        max_y = dst[i][0][1]
                    if min_y > dst[i][0][1]:
                        min_y = dst[i][0][1]

                if max_x - min_x < HOM_LOW or max_x - min_x > HOM_HIGH:

                    print("SCENARIO 2 HAPPENED")
                    return None
                    #TESTING
                    #return (target_gray, output_matches)
                elif max_y - min_y < HOM_LOW or max_y - min_y > HOM_HIGH:
                    
                    print("SCENARIO 3 HAPPENED")
                    return None
                    #TESTING
                    #return (target_gray, output_matches)
                else:
                    return ((min_x, min_y), (max_x, max_y)) 
                    #TESTING
                    #dst = [dst]
                    #draw the homography and show it 
                    #homography = cv.polylines(target_gray, [np.int32(dst)], True, (255, 0, 0), 3)
                    #return (homography, output_matches)
        else:
            
            train_pts = np.float32([kp_target[m.trainIdx].pt \
                                                 for m in good_pts]).reshape(-1,1,2)
             
            print("SCENARIO 4 HAPPENED")
            print("DUMP: {}".format(train_pts))
            
            return None
            #TESTING
            #return (target_gray, output_matches)

