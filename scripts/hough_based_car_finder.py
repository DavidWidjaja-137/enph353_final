#! /usr/bin/env python

# ENPH 353 final project: Self-driving vehicle with license-plate reading
# hough_based_car_finder.py: script which handles finding cars by exploiting
#                           the Hough Line Transform
# Authors:
#   Miles Justice
#   David Widjaja 18950063

import cv2 as cv
import numpy as np

CANNY_HIGH = 200
CANNY_LOW = 25

HOUGH_THRESH = 50

class EdgeBasedCarFinder:

    def __init__(self):

        print("EdgeBasedLineTransform Initialized")

    
    def match_car(self, binary_img):

        #Obtain all the edges using a Canny edge detector
        edges = cv.Canny(binary_img, CANNY_LOW, CANNY_HIGH, apertureSize = 3)

        #Filter and parametrize the edges using Hough Line Transform
        lines = cv.HoughLines(edges, 1, np.pi/180, HOUGH_THRESH)
        
        line_pts = []
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(-a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                line_pts.append( (x1,y1,x2,y2) )

        return line_pts

