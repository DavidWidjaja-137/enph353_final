#!/usr/bin/env python

import string
import random
from random import randint
import cv2
import numpy as np
import os
from PIL import Image, ImageFont, ImageDraw

path = os.path.dirname(os.path.realpath(__file__)) + '/'

label_list = [0,1,2,3,4,5,6,7,8,9,'Q','W','E','R','T','Y','U','I','O','P','A','S','D','F','G','H','J','K','L','Z','X','C','V','B','N','M']

for label in label_list:
    try:
        os.mkdir(path+"val/{}/".format(label))
    except:
        print("already did this one")