#!/usr/bin/env python

import string
import random
from random import randint
import cv2
import numpy as np
import os
from PIL import Image, ImageFont, ImageDraw

path = os.path.dirname(os.path.realpath(__file__)) + '/'
pic_path = path + "imgs_beforesplit/"
image_names = [f for f in os.listdir(pic_path) if os.path.isfile(os.path.join(pic_path, f))]

# decide limits of characters
text_start = 48  # we know(?) this from plate_generator
chunk = 100 # (w1 - 2*text_start)/5 (cheating bc i did this and its 100, don't want to convert so)

NUM_EXAMPLES = 1

count = -1
for plate_name in image_names:
    count += 1
    plate_img = cv2.imread(pic_path+plate_name)
    h1, w1, _ = plate_img.shape
    
    # separate into four pieces, store each piece in 'characters' and its correspinding label in 'labels_raw'
    # split into 4 parts, each with letter
    j = 0
    for i in range(5):
        # take ith subsec of image (ignore 3rd)
        if i != 2:
            chunk_start = text_start + i*chunk
            chunk_end = chunk_start + chunk
            name = plate_name[j]
            j += 1
            part = plate_img[0:h1, chunk_start:chunk_end]
            try:
                os.mkdir(path+"train/{}/".format(name))
            except:
                # print("already did this one")
                idontwantexceptiontodoanything = True
            
            for k in range(NUM_EXAMPLES):
                cv2.imwrite(os.path.join(path + "train/{}/".format(name), 
                                "{}{}{}.png".format(count, i, k)), part)
            
            '''
            if count == 0:
                try:
                    os.mkdir(pic_path+"val/{}/".format(name))
                except:
                   asdfasdfasdfasdf = 0
                
                for n in range(NUM_EXAMPLES):
                    cv2.imwrite(os.path.join(path + "val/{}/".format(name), 
                                    "{}.png".format(i)), part)
            '''