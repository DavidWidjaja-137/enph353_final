#!/usr/bin/env python

# this is a modified version of plate_generator.py written by Miti for lab5
import string
import random
from random import randint
import cv2
import numpy as np
import os
from PIL import Image, ImageFont, ImageDraw

path = os.path.dirname(os.path.realpath(__file__)) + "/"

NUMBER_OF_LETTERS = 26
NUMBER_OF_NUMBERS = 10
UPPERCASE_START = 65
NUMBERS_START = 48

blank_list = [f for f in os.listdir(path+'base_images/') if os.path.isfile(os.path.join(path+'base_images/', f))]
print(blank_list)

count = 0
for img_name in blank_list:
    count += 1

    for i in range(0, NUMBER_OF_LETTERS, 4):

        # Pick two letters
        plate_alpha = ""
        for j in range(0, 2):
            plate_alpha += chr(i+UPPERCASE_START+j)

        # Pick two more letters
        plate_alpha2 = "" 
        if (i < 24):
            for j in range(0, 2):
                plate_alpha2 += chr(i+UPPERCASE_START+j+2)
        else:
            plate_alpha2 = 'CG'

        # Write plate to image
        temp = cv2.imread(path+'base_images/{}'.format(img_name))
        blank_plate = cv2.resize(temp, (600,298))

        # Convert into a PIL image (this is so we can use the monospaced fonts)
        blank_plate_pil = Image.fromarray(blank_plate)

        # Get a drawing context
        draw = ImageDraw.Draw(blank_plate_pil)
        monospace = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf", 200)
        draw.text((48, 50),plate_alpha + " " + plate_alpha2, (255,0,0), font=monospace)

        # Convert back to OpenCV image and save
        blank_plate = np.array(blank_plate_pil)

        # Write license plate to file
        cv2.imwrite(os.path.join(path + "imgs_beforesplit/", 
                                    "{}{}_{}{}.png".format(plate_alpha, plate_alpha2, count, i)),
                    blank_plate)

    for i in range(0, NUMBER_OF_NUMBERS, 4):

        # Pick two numbers
        plate_alpha = ""
        for j in range(0, 2):
            plate_alpha += chr(i+NUMBERS_START+j)

        # Pick two more letters
        plate_alpha2 = "" 
        if (i < 8):
            for j in range(0, 2):
                plate_alpha2 += chr(i+NUMBERS_START+j+2)
        else:
            plate_alpha2 = 'CG'

        # Write plate to image
        temp = cv2.imread(path+'base_images/{}'.format(img_name))
        blank_plate = cv2.resize(temp, (600,298))
        
        # Convert into a PIL image (this is so we can use the monospaced fonts)
        blank_plate_pil = Image.fromarray(blank_plate)

        # Get a drawing context
        draw = ImageDraw.Draw(blank_plate_pil)
        monospace = ImageFont.truetype("/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf", 200)
        draw.text((48, 50),plate_alpha + " " + plate_alpha2, (255,0,0), font=monospace)

        # Convert back to OpenCV image and save
        blank_plate = np.array(blank_plate_pil)

        # Write license plate to file
        cv2.imwrite(os.path.join(path + "imgs_beforesplit/", 
                                    "{}{}_{}{}.png".format(plate_alpha, plate_alpha2, count, i)),
                    blank_plate)
