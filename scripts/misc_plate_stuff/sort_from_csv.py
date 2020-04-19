#!/usr/bin/env python

import string
import random
from random import randint
import cv2
import numpy as np
import os
import csv
import shutil


PATH = os.path.dirname(os.path.realpath(__file__)) + '/'
PIC_PATH = PATH + "test_imgs_aftersplit/"
CSV_FILE = PATH + 'cnn_stuff/val_labels.csv'

with open(CSV_FILE, 'rb') as csvfile:
    labelreader = csv.reader(csvfile)
    for row in labelreader:
        # get the label
        label = row[1]
        img_name = row[0]
        
        # if no directory for this label, make one
        try:
            os.mkdir(PATH+"cnn_stuff/val/{}/".format(label))
        except:
            print("already did this one")
        
        shutil.copy(os.path.join(PIC_PATH,img_name),PATH+"cnn_stuff/val/{}/".format(label))
