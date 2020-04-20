import random
import os
import cv2
import numpy as np

from plate_segmentation import PlateSegmentator
from plate_cnn import PlateCNN
from matplotlib import pyplot as plt

class PlateReader:
    '''
    Call this from main robot code to process an image of a license plate
    needs the pickle file from training the cnn.
    '''

    def __init__(self, pickle_file):
        self.segmentator = PlateSegmentator()
        self.cnn = PlateCNN(picklefile=pickle_file)
    
    def read_plate(self, plate_img):

        # segment plate

        #TEST
        print("cnn reading plate")

        list_of_num_sets = self.segmentator.segment_plate(plate_img)

        #TEST
        #print("num sets len: {}".format(len(list_of_num_sets)))

        pred_vals = []
        pred_strings = []
        for i in range(len(list_of_num_sets)): # these are the spot-plate pairs
            pred_vals.append([])
            pred_strings.append([])

            for j in range(len(list_of_num_sets[i])):  # these are individual spots or plates
                char_group = list_of_num_sets[i][j]
                temp = self.cnn.predict_chars(char_group)
                pred_vals[i].append(temp)

                # convert to string
                st = ''
                for ch in pred_vals[i][j]:
                    print(ch)
                    if ch != '?':
                        st = st + ch
                
                if st != '':
                    pred_strings[i].append(st)
       
            #TEST 
            print(pred_strings[i])

        print("PredStrings: {}".format(pred_strings))
        print("PredVals: {}".format(pred_vals))

        return (pred_vals, pred_strings)


if __name__ == '__main__':
    pr = PlateReader('defaultpickle.pickle')
    im_path = os.path.dirname(os.path.realpath(__file__)) + '/misc_plate_stuff/test_imgs/test109.png'

    img = cv2.imread(im_path)
    # plt.imshow(img)
    # plt.show()
    

    pr.read_plate(img)

