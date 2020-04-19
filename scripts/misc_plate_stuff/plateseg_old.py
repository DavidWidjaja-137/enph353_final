import cv2
import os
import numpy as np
import statistics as sts



def split_images(image_list, horizontal=True, avg_mod=3, t_mod=3.3, min_len=5):
    ''' 
    image_list = a list of images to split. these should be greyscale

    t_mod = to modify the threshold value for greyscaling image. 
                defaults to 3 (best val for first split). 
                probably needs to be changed for other splits
                
    horizontal = True if splitting the image horizontally. default True

    avg_mod = to pull the space-threshold value towards the average pixel value. 
                defaults to 3 based on testing done (most robust)
                
    min_len = the minimum allowed length of chunk (pixels)
        
        
    returns: chunks = a list of lists of indices to split the image with, with:
        len(chunks) = len(image_list)
        chunks[*] = list of pairs corresponding to image[*]
    '''

    chunks = []
    
    for img_g in image_list:
        ''' threshold the image based on average pixel val, then blur '''
        grey_thresh = np.mean(img_g)/t_mod
        _, img_b = cv2.threshold(img_g, grey_thresh, 255, cv2.THRESH_BINARY)
        
        h, w = img_b.shape
        b = 5
        if w < 100:
            b = 2
            
        img = cv2.blur(img_b, (b,b))
        
        ''' get average pix val for each row '''
        pix_vals = []
        
        if horizontal:
            for i in range(h):
                pix_sum = 0
                for j in range(w):
                    pix_sum += img[i,j]
                pix_vals.append(pix_sum/w)
            
        else:
            for i in range(w):
                pix_sum = 0
                for j in range(h):
                    pix_sum += img[j,i]
                pix_vals.append(pix_sum/h)
      

        ''' 
        find where the average vals dip below a certain
        threshold, and count until they rise above that 
        threshold 
        '''
        avg_val = sts.mean(pix_vals)

        pix_vals_temp = pix_vals
        for i in range(50):
            try:
                mode_val = sts.mode(pix_vals_temp)
                break
            except sts.StatisticsError:
                # increase every other thing by one
                for j in range(len(pix_vals_temp)):
                    if j % (i+1) == 0:
                        pix_vals_temp[j] += 1
        
        space_thresh = mode_val - abs(mode_val-avg_val)*(255-avg_val)*(255-avg_val)*(255-avg_val)*avg_mod/(255*255*255)

        temp = []
        i = 0
        keep_going = False
        while i in range(len(pix_vals)):
            if(pix_vals[i] < space_thresh):

                if not keep_going:  # this is for if the previous chunk was determined too small
                    start = i

                while(pix_vals[i] < space_thresh):
                    i += 1
                    if i not in range(len(pix_vals)):
                        break
                        
                ''' 
                check if the chunk is very small. if it is, then
                keep the starting position and keep going 
                '''      
                if(abs(start-i) > 5):  # if its really short, keep going (assume its stopping too early)
                    if(abs(start-i) > min_len): # throw away anything that isn't above minimum length
                        temp.append([start, i])
                    keep_going = False
                    
                else:
                    keep_going = True

            i += 1

        if len(temp) < 1:
            temp = [] # this might already be the case but i just want to be sure lol
        
        chunks.append(temp)
    
    return chunks


# get path to this directory
dirpath = os.path.dirname(os.path.realpath(__file__))
path = dirpath + '/test_imgs/'
print(path)

img_names = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]
test_img_list = []
print(img_names)
count = 0

for n in img_names:
    img = cv2.imread(path + n)
    test_img_list.append(img)


# 1) Convert to greyscale
grey_img_list = []

for test_img in test_img_list:
    grey_img = cv2.cvtColor(test_img, cv2.COLOR_BGR2GRAY)
    grey_img_list.append(grey_img)


# split into strips vertically
chunks = split_images(grey_img_list, horizontal=False, min_len=25)

# 1D list of all vertical strips
new_images = []

for i in range(len(chunks)):
    for chunk in chunks[i]:
        new_images.append(grey_img_list[i][:, chunk[0]:chunk[1]])


# now split each chunk horizontally
    
chunks_2 = split_images(new_images, horizontal=True, t_mod=5, min_len=10)

# a 2D list: a list of pairs of plates (each vertical strip should be two plates)
plate_pairs = []

for i in range(len(chunks_2)):
    plate_pairs.append([])
    for chunk in chunks_2[i]:
        plate_pairs[i].append(new_images[i][chunk[0]:chunk[1], :])


# now take each pair of license plates and vertically split them again

# each split_plates[i] should contain two lists, one of the parking spot (size 2) and one of the plate (size 4)
split_plates = []

for i in range(len(plate_pairs)):
    split_plates.append([])
        
    chunks_3 = split_images(plate_pairs[i], horizontal=False, t_mod=2.5, avg_mod=5)
    
    for j in range(len(chunks_3)):
        split_plates[i].append([])
        
        count = -1
        for chunk in chunks_3[j]:
            count += 1
            temp = plate_pairs[i][j][:, chunk[0]:chunk[1]]
            split_plates[i][j].append(temp)

            # put in folder called test_imgs_aftersplit. this is only
            #      for testing purposes change for actual cnn
            cv2.imwrite(os.path.join(dirpath + "/test_imgs_aftersplit/", 
                                "a{}{}{}.png".format(i, j, count)), temp)

