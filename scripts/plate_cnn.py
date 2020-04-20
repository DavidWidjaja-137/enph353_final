from keras import layers
from keras import models
from keras import optimizers
from keras.utils import plot_model
from keras import backend
from keras.preprocessing.image import ImageDataGenerator
from keras.preprocessing.image import load_img
from keras.preprocessing.image import img_to_array

import numpy as np
from numpy import expand_dims
import random
import cv2
from matplotlib import pyplot as plt
import pickle
import os

class PlateCNN:
    LABELS = "0123456789?ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    l_ar = []
    path = ''
    pic_path = ''

    # list of instance variables
    # conv_model

    def __init__(self, picklefile=None, to_picklefile='defaultpickle.pickle'):
        # setup CNN here
        self.l_ar = [x for x in self.LABELS]
        self.path = os.path.dirname(os.path.realpath(__file__)) + '/misc_plate_stuff/'
        self.pic_path = self.path + "cnn_imgdirs/"
        self.picklefile = picklefile
        self.to_picklefile = to_picklefile

        if self.picklefile is not None:
            print("Reading pickle file")
            # open and read from pickle file
            with open(picklefile) as f:
                self.conv_model = pickle.load(f)


        # set up CNN if no pickle file
        else:
            self.conv_model = models.Sequential()
            self.conv_model.add(layers.Conv2D(32, (3, 3), activation='relu',
                                        input_shape=(90, 30, 3)))  # from C.shape
            self.conv_model.add(layers.MaxPooling2D((2, 2)))
            self.conv_model.add(layers.Conv2D(64, (3, 3), activation='relu'))
            self.conv_model.add(layers.MaxPooling2D((2, 2)))
            self.conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
            self.conv_model.add(layers.MaxPooling2D((2, 2)))
            # self.conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
            # self.conv_model.add(layers.MaxPooling2D((2, 2)))
            self.conv_model.add(layers.Flatten())
            self.conv_model.add(layers.Dropout(0.5))
            self.conv_model.add(layers.Dense(512, activation='relu'))
            self.conv_model.add(layers.Dense(37, activation='softmax'))  # at least i think it is num labels

            # sets learning rate of model, and also type
            LEARNING_RATE = 1e-4
            self.conv_model.compile(loss='categorical_crossentropy',
                            optimizer=optimizers.RMSprop(lr=LEARNING_RATE),
                            metrics=['acc'])
        
        self.conv_model.summary()
  

    def train(self, num_epochs=10):
        [X,Y] = self.generate_train_data()
        [X_v, Y_v] = self.generate_val_data()

        history_conv = self.conv_model.fit(X, Y, validation_data=(X_v,Y_v), epochs=num_epochs, batch_size=32)

        # save the new weights in given pickle file
        self.write_pickle()

        # display accuracy
        plt.plot(history_conv.history['acc'])
        plt.plot(history_conv.history['val_acc'])
        plt.title('model accuracy')
        plt.ylabel('accuracy (%)')
        plt.xlabel('epoch')
        plt.legend(['train accuracy', 'val accuracy'], loc='upper left')
        plt.show()


    def generate_train_data(self):
        '''
        Augments images in labelled directores to generate
        a set of training data.
        return: [X,Y] where X is image data, Y is corresponding labels
        '''
        # generate training data (or read from directory....) and train
        # TODO: probably needs to recieve the directory names for train/val data

        def blur_img(img):
            x = random.randint(3,8)
            return cv2.blur(img,(x,x))

        # make image data generators 
        train_datagen = ImageDataGenerator(
                rotation_range=10,
                brightness_range=(0.4,0.8),
                shear_range=30,
                preprocessing_function=blur_img)

        

        # these take the images from the directories and augment them
        train_generator = train_datagen.flow_from_directory(self.pic_path+"train", target_size=(90,30), batch_size=113)
        # print("TRAIN GENERATOR LEN = {}".format(train_generator.__len__()))
        
        X = []
        Y = []

        # the number in range is how many batches of augmented data we want
        for i in range(40):
            batch = train_generator.next()
            image = batch[0].astype('uint8')

            for j in range(113):
                # get label
                label = batch[1][j].tolist()
                ind = label.index(1)
                temp = cv2.cvtColor(image[j], cv2.COLOR_BGR2GRAY)
                temp = cv2.cvtColor(temp, cv2.COLOR_GRAY2BGR)

                X.append(temp/255.)
                Y.append(label)

        X = np.array(X)
        Y = np.array(Y)
        return [X,Y]     


    def generate_val_data(self):
        # same thing for validation data
        # TODO: need to get directory path or smth
        val_datagen = ImageDataGenerator()
        val_generator = val_datagen.flow_from_directory(self.pic_path+"val", target_size=(90,30), batch_size=283)

        X_v = []
        Y_v = []

        for i in range(4):
            batch = val_generator.next()
            image = batch[0].astype('uint8')

            for j in range(283):
                # get label
                label = batch[1][j].tolist()
                ind = label.index(1)
                temp = cv2.cvtColor(image[j], cv2.COLOR_BGR2GRAY)
                temp = cv2.cvtColor(temp, cv2.COLOR_GRAY2BGR)
                
                X_v.append(temp/255.)
                Y_v.append(label)

        X_v = np.array(X_v)
        Y_v = np.array(Y_v)
        return [X_v, Y_v]

    def write_pickle(self):
        # save values
        print("Pickling")
        with open(self.to_picklefile, 'wb') as f:
            pickle.dump(self.conv_model, f)

    # re-initialize model parameters. not 100% on what this means TODO figure this out
    def reset_weights(self, model):
        session = backend.get_session()
        for layer in model.layers: 
            if hasattr(layer, 'kernel_initializer'):
                layer.kernel.initializer.run(session=session)

    def get_summary(self):
        self.conv_model.summary()

    def predict_chars(self, list_of_images):
        # run images thru cnn
        im_l = np.array(list_of_images)
        pred_vals = self.conv_model.predict(im_l)
        pred_indices = np.argmax(pred_vals, axis=1)

        # pred_labels = []
        # for i in pred_vals:
        #     pred_labels.append(self.l_ar[i])
        pred_labels = [self.l_ar[i] for i in pred_indices]
        return pred_labels

if __name__ == '__main__':
    # train cnn here
    # p_cnn = PlateCNN(picklefile='defaultpickle.pickle')
    p_cnn = PlateCNN()
    p_cnn.get_summary()

    p_cnn.train(10)

