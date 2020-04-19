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

class PlateCNN:
    LABELS = "0123456789?ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    l_ar = []
    model_weights = ??

    # list of instance variables
    # conv_model

    def __init__(self):
        # setup CNN here
        l_ar = [x for x in LABELS]

        # TODO: read pickle

        # set up CNN
        conv_model = models.Sequential()
        conv_model.add(layers.Conv2D(32, (3, 3), activation='relu',
                                    input_shape=(90, 30, 3)))  # from C.shape
        conv_model.add(layers.MaxPooling2D((2, 2)))
        conv_model.add(layers.Conv2D(64, (3, 3), activation='relu'))
        conv_model.add(layers.MaxPooling2D((2, 2)))
        conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
        conv_model.add(layers.MaxPooling2D((2, 2)))
        # conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
        # conv_model.add(layers.MaxPooling2D((2, 2)))
        conv_model.add(layers.Flatten())
        conv_model.add(layers.Dropout(0.5))
        conv_model.add(layers.Dense(512, activation='relu'))
        conv_model.add(layers.Dense(37, activation='softmax'))  # at least i think it is num labels

        # sets learning rate of model, and also type
        LEARNING_RATE = 1e-4
        conv_model.compile(loss='categorical_crossentropy',
                        optimizer=optimizers.RMSprop(lr=LEARNING_RATE),
                        metrics=['acc'])
  

    def train(self):
        [X,Y] = self.generate_train_data()
        [X_v, Y_v] = self.generate_val_data()

        history_conv = conv_model.fit(X, Y, validation_data=(X_v,Y_v), epochs=10, batch_size=32)

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
        path = os.getcwd() + "/" CHANGE THIS CHANGE THIS
        pic_path = path + "???/"

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
        train_generator = train_datagen.flow_from_directory(pic_path+"train", target_size=(90,30), batch_size=82)
        
        X = []
        Y = []

        # the number in range is how many batches of augmented data we want
        for i in range(40):
            batch = train_generator.next()
            image = batch[0].astype('uint8')

            for j in range(82):
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
        val_generator = val_datagen.flow_from_directory(pic_path+"val", target_size=(90,30), batch_size=266)

        X_v = []
        Y_v = []

        for i in range(4):
            batch = val_generator.next()
            image = batch[0].astype('uint8')

            for j in range(188):
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
        pass
    
    def read_pickle(self):
        # get saved values
        pass

    # re-initialize model parameters. not 100% on what this means TODO figure this out
    def reset_weights(self, model):
        session = backend.get_session()
        for layer in model.layers: 
            if hasattr(layer, 'kernel_initializer'):
                layer.kernel.initializer.run(session=session)

if __name__ == '__main__':
    # train cnn here
