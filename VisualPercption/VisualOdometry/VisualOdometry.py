import numpy as np
import cv2
from matplotlib import pyplot as plt
from m2bk import *

np.random.seed(1)
np.set_printoptions(suppress=True)

dataset_handler = DatasetHandler()

#plt.figure(figsize=(8, 6), dpi=100)
#plt.imshow(dataset_handler.images[0], cmap='gray')
#plt.show()
#plt.figure(figsize=(8, 6), dpi=100)
#plt.imshow(dataset_handler.images_rgb[0])
#plt.figure(figsize=(8, 6), dpi=100)
#plt.imshow(dataset_handler.depth_maps[0], cmap='jet')

i = 0
image = dataset_handler.images[i]

### Feature extraction ############################################### 

def extract_features(image):
    """
    Find keypoints and descriptors for the image

    Arguments:
    image -- a grayscale image

    Returns:
    kp_list -- list of the extracted keypoints (features) in an image
    des_list -- list of the keypoint descriptors in an image
    """
    ### START CODE HERE ### 

##### Using sift
#     sift = cv2.xfeatures2d.SIFT_create()
#     kp_list = sift.detect(image, None)
#     kp_list, des_list = sift.compute(image, kp_list)
    
    surf = cv2.xfeatures2d.SURF_create(500) #cv2 version 3.4
    kp_list = surf.detect(image, None)
    kp_list, des_list = surf.compute(image, kp_list)
    ### END CODE HERE ###
    
    return kp_list, des_list

image0_kp_list, image0_des_list = extract_features(dataset_handler.images[0])

print("Number of features detected in frame {0}: {1}\n".format(0, len(image0_kp_list)))
print("Coordinates of the first keypoint in frame {0}: {1}".format(0, str(image0_kp_list[0].pt)))

def visualize_features(image, kp):
    """
    Visualize extracted features in the image

    Arguments:
    image -- a grayscale image
    kp -- list of the extracted keypoints

    Returns:
    """
    display = cv2.drawKeypoints(image, kp, None)
    plt.figure(figsize=(8, 6), dpi=100)
    plt.imshow(display)


# Optional: visualizing and experimenting with various feature descriptors
visualize_features(dataset_handler.images_rgb[0], image0_kp_list)


def extract_features_dataset(images, extract_features_function):
    """
    Find keypoints and descriptors for each image in the dataset

    Arguments:
    images -- a list of grayscale images
    extract_features_function -- a function which finds features (keypoints and descriptors) for an image

    Returns:
    kp_lists -- a list of keypoint_lists for each image in images
    des_lists -- a list of descriptor_lists for each image in images

    """
    kp_lists = []
    des_lists = []

    ### START CODE HERE ###
    for image in images:
        kp_list, des_list = extract_features_function(image)
        kp_lists.append(kp_list)
        des_lists.append(des_list)

    ### END CODE HERE ###

    return kp_lists, des_lists

images = dataset_handler.images
kp_lists, des_lists = extract_features_dataset(images, extract_features)

print("Number of features detected in frame {0}: {1}".format(0, len(kp_lists[0])))
print("Coordinates of the first keypoint in frame {0}: {1}\n".format(0, str(kp_lists[0][0].pt)))

# Remember that the length of the returned by dataset_handler lists should be the same as the length of the image array
print("Length of images array: {0}".format(len(images)))
