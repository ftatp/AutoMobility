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


def match_features(des_list1, des_list2):
    """
    Match features from two images

    Arguments:
    des_list1 -- list of the keypoint descriptors in the first image
    des_list2 -- list of the keypoint descriptors in the second image

    Returns:
    match_list -- list of matched features from two images. Each match[i] is k or less matches for the same query descriptor
    """
    ### START CODE HERE ###
#     bf = cv2.BFMatcher(cv2.NORM_L2)#, crossCheck=True)
#     match_list = bf.knnMatch(des_list1, des_list2, k=2)

    # FLANN parameters
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)   # or pass empty dictionary

    flann = cv2.FlannBasedMatcher(index_params, search_params)

    match_list = flann.knnMatch(des_list1, des_list2, k=2)
    # match_list = sorted(match, key = lambda x:x.distance)
    ### END CODE HERE ###

    return match_list

i = 0
query_des_list1 = des_lists[i]
train_des_list2 = des_lists[i+1]

match_list = match_features(query_des_list1, train_des_list2)
#match_list = sorted(match, key = lambda x:x.distance)
print("Number of features matched in frames {0} and {1}: {2}".format(i, i+1, len(match_list)))

# Remember that a matcher finds the best matches for EACH descriptor from a query set

# Optional
def filter_match_list_by_distance(matches, dist_threshold):
    """
    Filter matched features from two images by distance between the best matches

    Arguments:
    match -- list of matched features from two images
    dist_threshold -- maximum allowed relative distance between the best matches, (0.0, 1.0)

    Returns:
    filtered_match -- list of good matches, satisfying the distance threshold
    """
    filtered_matches = []

    ### START CODE HERE ###
    for first_feature_point, sec_feature_point in matches:
        if first_feature_point.distance <= dist_threshold * sec_feature_point.distance :
            filtered_matches.append(first_feature_point)

    ### END CODE HERE ###

    return filtered_matches

# Optional
i = 0
query_des_list1 = des_lists[i]
train_des_list2 = des_lists[i+1]
match_list = match_features(query_des_list1, train_des_list2)
print("Number of features matched in frames {0} and {1} before filtering by distance: {2}".format(i, i+1, len(match_list)))

dist_threshold = 0.6
filtered_match_list = filter_match_list_by_distance(match_list, dist_threshold)

print("Number of features matched in frames {0} and {1} after filtering by distance: {2}".format(i, i+1, len(filtered_match_list)))


def visualize_matches(image1, kp_list1, image2, kp_list2, match):
    """
    Visualize corresponding matches in two images

    Arguments:
    image1 -- the first image in a matched image pair
    kp1 -- list of the keypoints in the first image
    image2 -- the second image in a matched image pair
    kp2 -- list of the keypoints in the second image
    match -- list of matched features from the pair of images

    Returns:
    image_matches -- an image showing the corresponding matches on both image1 and image2 or None if you don't use this function
    """
    image_matches = cv2.drawMatches(image1, kp_list1, image2, kp_list2, match, None, flags=2)
    plt.figure(figsize=(16, 6), dpi=100)
    plt.imshow(image_matches)
    plt.show()


# Visualize n first matches, set n to None to view all matches
# set filtering to True if using match filtering, otherwise set to False
n = 20
filtering = True

i = 0
query_image = dataset_handler.images[i]
train_image = dataset_handler.images[i+1]

query_kp_list = kp_lists[i]
train_kp_list = kp_lists[i+1]

query_des_list = des_lists[i]
train_des_list = des_lists[i+1]

match_list = match_features(query_des_list, train_des_list)
if filtering:
    dist_threshold = 0.6
    filtered_match_list = filter_match_list_by_distance(match_list, dist_threshold)

image_matches = visualize_matches(query_image, query_kp_list, train_image, train_kp_list, filtered_match_list[:n])
