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


######################################################################
###     Feature extraction                                         ###
######################################################################

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
#visualize_features(dataset_handler.images_rgb[0], image0_kp_list)


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


######################################################################
###     Feature matching                                           ###
######################################################################

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

#image_matches = visualize_matches(query_image, query_kp_list, train_image, train_kp_list, filtered_match_list[:n])

def match_features_dataset(des_lists, match_features):
    """
    Match features for each subsequent image pair in the dataset

    Arguments:
    des_list -- a list of descriptors for each image in the dataset
    match_features -- a function which maches features between a pair of images

    Returns:
    matches -- list of matches for each subsequent image pair in the dataset.
               Each matches[i] is a list of matched features from images i and i + 1

    """
    match_lists = []

    ### START CODE HERE ###
    for i in range(len(des_lists) - 1):
        match_lists.append(match_features(des_lists[i], des_lists[i+1]))

    ### END CODE HERE ###

    return match_lists

match_lists = match_features_dataset(des_lists, match_features)

i = 0
print("Number of features matched in frames {0} and {1}: {2}".format(i, i+1, len(match_lists[i])))


# Optional
def filter_matches_dataset(filter_match_list_by_distance, match_lists, dist_threshold):
    """
    Filter matched features by distance for each subsequent image pair in the dataset

    Arguments:
    filter_matches_distance -- a function which filters matched features from two images by distance between the best matches
    matches -- list of matches for each subsequent image pair in the dataset.
               Each matches[i] is a list of matched features from images i and i + 1
    dist_threshold -- maximum allowed relative distance between the best matches, (0.0, 1.0)

    Returns:
    filtered_matches -- list of good matches for each subsequent image pair in the dataset.
                        Each matches[i] is a list of good matches, satisfying the distance threshold

    """
    filtered_match_lists = []

    ### START CODE HERE ###
    for match_list in match_lists:
        filtered_match_list = filter_match_list_by_distance(match_list, dist_threshold)
        filtered_match_lists.append(filtered_match_list)

    ### END CODE HERE ###

    return filtered_match_lists

# Optional
dist_threshold = 0.6

filtered_match_lists = filter_matches_dataset(filter_match_list_by_distance, match_lists, dist_threshold)

# if len(filtered_matches) > 0:
    
#     # Make sure that this variable is set to True if you want to use filtered matches further in your assignment
#     is_main_filtered_m = True
#     if is_main_filtered_m: 
#         match_lists = filtered_match_lists

#     i = 0
#     print("Number of filtered matches in frames {0} and {1}: {2}".format(i, i+1, len(filtered_match_lists[i])))

print("Number of matches in frames {0} and {1}: {2}".format(i, i+1, len(match_lists[i])))
print("Number of filtered matches in frames {0} and {1}: {2}".format(i, i+1, len(filtered_match_lists[i])))


######################################################################
###     Trajectory Estimation                                      ###
######################################################################

def estimate_motion(match_list, kp_list1, kp_list2, k, depth1=None):
    """
    Estimate camera motion from a pair of subsequent image frames

    Arguments:
    match -- list of matched features from the pair of images
    kp1 -- list of the keypoints in the first image
    kp2 -- list of the keypoints in the second image
    k -- camera calibration matrix 
    
    Optional arguments:
    depth1 -- a depth map of the first frame. This argument is not needed if you use Essential Matrix Decomposition

    Returns:
    rmat -- recovered 3x3 rotation numpy matrix
    tvec -- recovered 3x1 translation numpy vector
    image1_points -- a list of selected match coordinates in the first image. image1_points[i] = [u, v], where u and v are 
                     coordinates of the i-th match in the image coordinate system
    image2_points -- a list of selected match coordinates in the second image. image1_points[i] = [u, v], where u and v are 
                     coordinates of the i-th match in the image coordinate system
               
    """
    rmat = np.eye(3)
    tvec = np.zeros((3, 1))
    image1_points = []
    image2_points = []
    object_points = []
    
    ### START CODE HERE ###
    for m in match_list:
        query_idx = m.queryIdx
        train_idx = m.trainIdx

        # get first img matched keypoints
        p1_x, p1_y = kp_list1[query_idx].pt
    
        ### solvePnp ###############################################################
        #print(kp_list1[query_idx].pt)
        depth = depth1[int(p1_y), int(p1_x)]
        
        if depth < 900:
            image1_point = [int(p1_x), int(p1_y)]
            image1_points.append(image1_point)
    
            World_Coord = [image1_point[0] * depth, image1_point[1] * depth,  depth]
            Camera_coord = np.dot(np.linalg.inv(k), np.array(World_Coord))#
            #print(Camera_coord)
            object_points.append(Camera_coord)
          
            # get second img matched keypoints
            p2_x, p2_y = kp_list2[train_idx].pt
            image2_points.append([int(p2_x), int(p2_y)])
       #print(kp_list2[train_idx].pt)
    
    _, rvec, tvec, _ = cv2.solvePnPRansac(np.float32(object_points), np.float32(image2_points), k, None, iterationsCount=10000, flags=cv2.SOLVEPNP_EPNP)
    rmat, jac = cv2.Rodrigues(rvec)
        
        ### essential matrix #########################################################
    
#         p2_x, p2_y = kp_list2[train_idx].pt
#         image2_points.append([int(p2_x), int(p2_y)])
#
#     # essential matrix
#     E, mask = cv2.findEssentialMat(np.array(image1_points), np.array(image2_points), k)
#     _, rmat, tvec, mask = cv2.recoverPose(E, np.array(image1_points), np.array(image2_points), dataset_handler.k)
    
    ### END CODE HERE ###
    
    return rmat, tvec, image1_points, image2_points

i = 48
match_list = filtered_match_lists[i]
query_kp_list = kp_lists[i]
train_kp_list = kp_lists[i+1]
k = dataset_handler.k
depth = dataset_handler.depth_maps[i]

rmat, tvec, image1_points, image2_points = estimate_motion(match_list, query_kp_list, train_kp_list, k, depth1=depth)

print("Estimated rotation:\n {0}".format(rmat))
print("Estimated translation:\n {0}".format(tvec))


query_image  = dataset_handler.images_rgb[i]
train_image = dataset_handler.images_rgb[i + 1]

#image_move = visualize_camera_movement(query_image, image1_points, train_image, image2_points)
#plt.figure(figsize=(16, 12), dpi=100)
#plt.imshow(image_move)
#plt.show()

def estimate_trajectory(estimate_motion, filtered_match_lists, kp_list, k, depth_maps=[]):
    """
    Estimate complete camera trajectory from subsequent image pairs

    Arguments:
    estimate_motion -- a function which estimates camera motion from a pair of subsequent image frames
    matches -- list of matches for each subsequent image pair in the dataset.
               Each matches[i] is a list of matched features from images i and i + 1
    des_list -- a list of keypoints for each image in the dataset
    k -- camera calibration matrix

    Optional arguments:
    depth_maps -- a list of depth maps for each frame. This argument is not needed if you use Essential Matrix Decomposition

    Returns:
    trajectory -- a 3xlen numpy array of the camera locations, where len is the lenght of the list of images and
                  trajectory[:, i] is a 3x1 numpy vector, such as:

                  trajectory[:, i][0] - is X coordinate of the i-th location
                  trajectory[:, i][1] - is Y coordinate of the i-th location
                  trajectory[:, i][2] - is Z coordinate of the i-th location

                  * Consider that the origin of your trajectory cordinate system is located at the camera position
                  when the first image (the one with index 0) was taken. The first camera location (index = 0) is geven
                  at the initialization of this function

    """
    trajectory = np.zeros((3, 1))

    ### START CODE HERE ###
    RT = np.identity(4)
    for i in range(len(filtered_match_lists)):
        match_list = filtered_match_lists[i]
        query_kp_list = kp_lists[i]
        train_kp_list = kp_lists[i + 1]
        depth = depth_maps[i]
        rmat, tvec, image1_points, image2_points = estimate_motion(match_list, query_kp_list, train_kp_list, k, depth1=depth)

        rt_i = np.hstack([rmat, tvec])
        rt_i = np.vstack([rt_i, np.zeros([1, 4])])
        rt_i[-1, -1] = 1

#       https://docs.opencv.org/3.4.3/d9/dab/tutorial_homography.html
        rt_i_inv = np.linalg.inv(rt_i)

        RT = np.dot(RT, rt_i_inv)
        camera_location = np.array(RT[:3, 3]).reshape(3,1)
        trajectory = np.hstack([trajectory, camera_location])

    ### END CODE HERE ###

    return trajectory

depth_maps = dataset_handler.depth_maps
trajectory = estimate_trajectory(estimate_motion, filtered_match_lists, kp_lists, k, depth_maps=depth_maps)
print(trajectory.shape)
i = 1
print("Camera location in point {0} is: \n {1}\n".format(i, trajectory[:, [i]]))

# Remember that the length of the returned by trajectory should be the same as the length of the image array
print("Length of trajectory: {0}".format(trajectory.shape[1]))

# Part 1. Features Extraction
images = dataset_handler.images
kp_list, des_list = extract_features_dataset(images, extract_features)


# Part II. Feature Matching
matches = match_features_dataset(des_list, match_features)

# Set to True if you want to use filtered matches or False otherwise
is_main_filtered_m = True
if is_main_filtered_m:
    dist_threshold = 0.75
    filtered_matches = filter_matches_dataset(filter_match_list_by_distance, matches, dist_threshold)
    matches = filtered_matches


# Part III. Trajectory Estimation
depth_maps = dataset_handler.depth_maps
trajectory = estimate_trajectory(estimate_motion, matches, kp_list, k, depth_maps=depth_maps)


#!!! Make sure you don't modify the output in any way
# Print Submission Info
#print("Trajectory X:\n {0}".format(trajectory[0,:].reshape((1,-1))))
#print("Trajectory Y:\n {0}".format(trajectory[1,:].reshape((1,-1))))
#print("Trajectory Z:\n {0}".format(trajectory[2,:].reshape((1,-1))))

visualize_trajectory(trajectory)
