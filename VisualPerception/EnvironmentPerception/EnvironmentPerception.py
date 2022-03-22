import numpy as np
import cv2
from matplotlib import pyplot as plt
from m6bk import *

np.random.seed(1)
np.set_printoptions(suppress=True)


#####################################################################
### Variable Settings                                             ### 
#####################################################################
dataset_handler = DatasetHandler()

image = dataset_handler.image
depth = dataset_handler.depth
segmentation = dataset_handler.segmentation
colored_segmentation = dataset_handler.vis_segmentation(segmentation)

#plt.imshow(image)
#plt.imshow(depth, cmap='jet')
#plt.imshow(segmentation)
#plt.imshow(colored_segmentation)

#####################################################################
### Drivable Space Estimation Using Semantic Segmentation Output  ###
#####################################################################

def xy_from_depth(depth, k):
    """
    Computes the x, and y coordinates of every pixel in the image using the depth map and the calibration matrix.

    Arguments:
    depth -- tensor of dimension (H, W), contains a depth value (in meters) for every pixel in the image.
    k -- tensor of dimension (3x3), the intrinsic camera matrix

    Returns:
    x -- tensor of dimension (H, W) containing the x coordinates of every pixel in the camera coordinate frame.
    y -- tensor of dimension (H, W) containing the y coordinates of every pixel in the camera coordinate frame.
    """
    # Get the shape of the depth tensor
    shape = depth.shape
    x = np.zeros(shape)
    y = np.zeros(shape)
    
    # Grab required parameters from the K matrix
    f = k[0][0]
    c_u = k[0][2]
    c_v = k[1][2]
    
    # Generate a grid of coordinates corresponding to the shape of the depth map
    for v in range(shape[0]):
        for u in range(shape[1]):
            z = depth[v][u]
            x[v, u] = ((u + 1 - c_u) * z) / f
            y[v, u] = ((v + 1 - c_v) * z) / f
            
    # Compute x and y coordinates
    
    ### END CODE HERE ###
    
    return x, y

dataset_handler.set_frame(0)

k = dataset_handler.k
z = dataset_handler.depth
x, y = xy_from_depth(z, k)

# Get road mask by choosing pixels in segmentation output with value 7
road_mask = np.zeros(segmentation.shape)
road_mask[segmentation == 7] = 1

# Show road mask
#plt.imshow(road_mask)
#plt.show()

# Get x,y, and z coordinates of pixels in road mask
x_ground = x[road_mask == 1]
y_ground = y[road_mask == 1]
z_ground = dataset_handler.depth[road_mask == 1]
xyz_ground = np.stack((x_ground, y_ground, z_ground))

random_indice = np.random.choice(xyz_ground.shape[1], 3)
random_points = xyz_ground.T[random_indice]

#print(xyz_ground[:, random_indice])
#print(random_points.T)
#print(random_points[0][0])

# splited_random_points = np.split(random_points, [2] ,axis=1)
# A = np.split(random_points, [2], axis=1)[0]
# A = np.hstack((A, np.array([[-1.0, -1.0, -1.0]]).T))
# B = -1 * splited_random_points[1]

# p = np.linalg.inv(A.T.dot(A)).dot(A.T).dot(B)
# print(p)
p = compute_plane(random_points.T)
#print(p[0], p[1], p[2])


# distance = dist_to_plane(p, xyz_ground[0].T, xyz_ground[1].T, xyz_ground[2, :].T)
# print(distance.shape)

def ransac_plane_fit(xyz_data):
    """
    Computes plane coefficients a,b,c,d of the plane in the form ax+by+cz+d = 0
    using ransac for outlier rejection.

    Arguments:
    xyz_data -- tensor of dimension (3, N), contains all data points from which random sampling will proceed.
    num_itr -- 
    distance_threshold -- Distance threshold from plane for a point to be considered an inlier.

    Returns:
    p -- tensor of dimension (1, 4) containing the plane parameters a,b,c,d
    """
    
    ### START CODE HERE ### (≈ 23 lines in total)
    
    # Set thresholds:
    num_itr = 50  # RANSAC maximum number of iterations
    min_num_inliers = int(0.9 * xyz_data.shape[1]) # RANSAC minimum number of inliers
    distance_threshold = 0.001  # Maximum distance from point to plane for point to be considered inlier
    
    selected_plane_num_of_inliers = 0
    
    for i in range(num_itr):
        # Step 1: Choose a minimum of 3 points from xyz_data at random.
        random_indice = np.random.choice(xyz_data.shape[1], 3)
        random_points = xyz_data.T[random_indice]
        
        # Step 2: Compute plane model
#         splited_random_points = np.split(random_points, [2] ,axis=1)
#         A = np.split(random_points, [2], axis=1)[0]
#         A = np.hstack((A, np.array([[-1.0, -1.0, -1.0]]).T))
#         B = -1 * splited_random_points[1]
        
#         p = np.linalg.inv(A.T@A).dot(A.T).dot(B).T
        
        p = compute_plane(random_points.T)
    
        # Step 3: Find number of inliers
        num_of_inliers = 0
        inlier_points = []
#         for j in range(xyz_data.shape[1]):
#             distance = dist_to_plane(p, xyz_data[0][j], xyz_data[1][j], xyz_data[2][j])
#             if distance < distance_threshold:
#                 num_of_inliers += 1
#                 inlier_points.append(i)
                
        distance_list = dist_to_plane(p, xyz_data[0].T, xyz_data[1].T, xyz_data[2].T)
        num_of_inliers = len(distance_list[distance_list < distance_threshold])
#         print("iter: ", i, " ", float(num_of_inliers / xyz_ground.shape[1]))
        
        # Step 4: Check if the current number of inliers is greater than all previous iterations and keep the inlier set with the largest number of points.
        if selected_plane_num_of_inliers < num_of_inliers:
            selected_plane_num_of_inliers = num_of_inliers
            inlier_points = np.where(distance_list < distance_threshold)
        
        # Step 5: Check if stopping criterion is satisfied and break.         
        if selected_plane_num_of_inliers >= min_num_inliers:
            break
        
    # Step 6: Recompute the model parameters using largest inlier set.         
    output_plane = compute_plane(xyz_data.T[inlier_points].T)
    
    ### END CODE HERE ###
    
    return output_plane 

p_final = ransac_plane_fit(xyz_ground)
print('Ground Plane: ' + str(p_final))

# Check all pixels world position's destination with the road plane
dist = np.abs(dist_to_plane(p_final, x, y, z)) # x, y, z is the whole img world coordinates
ground_mask = np.zeros(dist.shape)

ground_mask[dist < 0.1] = 1
ground_mask[dist > 0.1] = 0

#plt.imshow(ground_mask)
#plt.show()

dataset_handler.plot_free_space(ground_mask)

#####################################################################
### Lane Estimation Using The Semantic Segmentation Output        ###
#####################################################################


