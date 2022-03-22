import numpy as np
import cv2
from matplotlib import pyplot as plt
from m6bk import *
import math

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

# Get road mask by choosing pixels in segmentation output with value 7
line_mask = np.zeros(segmentation.shape).astype(np.uint8)
line_mask[segmentation == 6] = 1
line_mask[segmentation == 8] = 1
# Show road mask
#plt.imshow(line_mask)
# line_mask_gray = cv2.cvtColor(line_mask, cv2.COLOR_BGR2GRAY)
# line_mask_blur = cv.blur(line_mask_gray, (3, 3))

edge_detected = cv2.Canny(line_mask, 0, 0, 3)
lines = cv2.HoughLinesP(edge_detected, 1, np.pi / 180, 100)#,0,0)
print(lines.shape)
#print(lines)
lines.reshape((-1, 4))
#plt.imshow(line_detected)

# # Get x,y, and z coordinates of pixels in road mask
# x_ground = x[road_mask == 1]
# y_ground = y[road_mask == 1]
# z_ground = dataset_handler.depth[road_mask == 1]
# xyz_ground = np.stack((x_ground, y_ground, z_ground))

def estimate_lane_lines(segmentation_output):
    """
    Estimates lines belonging to lane boundaries. Multiple lines could correspond to a single lane.

    Arguments:
    segmentation_output -- tensor of dimension (H,W), containing semantic segmentation neural network output
    minLineLength -- Scalar, the minimum line length
    maxLineGap -- Scalar, dimension (Nx1), containing the z coordinates of the points

    Returns:
    lines -- tensor of dimension (N, 4) containing lines in the form of [x_1, y_1, x_2, y_2], where [x_1,y_1] and [x_2,y_2] are
    the coordinates of two points on the line in the (u,v) image coordinate frame.
    """
    ### START CODE HERE ### (≈ 7 lines in total)
    # Step 1: Create an image with pixels belonging to lane boundary categories from the output of semantic segmentation
    line_mask = np.zeros(segmentation.shape).astype(np.uint8)
    line_mask[segmentation == 6] = 255
    line_mask[segmentation == 8] = 255

    # line_mask_gray = cv2.cvtColor(line_mask, cv2.COLOR_BGR2GRAY)
    # line_mask_blur = cv.blur(line_mask_gray, (3, 3))

    edge_detected = cv2.Canny(line_mask, 100, 150)

    # Step 3: Perform Line estimation using cv2.HoughLinesP()
    lines = cv2.HoughLinesP(edge_detected, 10, np.pi / 180, 150, minLineLength=50, maxLineGap=150)
    #lines = cv2.HoughLinesP(edges, rho=10, theta=np.pi/180, threshold=200, minLineLength=150, maxLineGap=50)
    lines = lines.reshape((-1, 4))
    # Note: Make sure dimensions of returned lines is (N x 4)
    ### END CODE HERE ###

    return lines

def merge_lane_lines(lines):
    """
    Merges lane lines to output a single line per lane, using the slope and intercept as similarity measures.
    Also, filters horizontal lane lines based on a minimum slope threshold.

    Arguments:
    lines -- tensor of dimension (N, 4) containing lines in the form of [x_1, y_1, x_2, y_2],
    the coordinates of two points on the line.

    Returns:
    merged_lines -- tensor of dimension (N, 4) containing lines in the form of [x_1, y_1, x_2, y_2],
    the coordinates of two points on the line.
    """
    
    ### START CODE HERE ### (≈ 25 lines in total)
    
    # Step 0: Define thresholds
    slope_similarity_threshold = 0.1
    intercept_similarity_threshold = 40
    min_slope_threshold = 0.3
    
    # Step 1: Get slope and intercept of lines
    slopes, intercepts = get_slope_intecept(lines)
    
    # Step 2: Determine lines with slope less than horizontal slope threshold.
    indice_where_slope_is_bigger_than_thres = np.where(abs(slopes) > min_slope_threshold)
    filtered_lines = lines[indice_where_slope_is_bigger_than_thres]
    slopes = slopes[indice_where_slope_is_bigger_than_thres]
    intercepts = intercepts[indice_where_slope_is_bigger_than_thres]
    #print(slopes.shape)
    # Step 3: Iterate over all remaining slopes and intercepts and cluster lines that are close to each other using a slope and intercept threshold.
    groups = [] #List of groups (dictionary) that have fields average_slope, average_intercept, list of lines
    
    line_id = 0
    for slope, intercept in zip(slopes, intercepts):
        is_included_in_group = False
        for i in range(len(groups)):
            if abs(slope - groups[i]['average_slope']) < 0.1 and abs(intercept - groups[i]['average_intercept']):
                groups[i]['list_of_line_id'].append(line_id)
                groups[i]['average_slope'] = sum(slopes[groups[i]['list_of_line_id']]) / len(slopes[groups[i]['list_of_line_id']])
                groups[i]['average_intercept'] = sum(intercepts[groups[i]['list_of_line_id']]) / len(intercepts[groups[i]['list_of_line_id']])
                is_included_in_group = True
        
        if not is_included_in_group:
            group = {
                'average_slope' : slope,
                'average_intercept' : intercept,
                'list_of_line_id': [line_id]
            }
            groups.append(group)
            
        line_id += 1

    #pp.pprint(groups)
    # Step 4: Merge all lines in clusters using mean averaging
    merged_lines = []
    for group in groups:
        coordinates = np.array(filtered_lines[group['list_of_line_id']]).T
        #pp.pprint(coordinates)
        #pp.pprint([sum(coordinates[0]), sum(coordinates[1]), sum(coordinates[2]), sum(coordinates[3])])
        merged_line = [i / len(group['list_of_line_id']) for i in [sum(coordinates[0]), sum(coordinates[1]), sum(coordinates[2]), sum(coordinates[3])]]
        merged_lines.append(merged_line)
    
    # Note: Make sure dimensions of returned lines is (N x 4)
    merged_lines = np.array(merged_lines)
    ### END CODE HERE ###
    return merged_lines

lane_lines = estimate_lane_lines(segmentation)
merged_lane_lines = merge_lane_lines(lane_lines)
#plt.imshow(dataset_handler.vis_lanes(lane_lines))
#plt.imshow(dataset_handler.vis_lanes(merged_lane_lines))
#plt.show()

max_y = dataset_handler.image.shape[0]
min_y = np.min(np.argwhere(road_mask == 1)[:, 0])

extrapolated_lanes = extrapolate_lines(merged_lane_lines, max_y, min_y)
final_lanes = find_closest_lines(extrapolated_lanes, dataset_handler.lane_midpoint)
plt.imshow(dataset_handler.vis_lanes(final_lanes))
plt.show()

#####################################################################
### Computing Minimum Distance                                    ###
#####################################################################

def filter_detections_by_segmentation(detections, segmentation_output):
    """
    Filter 2D detection output based on a semantic segmentation map.

    Arguments:
    detections -- tensor of dimension (N, 5) containing detections in the form of [Class, x_min, y_min, x_max, y_max, score].

    segmentation_output -- tensor of dimension (HxW) containing pixel category labels.

    Returns:
    filtered_detections -- tensor of dimension (N, 5) containing detections in the form of [Class, x_min, y_min, x_max, y_max, score].

    """
    ### START CODE HERE ### (≈ 20 lines in total)

    # Set ratio threshold:
    ratio_threshold = 0.3  # If 1/3 of the total pixels belong to the target category, the detection is correct.
    filtered_detections = []
    for detection in detections:

        # Step 1: Compute number of pixels belonging to the category for every detection.

        x_min = int(float(detection[1]))
        y_min = int(float(detection[2]))
        x_max = int(float(detection[3]))
        y_max = int(float(detection[4]))
        object_detected_area_segmented = segmentation[y_min:y_max, x_min:x_max]

        # Step 2: Devide the computed number of pixels by the area of the bounding box (total number of pixels).
        object_detected_area_segmented_flatten = object_detected_area_segmented.ravel()
        if detection[0] == 'Car':
            label = 10
        elif detection[0] == 'Pedestrian':
            label = 4
        ratio = float(len(object_detected_area_segmented_flatten[object_detected_area_segmented_flatten == label])) / (object_detected_area_segmented.shape[0] * object_detected_area_segmented.shape[1])
        #print(ratio)
        # Step 3: If the ratio is greater than a threshold keep the detection. Else, remove the detection from the list of detections.
        if ratio > ratio_threshold:
            filtered_detections.append(detection)

    ### END CODE HERE ###

    return filtered_detections


detections = dataset_handler.object_detection
filtered_detections = filter_detections_by_segmentation(detections, segmentation)

#plt.imshow(dataset_handler.vis_object_detection(detections))
plt.imshow(dataset_handler.vis_object_detection(filtered_detections))
plt.show()

def find_min_distance_to_detection(detections, x, y, z):
    """
    Filter 2D detection output based on a semantic segmentation map.

    Arguments:
    detections -- tensor of dimension (N, 5) containing detections in the form of [Class, x_min, y_min, x_max, y_max, score].
    
    x -- tensor of dimension (H, W) containing the x coordinates of every pixel in the camera coordinate frame.
    y -- tensor of dimension (H, W) containing the y coordinates of every pixel in the camera coordinate frame.
    z -- tensor of dimensions (H,W) containing the z coordinates of every pixel in the camera coordinate frame.
    Returns:
    min_distances -- tensor of dimension (N, 1) containing distance to impact with every object in the scene.

    """
    ### START CODE HERE ### (≈ 20 lines in total)
    min_distances = []
    for detection in detections:
        # Step 1: Compute distance of every pixel in the detection bounds
        x_min = int(float(detection[1]))
        y_min = int(float(detection[2]))
        x_max = int(float(detection[3]))
        y_max = int(float(detection[4]))
#         print(x_min, y_min, x_max, y_max)
#         print(x.shape)
        object_detected_area_x = x[y_min:y_max, x_min:x_max]
        object_detected_area_y = y[y_min:y_max, x_min:x_max]
        object_detected_area_z = z[y_min:y_max, x_min:x_max]
    
        min_distance = float("inf")
        # Step 2: Find minimum distance
        for x_, y_, z_ in zip(object_detected_area_x.ravel(), object_detected_area_y.ravel(), object_detected_area_z.ravel()):
            distance = math.sqrt(x_**2 + y_**2 + z_**2)
            if distance < min_distance:
                min_distance = distance
        min_distances.append(distance)
    
    ### END CODE HERE ###
    return min_distances

min_distances = find_min_distance_to_detection(filtered_detections, x, y, z)

print('Minimum distance to impact is: ' + str(min_distances))

font = {'family': 'serif','color': 'red','weight': 'normal','size': 12}

im_out = dataset_handler.vis_object_detection(filtered_detections)

for detection, min_distance in zip(filtered_detections, min_distances):
    bounding_box = np.asfarray(detection[1:5])
    plt.text(bounding_box[0], bounding_box[1] - 20, 'Distance to Impact:' + str(np.round(min_distance, 2)) + ' m', fontdict=font)

plt.imshow(im_out)
plt.show()
