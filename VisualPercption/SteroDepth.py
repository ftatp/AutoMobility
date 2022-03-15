import numpy as np
import cv2
from matplotlib import pyplot as plt
from matplotlib import patches
import files_management

#%matplotlib inline
#%load_ext autoreload
#%autoreload 2
#%precision %.2f

img_left = files_management.read_left_image()
img_right = files_management.read_right_image()

# Use matplotlib to display the two images
#_, image_cells = plt.subplots(1, 2, figsize=(20, 20))
#image_cells[0].imshow(img_left)
#image_cells[0].set_title('left image')
#image_cells[1].imshow(img_right)
#image_cells[1].set_title('right image')
#plt.show()

# Read the calibration
p_left, p_right = files_management.get_projection_matrices()

# Use regular numpy notation instead of scientific one
np.set_printoptions(suppress=True)

print("p_left \n", p_left)
print("\np_right \n", p_right)

def compute_left_disparity_map(img_left, img_right):

    ### START CODE HERE ###
    num_disparities = 6*16
    block_size = 11

    min_disparity = 0
    window_size = 6

    img_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
    img_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)


    # Stereo BM matcher
    left_matcher_BM = cv2.StereoBM_create(
        numDisparities = num_disparities,
        blockSize = block_size
    )

    # Stereo SGBM matcher
    left_matcher_SGBM = cv2.StereoSGBM_create(
        minDisparity = min_disparity,
        numDisparities = num_disparities,
        blockSize = block_size,
        P1 = 8 * 3 * window_size ** 2,
        P2 = 32 * 3 * window_size ** 2,
        mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    disp_left = left_matcher_SGBM.compute(img_left, img_right).astype(np.float32)/16

    ### END CODE HERE ###

    return disp_left

disp_left = compute_left_disparity_map(img_left, img_right)

# Show the left disparity map
#plt.figure(figsize=(10, 10))
#plt.imshow(disp_left)
#plt.show()

def decompose_projection_matrix(p):

    ### START CODE HERE ###
    decomposedMatrice = cv2.decomposeProjectionMatrix(p)

    k = decomposedMatrice[0]
    r = decomposedMatrice[1]
    t = decomposedMatrice[2]
    t /= t[3]

    ### END CODE HERE ###

    return k, r, t

# Decompose each matrix
k_left, r_left, t_left = decompose_projection_matrix(p_left)
k_right, r_right, t_right = decompose_projection_matrix(p_right)

t_left /= t_left[3]
t_right /= t_right[3]
# Display the matrices
print("k_left \n", k_left)
print("\nr_left \n", r_left)
print("\nt_left \n", t_left)
print("\nk_right \n", k_right)
print("\nr_right \n", r_right)
print("\nt_right \n", t_right)
