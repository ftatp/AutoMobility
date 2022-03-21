import numpy as np
import cv2
from matplotlib import pyplot as plt
from m2bk import *

np.random.seed(1)
np.set_printoptions(suppress=True)

dataset_handler = DatasetHandler()

plt.figure(figsize=(8, 6), dpi=100)
plt.imshow(dataset_handler.images[0], cmap='gray')
plt.show()
plt.figure(figsize=(8, 6), dpi=100)
plt.imshow(dataset_handler.images_rgb[0])
plt.figure(figsize=(8, 6), dpi=100)
plt.imshow(dataset_handler.depth_maps[0], cmap='jet')
