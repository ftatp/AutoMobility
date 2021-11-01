import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from Vehicle import Vehicle

time_end = 20
sample_time = 0.01
t_data = np.arange(0, time_end, sample_time)
x_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)

# reset the states
model = Vehicle()

# ==================================
#  Learner solution begins here
# ==================================

# throttle percentage between 0 and 1
throttle = 0.2

# incline angle (in radians)
alpha = 0

for i in range(t_data.shape[0]):
    if model.x < 60:
        alpha = np.arctan(1/20)
    elif model.x >= 60 and model.x < 150 :
        alpha = np.arctan(1/10)
    else : 
        alpha = 0
    
    if i < 500 :
        throttle += 0.06 * sample_time
    elif i >= 500 and i < 1500 :
        throttle = 0.5
    else:
        throttle -= 0.1 * sample_time
        
    v_data[i] = model.v
    x_data[i] = model.x
    model.step(throttle, alpha)

# ==================================
#  Learner solution ends here
# ==================================

# Plot x vs t for visualization
plt.plot(t_data, x_data)
plt.plot(t_data, v_data)
plt.show()
