import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from Kinematic_BicycleModel import Bicycle


model = Bicycle()
sample_time = 0.01
time_end = 60

t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)

# maintain velocity at 4 m/s
v_data = np.zeros_like(t_data)
v_data[:] = 4 

w_data = np.zeros_like(t_data)

# ==================================
#  Square Path: set w at corners only
# ==================================
# w_data[650:750] = 0.753
# w_data[750:850] = -0.753
# w_data[2150:2250] = 0.753
# w_data[2250:2350] = -0.753
# w_data[3650:3750] = 0.753
# w_data[3750:3850] = -0.753
# w_data[5150:5250] = 0.753
# w_data[5250:5350] = -0.753

# ==================================
#  Spiral Path: high positive w, then small negative w
# ==================================
w_data[:] = -1/100
w_data[0:100] = 1

# ==================================
#  Wave Path: square wave w input
# ==================================
# w_data[:] = 0
# w_data[0:100] = 1
# w_data[100:300] = -1
# w_data[300:500] = 1
# w_data[500:5700] = np.tile(w_data[100:500], 13)
# w_data[5700:] = -1

# ==================================
#  Step through bicycle model
# ==================================
for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc
    model.step(v_data[i], w_data[i])
    
plt.axis('equal')
plt.plot(x_data, y_data,label='Learner Model')
plt.legend()
plt.show()
