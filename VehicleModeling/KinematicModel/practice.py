import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from Kinematic_BicycleModel import Bicycle

sample_time = 0.01
time_end = 30
model = Bicycle()

t_data = np.arange(0, time_end, sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)
w_data = np.zeros_like(t_data)

# ==================================
#  Learner solution begins here
# ==================================
# set delta directly
v_data[:] = 32 * np.pi / 30

w_data[:315] = 1
w_data[315:] = -0.01

for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc
    
    if i < 352 or i > 1800:
        if model.delta < np.arctan(2/8):
            model.step(v_data[i], model.w_max)
            w_data[i] = model.w_max
        else:
            model.step(v_data[i], 0)
            w_data[i] = 0
    else:
        if model.delta > -np.arctan(2/8):
            model.step(v_data[i],-model.w_max)
            w_data[i] = -model.w_max
        else:
            model.step(v_data[i],0)
            w_data[i] = 0
    
    model.beta = 0
    
# ==================================
#  Learner solution ends here
# ==================================
plt.axis('equal')
plt.plot(x_data, y_data)
plt.show()
