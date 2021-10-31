import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from Kinematic_BicycleModel import Bicycle

sample_time = 0.01
time_end = 20
model = Bicycle()

# set delta directly
model.delta = np.arctan(2/10)

t_data = np.arange(0, time_end, sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
x_solution = np.zeros_like(t_data)
y_solution = np.zeros_like(t_data)

for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc
    model.step(np.pi, 0)
    model.beta = 0

plt.axis('equal')
plt.plot(x_data, y_data,label='Learner Model')
plt.legend()
plt.show()
