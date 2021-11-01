import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Vehicle():
    def __init__(self):
 
        # ==================================
        #  Parameters
        # ==================================
    
        #Throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002
        
        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81
        
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01
        
        # Tire force 
        self.c = 10000
        self.F_max = 10000
        
        # State variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0
        
        self.sample_time = 0.01
        
    def reset(self):
        # reset state variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0

    def step(self, throttle, alpha):
        # ==================================
        #  Implement vehicle model here
        # ==================================
        T_e = throttle * (self.a_0 + self.a_1 * self.w_e + self.a_2 * (self.w_e**2))


        F_areo = self.c_a * (self.v**2)
        R_x = self.c_r1 * self.v
        F_g = self.m * self.g * np.sin(alpha)

        F_load = F_areo + R_x + F_g

        self.w_e_dot = (T_e - self.GR * self.r_e * F_load) / self.J_e

        W_w = self.GR * self.w_e
        S = (W_w * self.r_e - self.v) / self.v

        if S < 1:# and S > -1 :
            F_x = self.c * S
        else :
            F_x = self.F_max

        self.w_e += self.w_e_dot * self.sample_time
        self.a = (F_x - F_load) / self.m
        self.v += self.a * self.sample_time
        self.x += (self.v * self.sample_time) - (0.5 * self.a * self.sample_time**2)



sample_time = 0.01
time_end = 100
model = Vehicle()

t_data = np.arange(0, time_end, sample_time)
a_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)

# throttle percentage between 0 and 1
throttle = 0.2

# incline angle (in radians)
alpha = 0

for i in range(t_data.shape[0]):
    a_data[i] = model.a
    v_data[i] = model.v
    model.step(throttle, alpha)

plt.plot(t_data, v_data)
plt.show()
