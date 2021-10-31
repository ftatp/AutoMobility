import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Bicycle():
    def __init__(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0
        
        self.L = 2
        self.lr = 1.2
        self.w_max = 1.22
        
        self.sample_time = 0.01
        
    def reset(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0
    
    def step(self, v, w):
        # ==================================
        #  Implement kinematic model here
        # ==================================
        self.delta += w * self.sample_time
        self.beta = np.arctan(self.lr * np.tan(self.delta) / self.L)
        self.theta += self.sample_time * v * np.cos(self.beta) * np.tan(self.delta) / self.L
        #print(self.theta)
        
        self.xc += v * np.cos(self.theta + self.beta) * self.sample_time
        self.yc += v * np.sin(self.theta + self.beta) * self.sample_time


