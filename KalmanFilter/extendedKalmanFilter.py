import pickle
import numpy as np
import matplotlib.pyplot as plt

with open('data/data.pickle', 'rb') as f:
    data = pickle.load(f)

t = data['t']  # timestamps [s]

x_init  = data['x_init'] # initial x position [m]
y_init  = data['y_init'] # initial y position [m]
th_init = data['th_init'] # initial theta position [rad]

# # input signal
v  = data['v']  # translational velocity input [m/s]
om = data['om']  # rotational velocity input [rad/s]

# bearing and range measurements, LIDAR constants
b = data['b']  # bearing to each landmarks center in the frame attached to the laser [rad]
r = data['r']  # range measurements [m]
l = data['l']  # x,y positions of landmarks [m]
d = data['d']  # distance between robot center and laser rangefinder [m]


############################################################################################
### Initialize parameters
############################################################################################

v_var = 0.01  # translation velocity variance
om_var = 0.01  # rotational velocity variance
# allowed to tune these values
# r_var = 0.1  # range measurements variance
r_var = 0.01
# b_var = 0.1  # bearing measurement variance
b_var = 10

Q_km = np.diag([v_var, om_var]) # input noise covariance
cov_y = np.diag([r_var, b_var])  # measurement noise covariance

x_est = np.zeros([len(v), 3])  # estimated states, x, y, and theta
P_est = np.zeros([len(v), 3, 3])  # state covariance matrices

x_est[0] = np.array([x_init, y_init, th_init]) # initial state
P_est[0] = np.diag([1, 1, 0.1]) # initial state covariance

# Wraps angle to (-pi,pi] range
def wraptopi(x):
    if x > np.pi:
        x = x - (np.floor(x / (2 * np.pi)) + 1) * 2 * np.pi
    elif x < -np.pi:
        x = x + (np.floor(x / (-2 * np.pi)) + 1) * 2 * np.pi
    return x


############################################################################################
### Correction
############################################################################################

def measurement_update(lk, rk, bk, P_check, x_check):
    # lk : Landmark k's coordinate
    # rk : Distance between Landmark k and Robot
    # bk : Direction angle to reach Landmark k from Robot
    theta = wraptopi(x_check[2])

    # 1. Compute mepyenv-virtualenvasurement Jacobian
    dis_x = lk[0] - x_check[0] - d[0] * np.cos(theta)
    dis_y = lk[1] - x_check[1] - d[0] * np.sin(theta)
    r = np.sqrt(dis_x**2 + dis_y**2)

    #print(dis_x)

    H = np.array([[-dis_x / r  , -dis_y / r      , (d[0] * dis_x * np.sin(theta) - d[0] * dis_y * np.cos(theta)) / r],
                  [-dis_y / r**2, 2*dis_x / r**2, -d[0] * (dis_x * np.cos(theta) + dis_y * np.sin(theta)) / r**2]])

    #-1 - d * (d_y*np.sin(theta_k) + d_x*np.cos(theta_k)) / r**2
    #-d[0] * (dis_x * np.cos(theta) + dis_y * np.sin(theta)) / r**2

    #print(P_check.shape)
    #print(H.shape)

    # 2. Compute Kalman Gain
    K = P_check.dot(H.T).dot(np.linalg.inv(H.dot(P_check).dot(H.T) + cov_y))

    # 3. Correct predicted state (remember to wrap the angles to [-pi,pi])
    x_check = x_check.reshape(3,1) + K.dot(np.array([[rk, bk]]).T - np.array([[r, np.arctan2(dis_y, dis_x) - theta]]).T)
    x_check = x_check.reshape(3)
    x_check[2] = wraptopi(x_check[2])

    # 4. Correct covariance
    P_check = (np.identity(3) - K.dot(H)).dot(P_check)

    return x_check, P_check

############################################################################################
### Prediction
############################################################################################

#### 5. Main Filter Loop #######################################################################
for k in range(1, len(t)):  # start at 1 because we've set the initial prediciton

    delta_t = t[k] - t[k - 1]  # time step (difference between timestamps)
    
    # 1. Update state with odometry readings (remember to wrap the angles to [-pi,pi])
    previous_x_est = np.array([[x_est[k - 1][0], x_est[k - 1][1], x_est[k - 1][2]]]).T # (3 x 1)
    TransMatrix = np.array([[np.cos(x_est[k - 1][2]), 0], [np.sin(x_est[k - 1][2]), 0], [0, 1]]) # (3 x 2)
    x_check = previous_x_est + TransMatrix.dot(np.array([[v[k], om[k]]]).T) 
    x_check = x_check.reshape([3])
    
    # 2. Motion model jacobian with respect to last state
    theta = wraptopi(x_check[2])
    F_km = np.array([[1, 0, -np.sin(theta) * v[k]],
                     [0, 1, np.cos(theta) * v[k]],
                     [0, 0, 1]], dtype='float')
    
    # 3. Motion model jacobian with respect to noise
    L_km = np.array([[np.cos(theta)*delta_t, 0], 
                    [np.sin(theta)*delta_t, 0],
                    [0,1]], dtype='float')
    
    # 4. Propagate uncertainty
    P_check = F_km.dot(P_est[k - 1]).dot(F_km.T) + L_km.dot(Q_km).dot(L_km.T)
    
    # 5. Update state estimate using available landmark measurements
    for i in range(len(r[k])):
#        print(x_check)
#        print(P_check)
#         print(l[i])
#         print(r[k,i])
#         print(b[k,i])
        x_check, P_check = measurement_update(l[i], r[k, i], b[k, i], P_check, x_check)

    # Set final state predictions for timestep
    x_est[k, 0] = x_check[0]
    x_est[k, 1] = x_check[1]
    x_est[k, 2] = x_check[2]
    P_est[k, :, :] = P_check


e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(x_est[:, 0], x_est[:, 1])
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Estimated trajectory')
plt.show()

e_fig = plt.figure()
ax = e_fig.add_subplot(111)
ax.plot(t[:], x_est[:, 2])
ax.set_xlabel('Time [s]')
ax.set_ylabel('theta [rad]')
ax.set_title('Estimated trajectory')
plt.show()
