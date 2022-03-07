# AutoMobility

The process of self driving can be divided into 3 big parts: Data Input, Processing, Motion Output. The perpose of this github repository is to implement this process. It will be composed of 4 different parts: (1) Vehicle Modeling (2) Kalman Filter Implementation (3) Camera Image Processing (4) Motion Planning.


## 1. Vehicle Modeling

To analyze the vehicle's movement, we need to model the vehicle and figure out what kind of factors effect the vehicle in which kind of result. The vehicle modeling can by divided in 2 different aspects: Kinematic Modeling and Dynamic Modeling.

### Kinematic Modeling
Kinematic Modeling is used especially at low speeds when the accelerations are not significant to capture the motion of a vehicle. In most cases, it is sufficient to look only at kinamic models of vehicles. One of the most famous model is the bicycle model, which is implemented in the code. Using the bicycle model, we can make the bicycle to move in the direction as we want by controling the steering angle speed (the vehicle speed is fixed, hence it is used when accelerations are not signifcant). The result can show like this:

![그림1](https://user-images.githubusercontent.com/22390526/157134127-92de62be-0707-4eee-8ad2-487ded588a7e.png)

### Dynamic Modeling
When we need to include knowledge of the forces and moments acting on the vehicle, we're performing Dynamic Modeling. In this model, we have to control the throttle and break of the vehicle to make the vehicle to move in the required speed on the road. You can check the implemention of the vehicle speed on the code

### Model Mixing
Using the above 2 models, we can control the vehicle to run on the required trajectory, as ypu can imagine the the car have a steering wheel, an accelerator (throttle), and break. The final implementation of the vehicle modeling is about making a 3d simualator using the CARLA Unreal game engine.

The required path is provided as the data, and we can use this in the implement of pure pursuit or stanley method. Check the result of the development in the following video :

[![Screenshot from 2022-03-07 22-52-33](https://user-images.githubusercontent.com/22390526/157047285-27cb3363-66de-4710-88de-6cb21da6ae81.png)](https://youtu.be/31MsmHTRn6E)

### Conclusion
To make the vehicle to move as we expect, we have to understand exactly about the parameters that effect the movement of the vehicles. The modeling method can help this, and must be applied in the autonous driving system.
