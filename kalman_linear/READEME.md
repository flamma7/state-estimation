Kalman Linear

Steps to Run
### 1. edit relevant values in cfg/kalman.yaml
```
roscore
rosparam load cfg/kalman.yaml
```
You must load the parameters to the rosserver everytime you update the yaml
### 2. Start the simulator and corresponding filter
- To track __position and velocity__ in a single dimension with **position** updates
```
python point_sim_vector2d.py
python filters/kalman_dynamic_vector2d_pos.py
```
- To track __position and velocity__ in a single dimension with **odometry** updates
```
python point_sim_vector2d.py
python filters/kalman_dynamic_vector2d_odom.py
```  
Note the system is not fully observable with just **velocity** alone, hence no kalman filter was created.
- To track __position__ in a single dimension with **position** updates  
```
python point_sim_scalar.py
python filters/kalman_dynamic_scalar.py
```
- To estimate __constant position__ in a single dimension with **position** updates  
Edit cfg/kalman.yaml to have 0's for all twists
```
python point_sim_scalar.py
python filters/kalman_constant_scalar.py
```
- To estimate __constant position in 2 axis__ with **position** updates  
Edit cfg/kalman.yaml to have 0's for all twists
```
python point_sim_vector2d.py
python filters/kalman_constant_vector.py
```