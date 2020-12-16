# SIMULATION TESTS



- [x] Constant velocity and yaw rate  

  `constant_cmd.py`

![](../img/constant_cmd.gif)





- [ ] Apply own ekf python class and try same tests

  `ekf_simulation.py`
  
  
  
  ![](../img/test3_ekf_ownclass_odom.png)
  
  
  
  > The figure above is a test running on jupyter notebook the EKF python class using simulated odometry.
  
  

![](../img/plot_ekf_test.png)



> The image above shows the output of running `ekf_simulation.py` which runs the Gazebo rastreator shown on GIF above, it outputs the **True state** and **Estimated state** based on same EKF python class used on figure above. 



![](../img/output_topic_ekf.png)



> Screen-shot of the topic `ekf` containing `true_state` vector (real trajectory), `estimated_state` vector (ekf output), and a constant `control vector`



TODO: 

- ~~Output as a matplotlib graph while running the gazebo demo~~
- This test uses odometry as measurement but not landmarks. Test landmarks