# Manipulators-ROS2-From-Forward-Kinematics-to-Advanced-Algorithms
The Github series provides a guide to working with any manipulators designed in CAD or manually created URDF and in ROS2 simulation, implementing from Forward Kinematics (FK) to complex motion planning and control algorithms. The equation and some concepts were implemented from the <a href="https://hades.mech.northwestern.edu/index.php/Modern_Robotics"> Modern Robotics </a>

### will update soon the instrution for installation and uses.

### Milestones

- [x] [URDF and Rviz](#urdf-and-rviz)  
- [x] [Gazebo Ignition and ROS controls Integration](#gazebo)
- [x] [Forward Kinematics](#FK)
- [x] [Velocity Kinematics (Jacobian, Singularity, manipulability and Static force)](#vk)
- [ ] [Inverse Kinematics](#IK)
- [ ] Teleoperation with Multi-Threading
- [ ] Dynamics Equations
- [ ] Trajectory generation
- [ ] Impedance Control
- [ ] Admittance Control
- [ ] Motion Planning

<a name="urdf-and-rviz"></a>  
### URDF and Rviz

<div style="display: flex; justify-content: space-between;">
  <img src="assets/image.png" alt="Description of first image" width="29%">
  
</div>

``````
-----------------------------------------------------------------------------------------------------------
``````

<a name="gazebo"></a>  
### Gazebo Ignition and ROS controls Integration

<div style="display: flex; justify-content: space-between;">
  
  <img src="assets/gazebo.png" alt="Description of second image" width="29%">
</div>

<a name="fk"></a>  
### Forward Kinematics

``````
7_r_edu_control\7_r_edu_control\fk.py
``````

<img src="assets/fk1.jpg" alt="Description of second image" width="69%">
<img src="assets/fk2.jpg" alt="Description of second image" width="69%">
<img src="assets/fk3.jpg" alt="Description of second image" width="69%">

<a name="vk"></a>  
### Velocity Kinematics (Jacobian, Singularity, manipulability and Static force)

``````
7_r_edu_control\7_r_edu_control\vk.py
``````

<img src="assets/vk1.jpg" alt="Description of second image" width="69%">
<img src="assets/vk2.jpg" alt="Description of second image" width="69%">
<img src="assets/vk3.jpg" alt="Description of second image" width="69%">


<a name="ik"></a>  
### Inverse Kinematics

``````
yet to add
``````

<img src="assets/ik1.jpg" alt="Description of second image" width="69%">
<img src="assets/ik2.jpg" alt="Description of second image" width="69%">
<img src="assets/ik3.jpg" alt="Description of second image" width="69%">