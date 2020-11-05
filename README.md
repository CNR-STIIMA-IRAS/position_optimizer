# CGASA - Cascaded Genetic Algorithm Simulated Annealing
This directory provides the packages and example to use the CGASA algorithm to optimize the manipulabilty indexes of the initial and final poses for a robot manipulation task.

---

## 1.Installation
git clone
will install three main packages
---
##  2.Packages organization 

The three packages are:
- ##### manipulability_pkg 
The manipulability_pkg package is in charge to compute the manipulability, ask for trajectory planning and manages the
 - ##### position_optimizer 
The position_optimizer package runs the CGASA algorithm
 - ##### position_optimizer_examples
The position_optimizer_examples package contains examples

---
## 3.Configurations
Some configuration files must be specified to run the algorithm.

The GA_params.yaml and SA_params.yaml, in the position_optimizer_examples, are used to specify the parameters typical of the two algorithms, and are structured as follows:
- ##### GA_params.yaml
```yaml
individuals_number    : 50      # number of individuals
max_generations_number: 100     # maximum iteration number of the GA
selection_method      : "rank"  # selection method of the GA; already implemented are roulette_wheel, roulette_wheel_probability, tournament
mutation_rate         : 0.05    # mutation rate of the individuals each generation
last_elements         : 3       # stop criterion is when the best elements of the last n generations -specified here- are equals
probability_rank      : 0.05    # parameter to tune the probability assigned to each individual according to (1-pc)^(n-i)*pc
weight                : [1, 1]  # a weight used to scale the manipulability of the pick or the place poses
final_annealing       : True    # if False when the GA terminates no further SA optimization will be performed
save_data_to_txt      : False   # if True will save data in txt files relative to the history of the populations for analysis
```
- ##### SA_params.yaml
```yaml
temperature_decrease_step:  30              # number of SA iterations
trial_each_step: 20                         # number of new solution evaluated each iteration
initial_acceptance_probability: 0.7         # probability to accept a worst solution candidate at the first iteration
final_acceptance_probability: 0.01          # probability to accept a worst solution candidate at the last iteration
delta_pick:  [ 0.03,0.03,0.03,0,0,0.03 ]    # (x,y,z,R,P,Y), maximum search space around the candidate solution for the pick pose [m]
delta_place: [ 0.03,0.03,0.03,0,0,0.03 ]    # (x,y,z,R,P,Y), maximum search space around the candidate solution for the place pose [m]
```

The objects to be added to the scene are defined in the scene_objects.yaml, and can be both standard geometric shapes and custom meshes 
- ##### scene_objects.yaml
```yaml
object_name     : "box"     # name of the object to be manipulated (must coincide with one of the objects defined below)
environment_name: "table"   # name of the object representing teh environment where the object have to be placed (must coincide with one of the objects defined below)
# list of the possible objects
box           : { box : [ 0.01, 0.4, 0.1 ]                          , color: [ 255,255,   0, 1 ] }
table         : { box : [ 0.5, 0.7, 0.2 ]                           , color: [   0,  0, 255, 1 ] }
custom_object : { mesh: "package://<mesh_pkg>/meshes/mesh_name.stl" , color: [   0,255,   0, 1 ] }
``` 
To define the GA search space, two different methods can be used, the first is the Cartesian representation in the form {x, y, z, R, P, Y}, while the second method allows to use spherical parameters in the form {&rho;, &theta;, &phi;, R, P, Y}.
Such parameters along with the other required to create the chromosomes are defined in the *_chromosome_params.yaml

- ##### chromosome_params.yaml
``` 
chromosome_type: "cartesian"                  # type pf the chromosome - cartesian and spherical implemented

pick_pose:
  id        : "pick"                          # name of the pick pose
  base      : [0.7 , 0, 0.6, 0, 0, 0]         # base pose
  l_bound   : [-0.3,-0.3, -0.1, 0, 0, -0.3]   # search space lower bound
  u_bound   : [0.3 , 0.3, 0.1, 0, 0,  0.3]    # search space upwer bound
  linspace  : [120,120,1,1,1,50]              # search sapce discretization
  offset    : [0, 0, 0, 0, 0, 0]              # offset wrt mesh origin
  ee_offset : [-0.02, 0, 0, 0, 0, 0]          # goal pose of the end effector wrt component pose

place_pose:
  id        : "place"
  base      : [-0.7, 0.0, 0.25, 0, 1.5707, 3.1415]
  l_bound   : [-0.4, -0.4, -0.1, 0, 0, -0.1]
  u_bound   : [0.4,   0.4,  0.1, 0, 0, 0.1]
  linspace  : [120,120,1,1,1,50]
  offset    : [0, 0, 0, 0, 0, 0]
  ee_offset : [-0.29,0,0,0,0,0]

``` 
Finally some robot dependent parameters are defined in the joint_config.yaml file

- ##### joint_config.yaml

``` 
joint_home  : [       0, -1.5707,       0,       0,      0,       0 ] # joint values of the home position
lower_bounds: [ -3.1416, -3.1416, -3.1416, -3.1416,-3.1416, -3.1416 ] # lower joints limits
upper_bounds: [  3.1416,  3.1416,  3.1416,  3.1416, 3.1416,  3.1416 ] # upper joints limits

pick:
  seed_target_joints: [-2.9, -1.7, -1.7, 0.3, 1.3, 1.6]               # joint values used as seed to find the joint configuration relative to the pick pose
place:
  seed_target_joints: [0.25, -1.7, -1.25, -1.5, 1.5, -2.7]            # joint values used as seed to find the joint configuration relative to the place pose
``` 

---
## 4. Running the CGASA
Once the required parameters are set, launching the following manipulability_optimizer.launch file, with small modifications according to the robot used, will begin the optimization.

``` 
<?xml version="1.0"?>
<launch>
  
  <!-- parameters specific of the robot -->
  <arg name="robot"     default="ur"/>
  <arg name="use_case"  default="cartesian"/> <!--cartesian,spherical-->

  <param name="planning_group" type="string" value="manipulator" />
  <param name="base_link"      type="string" value="base_link" />
  <param name="tip_link"       type="string" value="ee_link" />

  <!--  loading .yaml files-->
  <rosparam command="load" file="$(find position_optimizer_examples)/config/GA_params.yaml"/>
  <rosparam command="load" file="$(find position_optimizer_examples)/config/SA_params.yaml"/>

  <rosparam command="load" file="$(find position_optimizer_examples)/config/scene_objects.yaml"/>
  <rosparam command="load" file="$(find position_optimizer_examples)/config/$(arg robot)/$(arg use_case)_chromosome_params.yaml"/>
  <rosparam command="load" file="$(find position_optimizer_examples)/config/$(arg robot)/joint_config.yaml"/>

  <!--  include the launcher file of the CGASA-->
  <include file="$(find position_optimizer)/launch/pose_optimizer.launch"/>

  <!--  include the moveit package of the specific robot-->
  <include file="$(find ur10_moveit_config)/launch/demo.launch">
    <arg name="limited" value="true" />
  </include>


</launch>

``` 

---

The position_optimizer_example already contains different robot examples, to easy run it just type
```
$ roslaunch position_optimizer_examples ur_manipulability_optimizer.launch 
``` 
This package will require different moveit packages for the different robot used, please refer to:

- [universal_robot](https://github.com/ros-industrial/universal_robot.git) to run the examples with the UR robots,
- [iiwa_stack](https://github.com/CNR-ITIA-IRAS/iiwa_stack) to run the examples with the KUKA iiwa robots,
- [panda](https://github.com/ros-planning/panda_moveit_config.git) to run the examples with the panda robot

Once the algorithm terminates, the best initial and final poses found, along with the corresponding joint configurations, are written in the _optimized_poses.txt_, under the position_optimizer/data/ directory