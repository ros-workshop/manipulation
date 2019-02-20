# Manipulation

**Goal:**

The session aims to familiaries you with one of the most crucial aspects of modern robotics, *manipulation*.

**What do we mean by manipulation?**

Whenever a robots physically interacts with its surrounding and modifies its environement, we talk about manipulation.

**What are the challenges of robotic manipulation?**
+ *Path Planning:* Generally, a robotic arm is used for manipulation ([ABB IRB 120](https://new.abb.com/products/robotics/industrial-robots/irb-120),[UR5](https://www.scottautomation.com/products/ur5-universal-robot/)). An arm often has between 5 and 7 Dof. Powerful planning algorithms must be used ([RTTs](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree) for example) to find a path in joint or end-effector space.
+ *Grasping:* Once the arm knows how to travel from one place to another, its time to actually grasping something. Grasping in a vast and vibrant research topic mostly because how challenigng it be be for robots to find suitable grasp poses fro everyday objects.


**What tools do we have to solve this problem?**

+ *ROS:* ROS allows us to integrate all aspects needed for manipulation.
+ *[Moveit](https://moveit.ros.org/):* Moveit is a ROS package for motion planning build on top of libraries such as [OMPL](http://ompl.kavrakilab.org/).

**What else do we need to know?**
+ Manipulation works only if the robot knows what it looks like so we will be using URDFs
+ The [MoveGroup](http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html) class will be useful.

## Let's begin

### Workspace
By now you should have already set up a catkin workspace. Clone this repo in `workshop_ws/src`.
<details><summary>Click for Hint</summary>
<p>

```python
cd ~/workshop_ws/src
git clone https://github.com/ros-workshop/manipulation.git
```
</p>
</details>

### Dependencies

There are quite a few packages that need to be installed for the session:

+ moveit_ros
+ moveit_core
+ apriltag2_ros
+ image_geometry
+ husky_description
+ controller_manager
+ gazebo_ros_control

<details><summary>Click for Hint</summary>
<p>
  
you can `sudo apt-get install` almost all of these packages but you will have to clone [apriltag2_ros](https://github.com/dmalyuta/apriltags2_ros)

</p>
</details>

## Everything a Robot needs

**ACTION**
Launch the robot in gazebo. 

<details><summary>Click for Hint</summary>
<p>
  
```
workshop_ws
│   devel
│   build   
└───src
│   │   pkg1
│   │   pkg2
│   └───robot_pkg
│       │   robot_description
|       └───robot_gazebo
|           |   robot_spawn.launch
|           |   robot.launch
```

</p> 

</details>

You should see something like this:

**INCLUDE PICTURE HERE**

The arm and fingers should stand up straight and be static.

### Controllers

Each joint or motor in the arm and hand needs a controller for actuation. Ask yourself this: What controllers do I expect for a mobile robotic arm with an end-effector? Make sure the controllers have loaded appropriatly. 

**ACTION**
Check which controllers are loaded.

<details><summary>Click for Hint</summary>
<p>
There should be a least 3 controllers.
  
+ Mobile Base Controller
+ Arm Controller
+ Hand Controller

There is a useful rqt plugin to check your controllers 

**INCLUDE PICTURE HERE**

</p> 

</details>

Now to see the controllers in action, jog the arm and fingers manually.

**ACTION**
Move the arm and fingers.

<details><summary>Click for Hint</summary>
<p>

There is an rqt plugin for that!

**INCLUDE PICTURE HERE**

</p> 

</details>

### Joint States

Quickly check that you are getting feedback from your robots `joint_states`.

**ACTION**
Check that you are getting joint feedback.



