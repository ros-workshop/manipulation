# manipulation
Repository containing the material for day 4 of the ROS Workshop.

<details><summary>Before you start</summary>
<p>
  
#### Clone this repo in your */src* folder

```python
git clone "URL"
```

#### Set the name of your catkin workspace

```python
export CATKIN_WS=/"THE NAME OF YOUR CATKIN WORKSPACE"
```
#### Go to the manipulation folder

```python
cd ~/$CATKIN_WS
```
#### Execute dependencies.sh
```python
chmod 777 dependencies.sh
./dependencies.sh
```

</p>
</details>

<details><summary>To get going</summary>
<p>

#### Bring up the robot

```python
roslaunch husky_ur5_gazebo husky_ur5.launch

```
#### Allow for planning

```python
roslaunch husky_ur5_moveit_config moveit_planning_execution_gazebo.launch 

```
#### Set the Gazebo environment

```python
roslaunch apriltags_gazebo apriltag_spawn.launch

```

#### Find the tags

```python
roslaunch apriltags_gazebo tag_detection.launch 

```

</details>


