
## Installation

# create a ROS workspace 
```bash
$ -mkdir\workspace_ros/src
```
# build the ROS workspace 
```bash
$ catkin_make
```
Clone this package in the src folder
```bash
$ git clone https://github.com/SillyVon/RT-Assignment-II-_-2023.git
```
# build the ROS wokrspace again 
```bash
$ catkin_make
```
# run the package by running the launcher in the package 
```bash
$ roslaunch assignment_2_2023 assignment1.launch
```

## Running the nodes 

in order to run the nodes and activate the services, use the rosservice calls in a seperated terminal with the argument True .

# node last target service 
```bash
$ rosservice call last_target True
```
# node velocity and distance 

```bash
$ rosservice call distance_from_goal True
```



