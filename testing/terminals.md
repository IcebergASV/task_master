
# ssh
`becky`

# Terminals to open on startup

## 1
`rostopic echo /prop_array `

## 2
`rostopic echo /yolov5/BoundingBoxes`

## 3
`rostopic echo /mavros/setpoint_position/local`

## 4   
`rostopic echo /mavros/local_position/pose`

## 5  
`rostopic echo /mavros/mission/reached`

## 6
`cd roboboat_2024_ws/src/`



# Aliases

## Build

`build`

### Action
`cd ../roboboat_2024_ws/`

`catkin build`

## Launching task master

`launch`

### Action
`cd ../roboboat_2024_ws/src/task_master/launch/`

`roslaunch master_launch.launch`

## Edit nav_channel_ctrl

`encc`
### Action
`cd ../roboboat_2024_ws/src/nav_channel/src/`

`vim nav_channel_ctrl.cpp`

## Edit task_master

`etmc`
### Action
`cd ../roboboat_2024_ws/src/task_master/src/`

`vim task_ctrl.cpp`

## Edit prop mapper params
`ecpm`

### Action
`cd ../roboboat_2024_ws/src/prop_mapper/config/`

`vim params.yml`

## Edit nav channel params

`ecnc`
### Action
`cd ../roboboat_2024_ws/src/nav_channel/config/`

`vim params.yml`

## Edit task master params
`ectm`

### Action
`cd ../roboboat_2024_ws/src/task_master/config/`

`vim params.yml`

## Edit speed challenge params

`ecsc`

### Action

`cd ../roboboat_2024_ws/src/speed_challenge/config/`

`vim params.yml`

# Rostopic echoing

## Prop array
`parr`

`rostopic echo /prop_array`
## Polar coords
`polc`

`rostopic echo /prop_polar_coords`

## completed props
`comp`

`rostopic echo /completed_props`

## local pose

`locpose`

`rostopic echo /mavros/local_position/pose`

## global pose

`glopose` 

`rostopic echo /mavros/global_position/global`

## Local Setpoint

`locsetp`

`rostopic echo /mavros/setpoint_position/local`

## Global Setpoint

`glosetp`

`rostopic echo /mavros/setpoint_position/global`
