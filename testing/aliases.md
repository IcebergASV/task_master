
# ssh
`becky`

# Launching task master

`launch`

### Action
`cd ../roboboat_2024_ws/src/task_master/launch/`

`roslaunch master_launch.launch`

# Edit prop mapper params
`ecpm`

### Action
`cd ../roboboat_2024_ws/src/prop_mapper/config/`

`vim params.yml`

# Edit nav channel params

`ecnc`
### Action
`cd ../roboboat_2024_ws/src/nav_channel/config/`

`vim params.yml`

# Edit task master params
`ectm`

### Action
`cd ../roboboat_2024_ws/src/task_master/config/`

`vim params.yml`

# Edit speed challenge params

`ecsc`

### Action

`cd ../roboboat_2024_ws/src/speed_challenge/config/`

`vim params.yml`

# Rostopic echoing

# Prop array
`parr`

`rostopic echo /prop_array`
# Polar coords
`polc`

`rostopic echo /prop_polar_coords`

# completed props
`comp`

`rostopic echo /completed_props`

# local pose

`locpose`

`rostopic echo /mavros/local_position/pose`

# global pose

`glopose` 

`rostopic echo /mavros/global_position/global`

# Local Setpoint

`locsetp`

`rostopic echo /mavros/setpoint_position/local`

# Global Setpoint

`glosetp`

`rostopic echo /mavros/setpoint_position/global`
