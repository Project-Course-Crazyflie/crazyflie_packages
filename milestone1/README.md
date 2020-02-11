1. roslaunch dd2419_launch base.launch
2. roslaunch milestone1 milestone1.launch
3. Run
    - rosrun milestone1 key_cmd_vel (control drone with arrow keys)
    - rosrun milestone1 inp_cmd_pos (command position with input)
    - rosrun milestone1 

Record in a rosbag: rosbag record <[topics]> -O crazybag.bag
Play rosbag:        rosbag play crazybag.bag 
