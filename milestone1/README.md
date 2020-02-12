1. roslaunch dd2419_launch base.launch
2. roslaunch milestone1 milestone1.launch
3. Run
    - rosrun milestone1 key_cmd_vel (control drone with arrow keys) and rosrun key_teleop key_teleop.py
    - rosrun milestone1 inp_cmd_pos (command position with input) and rosrun milestone1 navgoal
    - rosrun milestone1 demonstrate and rosrun milestone1 navgoal

Record in a rosbag: rosbag record cf1/pose /myresult -O crazybag.bag
Play rosbag:        rosbag play crazybag.bag 
