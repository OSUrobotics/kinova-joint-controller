# kinova-joint-controller
capture/execute joint poses on kinova arm either virtually or in real world.

## How to build environment: 
https://docs.google.com/document/d/1U_Y6YVRuo5g96acER3KHRvnXLiMlDnD1l2XDX_rjrJY/edit
- Instead of using kinova_path_planning.py, use general_path_planner_kinova.py
- Note: if *sudo apt-get install moveit** doesn't work, try *sudo apt install ros-$ROS_DISTRO-moveit*

## Change Robot Speed: (virtual or real world)
- go to *kinova-ros/kinova_moveit/robot_configs/j2s7s300_moveit_config/config*
- modify the joint velocity values in *joint_limits.yaml*
