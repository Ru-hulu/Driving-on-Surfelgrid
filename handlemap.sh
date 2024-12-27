gnome-terminal -x bash -c "cd /home/r/Mysoftware/ExplorationUneven; source devel/setup.bash; roslaunch scout_gazebo_sim scout_empty_world.launch"
sleep 1
gnome-terminal -x bash -c "cd /home/r/Mysoftware/ExplorationUneven; source devel/setup.bash; roslaunch planeoct_server t.launch"
sleep 2
gnome-terminal -x bash -c "rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
sleep 1