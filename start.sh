gnome-terminal -x bash -c "cd /home/r/Mysoftware/ExplorationUneven; source devel/setup.bash; roslaunch planeoct_server t.launch"
sleep 2
gnome-terminal -x bash -c "cd /home/r/Mysoftware/ExplorationUneven; source devel/setup.bash; roslaunch hybrida t.launch"
sleep 2
# gnome-terminal -x bash -c "rosservice call /static_map_server "data: false""
# sleep 2
# gnome-terminal -x bash -c "rosservice call /uneven_nav "data: false""
# sleep 2
gnome-terminal -x bash -c "rviz"
sleep 2