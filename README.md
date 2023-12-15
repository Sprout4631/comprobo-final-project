# Team Art2D2 repo
This is the repository for the 2023 CompRobo final project by Anusha, Alana, and Bill. The report website can be found [here](https://sites.google.com/view/comprobo-23-art2d2/home).

To run the drawing program, run the following commands in separate terminals:
```bash
$ ros2 launch neato_node2 bringup.py host:=$YOUR_HOST_IP_HERE
$ ros2 launch art2d2 test_slam_toolbox.py
$ ros2 run art2d2 localization_node
$ rviz2 -d rviz.rviz # Refer to the rviz.rviz conf file in this dir :w
# Wait 5 seconds for a laser scans show up in rviz,
$ ros2 run artd2 drawing_node
```