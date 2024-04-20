# Project for CS 393R
After cloning, run docker, e.g.,

> sudo make docker_all_v

Then, run the topmost line on the Dockerfile to pass the screen to the local machine:


> sudo docker run -it --net=host --ipc=host --privileged --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="${XAUTHORITY}:/root/.Xauthority" turtlebot4_nav

Lastly, after launching the docker container, build the package:

> colcon build && source install/setup.bash

Now, you can run the launch file:

> ros2 launch turtlebot4_nav simple_ahg2202.launch.py
