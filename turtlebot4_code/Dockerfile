###
# to use current window, run:
# sudo docker run -it --net=host --ipc=host --privileged --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="${XAUTHORITY}:/root/.Xauthority" turtlebot4_nav

FROM osrf/ros:humble-desktop

ENV ROS_VERSION=2
ENV GZ_VERSION fortress
ENV ROS_DISTRO humble
RUN echo "export GZ_VERSION=${GZ_VERSION}" >> ~/.bashrc

# install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# update packages
RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && sudo apt-get install -y wget

# install gazebo
RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -q -y --no-install-recommends \
    ignition-fortress \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    python3-colcon-ros \
    python3-colcon-common-extensions \
    python3-rosdep \
    && apt-get clean

# install GZ-ROS2-Control from source
RUN mkdir -p /home/ros2_ws/src \
    && cd /home/ros2_ws/src \
    && git clone https://github.com/ros-controls/gz_ros2_control/ --branch humble \
    && rosdep update \
    && rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}

RUN cd /home/ros2_ws/ \
  && . /opt/ros/"${ROS_DISTRO}"/setup.sh \
  && colcon build --merge-install

RUN apt-get update
RUN apt-get install -y ros-dev-tools
RUN apt-get install -y ros-humble-ros-gz
RUN apt-get install -y ros-humble-ros-ign-bridge
RUN export GZ_VERSION=${GZ_VERSION}

# install Turtlebot4 packages
RUN apt-get update && apt-get install -qq -y --no-install-recommends \
    ros-humble-turtlebot4-bringup \
    ros-humble-turtlebot4-description \
    ros-humble-turtlebot4-msgs \
    ros-humble-turtlebot4-navigation \
    ros-humble-turtlebot4-node \
    ros-humble-turtlebot4-simulator \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update
RUN apt-get install -y python-is-python3
RUN apt-get install -y tmux
RUN apt-get install -y ros-humble-plotjuggler-ros

# install apt deps
RUN apt-get update && apt-get install -y vim
RUN apt-get install -y libeigen3-dev
RUN apt install -y libx11-6

ARG HOST_UID
RUN useradd -ms /bin/bash dev && echo "dev:dev" | chpasswd && adduser dev sudo
USER dev
WORKDIR /home/dev

# set up .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=25" >> ~/.bashrc
RUN echo "export IGN_CONFIG_PATH=/usr/share/ignition/" >> ~/.bashrc
RUN echo "export GZ_VERSION=${GZ_VERSION}" >> ~/.bashrc
RUN echo "alias run_maze_sim='ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true world:=maze' " >> ~/.bashrc
#RUN echo "alias run_ahg_sim='ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true world:=ahg2202_world x:=-5.5 y:=24 yaw:=1.5708' " >> ~/.bashrc
RUN echo "alias run_ahg_sim='ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true rviz:=true localization:=true world:=ahg2202_world x:=-5.5 y:=24 yaw:=1.5708 map:=/home/dev/turtlebot4_nav/maps/AHG.yaml' " >> ~/.bashrc
RUN #echo "alias run_cdl_sim='ros2 launch turtlebot4_nav remapped_turtlebot4_nav_bringup.launch.py world:=CDL_map x:=-0.5 y:=2 yaw:=3.1416 map:=/home/dev/turtlebot4_nav/maps/CDL_map.yaml' " >> ~/.bashrc
RUN echo "alias run_cdl_sim='ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true rviz:=true localization:=true world:=CDL_map x:=-0.5 y:=2 yaw:=3.1416 map:=/home/dev/turtlebot4_nav/maps/CDL_map.yaml' " >> ~/.bashrc

RUN git clone https://github.com/ut-amrl/amrl_msgs.git && \
    git clone https://github.com/ut-amrl/ut_turtlebots.git --recurse-submodules

# build amrl_msgs and ut_turtlebots
RUN . /opt/ros/humble/setup.sh && \
    . /home/ros2_ws/install/setup.sh && \
    cd amrl_msgs/ && colcon build --packages-select amrl_msgs && \
    . /home/dev/amrl_msgs/install/setup.sh && \
    cd ../ut_turtlebots && colcon build --packages-select ut_turtlebots

RUN echo "source /home/dev/amrl_msgs/install/setup.bash" >> ~/.bashrc
RUN echo "source /home/dev/ut_turtlebots/install/setup.bash" >> ~/.bashrc

# env vars for the nvidia-container-runtime
ENV QT_X11_NO_MITSHM 1

# add launcher
ENV TURTLEBOT4_NAV_DOCKER_CONTEXT 1
COPY --chown=dev:dev ./tmux_session.sh /home/dev/tmux_session.sh
COPY --chown=dev:dev ./turtlebot4_nav /home/dev/turtlebot4_nav
COPY --chown=dev:dev ./pj_layouts /home/dev/pj_layouts
RUN echo "export GZ_SIM_RESOURCE_PATH=/home/dev/turtlebot4_nav/worlds/" >> ~/.bashrc

# uncomment below if you want to use GPU for Gazebo laser scan
#RUN echo "export LIBGL_ALWAYS_SOFTWARE=true" >> ~/.bashrc

RUN chmod u+x /home/dev/tmux_session.sh
CMD [ "/home/dev/tmux_session.sh" ]
ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
