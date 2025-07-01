FROM ros:humble-ros-base

RUN apt-get update -o Acquire::AllowInsecureRepositories=true \
                   -o Acquire::AllowDowngradeToInsecureRepositories=true \
                   -o Acquire::AllowWeakRepositories=true

RUN apt install wget python3-yaml -y  \
    && echo "chooses:\n" > fish_install.yaml \
    && echo "- {choose: 5, desc: '一键配置:系统源(更换系统源,支持全版本Ubuntu系统)'}\n" >> fish_install.yaml \
    && echo "- {choose: 2, desc: 更换系统源并清理第三方源}\n" >> fish_install.yaml \
    && echo "- {choose: 1, desc: 添加ROS/ROS2源}\n" >> fish_install.yaml \
    && wget http://fishros.com/install  -O fishros && /bin/bash fishros \
    && rm -rf /var/lib/apt/lists/*  /tmp/* /var/tmp/*

RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y git vim curl ros-humble-ur ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager ros-humble-xacro

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh
CMD [ "/bin/zsh" ]

# create workspace
RUN mkdir -p ~/ros_ws && \
    cd ~/ros_ws && \
    git clone https://github.com/William-zwy/dual_ur_inspire.git src

WORKDIR /root/ros_ws

# install dependencies and some tools
RUN rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# build
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-instal

RUN mkdir -p ~/ur_ws/src && \
    cd ~/ur_ws/src && \
    git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git

RUN cd ~/ur_ws &&\
    rosdep install --from-paths src --ignore-src -r -y

# setup .zshrc
RUN echo 'export TERM=xterm-256color\n\
source ~/ros_ws/install/setup.zsh\n\
eval "$(register-python-argcomplete3 ros2)"\n\
eval "$(register-python-argcomplete3 colcon)"\n'\
>> /root/.zshrc

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "/root/ros_ws/install/setup.bash"' \
      /ros_entrypoint.sh

RUN rm -rf /var/lib/apt/lists/*
