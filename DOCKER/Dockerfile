FROM osrf/ros:humble-desktop-full
MAINTAINER Rafal Staszak <staszak.raf@gmail.com>
RUN echo "Europe/Utc" > /etc/timezone
# RUN ln -fs /usr/share/zoneinfo/Europe/Rome /etc/localtime
RUN apt-get update -q && apt-get install -y apt-transport-https && \
	export DEBIAN_FRONTEND=noninteractive
RUN apt-get update -q && \
	export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends tzdata
RUN dpkg-reconfigure -f noninteractive tzdata
# Install packages
ENV DEBIAN_FRONTEND=noninteractive

# Update ROS key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# Now update package lists
RUN apt-get update && \
	export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends apt-utils software-properties-common wget curl rsync netcat-traditional mg vim bzip2 zip unzip && \
    apt-get install -y --no-install-recommends libxtst6 && \
    apt-get install -y --no-install-recommends bash-completion && \
    apt-get install -y --no-install-recommends nano && \ 
    apt-get install -y --no-install-recommends net-tools && \
    apt-get install -y --no-install-recommends iputils-ping && \
    apt-get install -y --no-install-recommends terminator && \
    apt-get install -y --no-install-recommends ros-humble-moveit && \
    apt-get install -y --no-install-recommends ros-humble-ur-description && \
    apt-get install -y --no-install-recommends ros-humble-controller-manager && \
    apt-get install -y --no-install-recommends ros-humble-gazebo-ros-pkgs && \
    apt-get install -y --no-install-recommends ros-humble-ros2-control && \
    apt-get install -y --no-install-recommends ros-humble-ros2-controllers && \
    apt-get install -y --no-install-recommends ros-humble-joint-state-broadcaster && \
    apt-get install -y --no-install-recommends ros-humble-joint-trajectory-controller && \
    apt-get install -y --no-install-recommends ros-humble-gazebo-ros2-control && \
    apt-get install -y --no-install-recommends ros-humble-ur && \
    apt-get install -y --no-install-recommends ros-humble-moveit-ros-perception && \
    apt-get install -y --no-install-recommends ros-humble-moveit-ros-occupancy-map-monitor && \
    apt-get install -y --no-install-recommends ros-humble-moveit-ros-visualization && \
    apt-get install -y --no-install-recommends ros-humble-filters && \
    apt-get autoclean -y && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
    ENV IGN_GAZEBO_RESOURCE_PATH=/opt/ros/humble/share
RUN apt-get update -q && \
        export DEBIAN_FRONTEND=noninteractive && \
    rm -rf /var/lib/apt/lists/*
RUN sed -i 's/--no-generate//g' /usr/share/bash-completion/completions/apt-get && \
    sed -i 's/--no-generate//g' /usr/share/bash-completion/completions/apt-cache
WORKDIR /root/

RUN sed -i "s/#force_color_prompt=yes/force_color_prompt=yes/g" /root/.bashrc

RUN echo 'if [ -f /etc/bash_completion ] && ! shopt -oq posix; then \n\
    . /etc/bash_completion \n\
fi \n\
\n\
export USER=root \n\
source /opt/ros/$ROS_DISTRO/setup.bash' >> /root/.bashrc

RUN touch /root/.Xauthority
