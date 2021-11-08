# ROS Melodic & Ubuntu 2018

#FROM ubuntu:18.04
FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04

ARG USER=ubuntu18
ARG HOME=/home/${USER}
# TIP (should be satisfied to avoid filesystem permision problems!): UID=$(id --user)
ARG UID=1000

# create USER and enable sudo
RUN apt update \
    && apt install -y sudo \
    && rm -rf /var/lib/apt/lists/*
RUN useradd --uid ${UID} --create-home --shell /bin/bash ${USER} \
    && echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USER} \
    && chmod 0440 /etc/sudoers.d/${USER} \
    && adduser ${USER} sudo

# install support for additional sources list
RUN apt update && apt install -y gnupg2 lsb-release && rm -rf /var/lib/apt/lists/*
#RUN apt update && apt install -y gnupg2 software-properties-common && rm -rf /var/lib/apt/lists/*

# install ROS
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y ros-melodic-desktop-full \
    && rm -rf /var/lib/apt/lists/*
RUN echo "source /opt/ros/melodic/setup.bash" >> ${HOME}/.bashrc

# navigation dependencies
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
        ros-melodic-map-server \
        ros-melodic-amcl \
        ros-melodic-gmapping \
    && rm -rf /var/lib/apt/lists/*

#
USER ${USER}
##
#RUN  mkdir -p ${HOME}/.config/nvim
#COPY init.vim ${HOME}/.config/nvim
#
ENV HOME ${HOME}
WORKDIR /WS
CMD /bin/bash
