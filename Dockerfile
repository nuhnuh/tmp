# Ubuntu 20

#FROM ubuntu:20.04
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04

ARG USER=ubuntu20
ARG HOME=/home/${USER}
# TIP: UID=$(id --user)
ARG UID=1000

# create USER and enable sudo
RUN apt update \
    && apt install -y sudo \
    && rm -rf /var/lib/apt/lists/*
RUN useradd --uid ${UID} --create-home --shell /bin/bash ${USER} \
    && echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USER} \
    && chmod 0440 /etc/sudoers.d/${USER} \
    && adduser ${USER} sudo

# Development dependencies
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
      build-essential cmake ccache \
    && rm -rf /var/lib/apt/lists/*

# GTSAM dependencies
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
      libboost-all-dev \
      libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

# GTSAM python dependencies
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
      python3 \
      python3-venv \
    && rm -rf /var/lib/apt/lists/*
# in ubuntu20: python-is-python3 \

#USER ${USER}
#RUN python3 -m pip install numpy
#RUN python3 -m pip install pyparsing
#RUN python3 -m pip install jupyter
###RUN python3 -m pip install Pillow
##RUN python3 -m pip install matplotlib
##RUN python3 -m pip install scipy
#USER root

# apriltag3 dependencies
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
      libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

## matplotlib under docker
#RUN apt update \
#    && DEBIAN_FRONTEND=noninteractive apt install -y \
#      python3-tk \
#    && rm -rf /var/lib/apt/lists/*

# opengv dependencies
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
      python3 \
      python3-venv \
      libeigen3-dev \
      libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

#USER ${USER}
#RUN python3 -m pip install opencv-python
#USER root

#
USER ${USER}
##
#RUN  mkdir -p ${HOME}/.config/nvim
#COPY init.vim ${HOME}/.config/nvim
#
ENV HOME ${HOME}
WORKDIR ${HOME}
CMD /bin/bash
