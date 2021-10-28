# Ubuntu 18

FROM ubuntu:18.04
#FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04

ARG USER=ubuntu18
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

# 
RUN apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y \
      python \
    && rm -rf /var/lib/apt/lists/*

#
USER ${USER}
##
#RUN  mkdir -p ${HOME}/.config/nvim
#COPY init.vim ${HOME}/.config/nvim
#
ENV HOME ${HOME}
WORKDIR ${HOME}
CMD /bin/bash
