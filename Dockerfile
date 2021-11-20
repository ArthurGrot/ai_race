ARG BASE_IMAGE=ros
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"] 
ENV SHELL /bin/bash

ENV DEBIAN_FRONTEND=noninteractive
ARG MAKEFLAGS=-j$(nproc)
ENV LANG=en_US.UTF-8 
ENV PYTHONIOENCODING=utf-8

#
# install required packages
#
RUN apt-get clean && \
    apt-get update && \
    apt-get install -y \
                locales \
                python3.8 \
                python3-pip \
                python3-yaml \
                libsdl1.2-dev \
                python-dev \
                libsdl-image1.2-dev \
                libsdl-mixer1.2-dev \
                libsdl-ttf2.0-dev \
                libsdl1.2-dev \
                libsmpeg-dev \
                python-numpy \
                subversion \
                libportmidi-dev \
                ffmpeg \
                libswscale-dev \
                libavformat-dev \
                libavcodec-dev \
                libfreetype6-dev
    
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

WORKDIR /tmp    

#
# JetBot hw controllers
#
RUN pip3 install Adafruit-MotorHAT Adafruit-SSD1306 --verbose
RUN pip3 install setuptools traitlets getkey
RUN pip3 install pygame==1.9.6

#
# environment setup
#   
ENV WORKSPACE_ROOT=/workspace
ENV JETBOT_ROOT=${WORKSPACE_ROOT}/src/ai_race
ARG ROS_ENVIRONMENT=/opt/ros/${ROS_DISTRO}/setup.bash

# setup workspace
WORKDIR ${WORKSPACE_ROOT}
RUN mkdir -p ${WORKSPACE_ROOT}/src

#COPY scripts/setup_workspace.sh ${WORKSPACE_ROOT}/setup_workspace.sh
ENV PYTHONPATH="${JETBOT_ROOT}:${PYTHONPATH}"


#
# build project
#
COPY ai_race ${JETBOT_ROOT}/ai_race
COPY launch ${JETBOT_ROOT}/launch
COPY resource ${JETBOT_ROOT}/resource

COPY package.xml ${JETBOT_ROOT}
COPY setup.py ${JETBOT_ROOT}
COPY setup.cfg ${JETBOT_ROOT}
    
RUN source ${ROS_ENVIRONMENT} && \
    cd ${WORKSPACE_ROOT} && \
    colcon build --symlink-install --event-handlers console_direct+


#
# setup entrypoint
#

RUN echo 'source ${WORKSPACE_ROOT}/install/setup.bash' >> /root/.bashrc && \
    echo 'source ${WORKSPACE_ROOT}/install/local_setup.bash' >> /root/.bashrc


CMD ["bash"]
