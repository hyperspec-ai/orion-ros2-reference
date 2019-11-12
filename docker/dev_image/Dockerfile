# This is an adapted file by DAS to geneate a
ARG FROM_IMAGE=ubuntu:bionic
FROM $FROM_IMAGE

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
  ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
  apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
  bash-completion \
  dirmngr \
  gnupg2 \
  lsb-release \
  python3-pip \
  && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
  git \
  python3-rosdep \
  python3-vcstool \
  && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip3 install -U \
  argcomplete \
  colcon-common-extensions \
  flake8 \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures

# FIXME This is a workaround for pytest not found causing builds to fail
# Following RUN statements tests for regression of https://github.com/ros2/ros2/issues/722
RUN pip3 freeze | grep pytest \
  && python3 -m pytest --version

# bootstrap rosdep
RUN rosdep init \
  && rosdep update

# clone source
ENV ROS2_WS /opt/ros2_ws
ENV KROS_WS /opt/ws_kros
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

# build source
RUN colcon \
  build \
  --cmake-args \
  -DSECURITY=ON --no-warn-unused-cli \
  --symlink-install

# setup bashrc
RUN cp /etc/skel/.bashrc ~/

COPY ./spinnaker-1.24.0.60-amd64-Ubuntu18.04-pkg.tar.gz /spinnaker-1.24.0.60-amd64-Ubuntu18.04-pkg.tar.gz
# install packages
RUN apt-get update && apt-get install -q -y \
  libasio-dev \
  libtinyxml2-dev \
  wget \
  libgstreamer1.0-0 \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-doc \
  gstreamer1.0-tools \
  gstreamer1.0-x \
  gstreamer1.0-alsa \
  gstreamer1.0-gl \
  gstreamer1.0-gtk3 \
  gstreamer1.0-qt5 \
  gstreamer1.0-pulseaudio \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  rtklib \
  libcurl4-openssl-dev \
  libarchive-dev \
  libarchive13 \
  gdbserver \
  && rm -rf /var/lib/apt/lists/*
#unpack and install spinnaker api
RUN tar -zxvf /spinnaker-1.24.0.60-amd64-Ubuntu18.04-pkg.tar.gz -C /
WORKDIR /spinnaker-1.24.0.60-amd64
RUN dpkg -i 'libspinnaker-1.24.0.60_amd64.deb' && dpkg -i 'libspinnaker-1.24.0.60_amd64-dev.deb' && dpkg -i 'libspinnaker-c-1.24.0.60_amd64.deb' \
  && dpkg -i 'libspinnaker-c-1.24.0.60_amd64-dev.deb' \
  && dpkg -i 'libspinvideo-1.24.0.60_amd64.deb' \
  && dpkg -i 'libspinvideo-1.24.0.60_amd64-dev.deb' \
  && dpkg -i 'libspinvideo-c-1.24.0.60_amd64.deb' \
  && dpkg -i 'libspinvideo-c-1.24.0.60_amd64-dev.deb' \
  && dpkg -i 'spinnaker-1.24.0.60_amd64.deb' \
  && dpkg -i 'spinnaker-doc-1.24.0.60_amd64.deb' \
  && dpkg -i 'spinupdate-1.24.0.60_amd64.deb' \
  && dpkg -i 'spinupdate-1.24.0.60_amd64-dev.deb'
# && dpkg -i 'libspinvideo-*.deb' && dpkg -i 'spinupdate-*.deb' && dpkg -i 'spinnaker-*.deb'
ARG ROS_DISTRO=dashing
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_VERSION=2 \
  ROS_PYTHON_VERSION=3

WORKDIR $ROS2_WS

RUN wget https://raw.githubusercontent.com/ros2/ros2/$ROS_DISTRO-release/ros2.repos \
  && vcs import src < ros2.repos

# install dependencies
RUN apt-get update && rosdep install -y \
  --from-paths src \
  --ignore-src \
  --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"


# build source
RUN colcon build --cmake-args -DSECURITY=ON --no-warn-unused-cli --symlink-install
#install paho lib
RUN mkdir /paho \
  && cd /paho \
  && git clone https://github.com/eclipse/paho.mqtt.c.git \
  && cd paho.mqtt.c \
  && cmake -Bbuild -H. -DPAHO_WITH_SSL=ON \
  && cmake --build build/ --target install \
  && ldconfig \
  && cd /paho \
  && git clone https://github.com/eclipse/paho.mqtt.cpp \
  && cd paho.mqtt.cpp \
  && cmake -Bbuild -H. -DPAHO_BUILD_DOCUMENTATION=FALSE -DPAHO_BUILD_SAMPLES=TRUE -DCMAKE_PREFIX_PATH=/paho/paho.mqtt.c/build/install \
  && cmake --build build/ --target install && ldconfig

RUN apt-get install -q -y g++-aarch64-linux-gnu \
  g++-arm-linux-gnueabihf \
  pkg-config-aarch64-linux-gnu \
  && rm -rf /var/lib/apt/lists/*


# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
