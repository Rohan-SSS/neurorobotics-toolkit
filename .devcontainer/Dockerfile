FROM arm64v8/ros:humble
ENV DISPLAY=host.docker.internal:0
ENV QT_X11_NO_MITSHM=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    curl \
    wget \
    nano \
    usbutils \
    screen \
    x11-apps \
    gnupg2 \
    lsb-release \
    liboctomap-dev \
    octovis \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace
RUN chmod -R a+rw /workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-xacro \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-humble-ros2-controllers \
#     ros-humble-ros2-control \
#     && rm -rf /var/lib/apt/lists/*

# RUN apt-get update && apt-get install -y software-properties-common 
# RUN add-apt-repository -y universe
# RUN apt-get update


# RUN apt-get update && apt-get install -y libgoogle-glog-dev

# RUN mkdir /flann
# WORKDIR /flann
# RUN git clone https://github.com/flann-lib/flann.git .
# RUN mkdir build && cd build && \
#     cmake .. && \
#     make -j$(nproc) && \
#     make install
WORKDIR /
CMD ["/bin/bash"]
