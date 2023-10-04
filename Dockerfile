# Use a minimal Ubuntu base image
FROM ubuntu:20.04

# TODO use alpine linux (more secure and leaner): https://jfrog.com/devops-tools/article/why-use-ubuntu-as-a-docker-base-image-when-alpine-exists/
# But there are some challenges: https://unix.stackexchange.com/questions/593549/can-you-install-ubuntu-or-debian-packages-in-alpine

# Error: https://uk.mathworks.com/help/matlab/matlab_env/remove-canberra-gtk-module-and-pk-gtk-module-messages.html#
# Solution: https://askubuntu.com/questions/1175572/how-to-fix-error-failed-to-load-module-canberra-gtk-module
#           https://uk.mathworks.com/help/matlab/matlab_env/remove-canberra-gtk-module-and-pk-gtk-module-messages.html# 

# Set environment variables to avoid interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
add-apt-repository -y "deb http://security.ubuntu.com/ubuntu xenial-security main"; \
fi

# Remove optional packages in production: sudo, usbutils, libcanberra-gtk3-module, iputils-ping

# Install dependencies
RUN apt update && \
    apt install -y --no-install-recommends software-properties-common && \
    add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    add-apt-repository -y ppa:deadsnakes/ppa  && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo \
    usbutils \
    libcanberra-gtk3-module \
    udev \
    git \
    gcc-9 \
    g++ \
    libeigen3-dev \
    build-essential \
    cmake \
    unzip \
    pkg-config \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libatlas-base-dev \
    gfortran \
    python3.6-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libglew-dev \
    libyaml-cpp-dev \
    geographiclib-tools \
    libgeographic-dev \
    libspdlog-dev \
    libgtest-dev && \
    rm -rf /var/lib/apt/lists/*

RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
apt install --fix-missing -y libjasper-dev; \
fi

# Clone repositories
WORKDIR /ws

RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout 3.4 && \
    cd ..

RUN git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && \
    git checkout 3.4 && \
    cd ..

RUN git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    git reset --hard 61cf21520b4bb29c2c1074f7bad74a6cfcd93ca3 && \
    cd ..

RUN git config --global advice.detachedHead false && \
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git -b v0.8

# Build OpenCV
WORKDIR /ws/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$(python3 -c "import sys; print(sys.prefix)") -D PYTHON3_EXECUTABLE=$(which python3) -D PYTHON_DEFAULT_EXECUTABLE=$(which python3) -D WITH_GTK=ON -D WITH_FFMPEG=ON -D BUILD_EXAMPLES=ON -D INSTALL_C_EXAMPLES=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -D INSTALL_PYTHON_EXAMPLES=ON -D HAVE_opencv_python3=ON .. && \
    make -j4 && \
    make install

# Build librealsense 
WORKDIR /ws/librealsense/
RUN chmod +x ./scripts/setup_udev_rules.sh && \
    ./scripts/setup_udev_rules.sh

WORKDIR /ws/librealsense/build
RUN cmake ../ -DBUILD_EXAMPLES=true -DFORCE_RSUSB_BACKEND=true && \
    make -j6 && \
    make install

# Build Pangolin
WORKDIR /ws/Pangolin/build
RUN cmake .. && \
    make -j6

WORKDIR /ws

ENTRYPOINT ["tail", "-f", "/dev/null"]
