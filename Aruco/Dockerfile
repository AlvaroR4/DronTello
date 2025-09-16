# Dockerfile para flock2 (ROS2 Foxy / Ubuntu 20.04)
FROM osrf/ros:foxy-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 1) paquetes del sistema y dependencias ROS adicionales (según README)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    wget \
    curl \
    lsb-release \
    sudo \
    ros-foxy-cv-bridge \
    ros-foxy-camera-calibration-parsers \
    ros-foxy-gazebo-ros \
    && rm -rf /var/lib/apt/lists/*

# 2) dependencias Python (según README: numpy, transformations)
RUN pip3 install --no-cache-dir numpy transformations

# 3) inicializar rosdep
RUN rosdep init || true
RUN rosdep update

# 4) crear workspace y clonar repos (flock2 + dependencias)
WORKDIR /opt/flock2_ws/src
RUN git clone https://github.com/clydemcqueen/flock2.git \
 && git clone https://github.com/clydemcqueen/tello_ros.git \
 && git clone https://github.com/ptrmu/fiducial_vlam.git \
 && git clone https://github.com/ptrmu/ros2_shared.git

# 5) instalar dependencias con rosdep y compilar (si no quieres Gazebo, saltamos tello_gazebo)
WORKDIR /opt/flock2_ws
RUN /bin/bash -lc "source /opt/ros/foxy/setup.bash && rosdep install --from-paths src --ignore-src -r -y || true"
RUN /bin/bash -lc "source /opt/ros/foxy/setup.bash && colcon build --event-handlers console_direct+ --packages-skip tello_gazebo || true"

# 6) que el contenedor venga con el entorno ya sourceado
RUN echo 'source /opt/ros/foxy/setup.bash' >> /root/.bashrc
RUN echo 'source /opt/flock2_ws/install/setup.bash' >> /root/.bashrc

# por seguridad dejamos el container en bash por defecto
CMD ["/bin/bash"]

