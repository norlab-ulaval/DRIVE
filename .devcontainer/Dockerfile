ARG ROS_DISTRO=humble

FROM ros:$ROS_DISTRO AS dev

ARG RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV ROS_DISTRO=${ROS_DISTRO}
ENV RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update -y \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install dependencies
RUN apt-get update -y && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-$(echo $RMW_IMPLEMENTATION | tr '_' '-') \
    screen \
    ssh \
    git \
    python3-pip \
    python3-tk && \
    rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --upgrade pip

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash && source /home/ws/install/setup.bash" >> /home/$USERNAME/.bashrc
ENV SHELL /bin/bash

USER $USERNAME
CMD ["/bin/bash"]

FROM dev AS prod
COPY . /home/ws
WORKDIR /home/ws

RUN sudo apt-get update -y

RUN sudo chown -R $(whoami) /home/ws/ \
    && pip install -e . \
    && rosdep update \
    && rosdep install --from-paths ros --ignore-src -y -r \
    && bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
ENTRYPOINT ["/bin/bash", "/home/ws/entrypoint.bash"]

FROM dev AS gui
RUN pip install --no-cache-dir customtkinter==5.2.2 pillow
WORKDIR /home/ws/src/DRIVE_GUI
ENV QT_X11_NO_MITSHM=1
CMD ["python3", "home.py"]
