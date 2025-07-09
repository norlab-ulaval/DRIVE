ARG ROS_DISTRO=humble

FROM ros:$ROS_DISTRO AS dev

ENV ROS_DISTRO=${ROS_DISTRO}
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-foxglove-bridge screen -y
RUN apt-get install -y python3-pip
RUN python3 -m pip install --upgrade pip
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash && source /home/ws/install/setup.bash" >> /home/$USERNAME/.bashrc
ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************
# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]

FROM dev AS prod
COPY . /home/ws
WORKDIR /home/ws
RUN sudo chown -R $(whoami) /home/ws/ \
    && pip install -e . \
    && rosdep update \
    && rosdep install --from-paths ros --ignore-src -y -r \
    && bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
ENTRYPOINT ["/bin/bash", "/home/ws/entrypoint.bash"]