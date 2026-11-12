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

# Only upgrade pip if NOT Jazzy
RUN if [ "${ROS_DISTRO}" != "jazzy" ]; then \
        python3 -m pip install --upgrade pip; \
    fi

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash && source /home/ws/install/setup.bash" >> /home/$USERNAME/.bashrc
ENV SHELL=/bin/bash

USER $USERNAME
CMD ["/bin/bash"]

FROM dev AS prod
ARG USERNAME=USERNAME
# Needed on Jazzy (PEP 668), harmless on Humble
ENV PIP_BREAK_SYSTEM_PACKAGES=1

RUN sudo mkdir -p /home/ws && sudo chown ${USERNAME}:${USERNAME} /home/ws
WORKDIR /home/ws

# Dependencies are installed from the manifests only, so that editing the
# source code does not invalidate these (slow) layers.
COPY --chown=${USERNAME}:${USERNAME} ros/package.xml ros/package.xml
RUN sudo apt-get update -y \
    && rosdep update \
    && rosdep install --from-paths ros --ignore-src -y \
    && sudo rm -rf /var/lib/apt/lists/*

COPY --chown=${USERNAME}:${USERNAME} pyproject.toml pyproject.toml
RUN python3 -c "import sys; \
toml = __import__('tomllib') if sys.version_info >= (3, 11) else __import__('pip._vendor.tomli', fromlist=['_']); \
print('\n'.join(toml.load(open('pyproject.toml', 'rb'))['project']['dependencies']))" > /tmp/requirements.txt \
    && pip install -r /tmp/requirements.txt

# Source code
COPY --chown=${USERNAME}:${USERNAME} . /home/ws
RUN pip install --no-deps -e . \
    && bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
ENTRYPOINT ["/bin/bash", "/home/ws/entrypoint.bash"]
