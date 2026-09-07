ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base-noble AS base
LABEL org.opencontainers.image.source="https://github.com/jcjcjc0705/three_finger_gripper"

ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

ENV ROS_DISTRO=${ROS_DISTRO} \
    ROS2_WS=/ws

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-dynamixel-sdk \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-v4l2-camera \
        ros-${ROS_DISTRO}-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# debug mode runs the container with --entrypoint bash and docker exec never
# goes through the entrypoint either, so interactive shells source it here.
# Single quotes defer expansion to shell start, where ROS2_WS is /ws in dev and
# /opt/omx_arm in runtime.
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash --' >> /etc/bash.bashrc \
 && echo '[ -f ${ROS2_WS}/install/setup.bash ] && source ${ROS2_WS}/install/setup.bash --' \
    >> /etc/bash.bashrc

FROM base AS dev

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ubuntu

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        git vim less gdb \
    && rm -rf /var/lib/apt/lists/*

RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
RUN rosdep update --rosdistro ${ROS_DISTRO}

WORKDIR ${ROS2_WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

FROM base AS builder

ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
COPY src ./src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash -- \
 && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM base AS runtime

ARG USERNAME=ubuntu
ENV ROS2_WS=/opt/omx_arm

COPY --from=builder /build/install ${ROS2_WS}/install

USER ${USERNAME}
WORKDIR ${ROS2_WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
