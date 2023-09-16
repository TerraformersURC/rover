FROM armdocks/ros-dev:noetic

# add a non-root user
ARG USERNAME=noetic
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV ROS_DISTRO=noetic

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /home/$USERNAME/rover_ws/devel/setup.bash ]; then source /home/$USERNAME/rover_ws/devel/setup.bash; fi" >> /home/$USERNAME/.bashrc

USER $USERNAME

ENV DEBIAN_FRONTEND=