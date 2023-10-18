ARG ROS_DISTRO="humble"
FROM ros:${ROS_DISTRO}

COPY . /ros2_dev/src/ft_tools_ros2
RUN apt update && apt upgrade -y
RUN apt install -y python3-pip
RUN apt install -y git tree clang-format-14 cppcheck
RUN pip install pre-commit
RUN cd ros2_dev/src && \
    vcs import . < ft_tools_ros2/ft_tools_ros2.repos
