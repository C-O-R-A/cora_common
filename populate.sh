#!/usr/bin/env bash

# ------------------------------------------------------------
# Usage:
#   ./populate.sh <robot_name>
#
# Example:
#   ./populate.sh mybot
# ------------------------------------------------------------

set -e

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <robot_name>"
    exit 1
fi

ROBOT_NAME=$1

echo "Populating ROS 2 src folder: $(pwd)"
echo "Robot name: $ROBOT_NAME"
echo ""


# 1. Interfaces package (C++)
echo "Creating ${ROBOT_NAME}_interfaces (ament_cmake)..."
ros2 pkg create "${ROBOT_NAME}_interfaces" \
    --build-type ament_cmake \
    --dependencies builtin_interfaces std_msgs geometry_msgs sensor_msgs


# 2. Description package (C++)
echo "Creating ${ROBOT_NAME}_description (ament_cmake)..."
ros2 pkg create "${ROBOT_NAME}_description" \
    --build-type ament_cmake \
    --dependencies xacro

mkdir -p "${ROBOT_NAME}_description/urdf"
mkdir -p "${ROBOT_NAME}_description/meshes"


# 3. Bringup package (Python)
echo "Creating ${ROBOT_NAME}_bringup (ament_python)..."
ros2 pkg create "${ROBOT_NAME}_bringup" \
    --build-type ament_python \
    --dependencies rclpy launch launch_ros

mkdir -p "${ROBOT_NAME}_bringup/launch"

# 4. Vision package (Python)
echo "Creating ${ROBOT_NAME}_vision (ament_python)..."
ros2 pkg create "${ROBOT_NAME}_vision" \
    --build-type ament_python \
    --dependencies rclpy sensor_msgs cv_bridge image_transport cv2 numpy

# 5. Messages package (C++)
echo "Creating ${ROBOT_NAME}_msgs (ament_cmake)..."
ros2 pkg create "${ROBOT_NAME}_msgs" \
    --build-type ament_cmake \
    --dependencies std_msgs geometry_msgs  sensor_msgs

mkdir -p "${ROBOT_NAME}_msgs/msg"
mkdir -p "${ROBOT_NAME}_msgs/srv"
mkdir -p "${ROBOT_NAME}_msgs/action"

# 6. Ethernet communication package (C++)
echo "Creating ${ROBOT_NAME}_codi (ament_python)..."
ros2 pkg create "${ROBOT_NAME}_codi" \
    --build-type ament_python \
    --dependencies rclcpp std_msgs geometry_msgs moveit codi

echo ""
echo "------------------------------------------------------------"
echo "Finished creating ROS 2 packages in: $(pwd)"
echo ""
echo "Next steps:"
echo "  cd ..  # go back to workspace root"
echo "  colcon build"
echo "  source install/local_setup.bash"
echo "------------------------------------------------------------"

echo cd ..
echo colcon build --symlink-install
echo source install/setup.bash
