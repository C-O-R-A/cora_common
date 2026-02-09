sudo apt install ros-jazzy
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-moveit
sudo apt install ros-jazzy-moveit-py
sudo apt install ros-jazzy-moveit-servo
sudo rosdep-install

pip install -r requirements.txt

sudo apt update && sudo apt upgrade

cd .. && colcon build --symlink-install

source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ./bashrc
source /install/setup.bash
echo "source /install/setup.bash" >> ./bashrc
export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

set -e

echo "Setting ethernet hostname to cora"
sudo hostnamectl set-hostname cora

echo "Installing Avahi"
sudo apt update
sudo apt install -y avahi-daemon

echo "Enabling Avahi"
sudo systemctl enable avahi-daemon
sudo systemctl start avahi-daemon

echo "Done. Reboot recommended."

# run with: './install.sh'

# NOTES
# To get the moveit setup assistant working in vscode, unset the gtk path using 'unset GTK_PATH'
#
# Make sure cmake file contains:
#
# install(
#   DIRECTORY
#       <folder_name>
#   DESTINATION share/${PROJECT_NAME}
# )
#
# Also make sure onshape config files are in the source root and point to 'description_pkg/meshes/' and not to 'meshes/'
# 
# if the setup assistant segfaults after clicking load model, install older rviz snapshot: https://github.com/moveit/moveit2/issues/3546#issuecomment-32666
#   wget http://snapshots.ros.org/jazzy/2025-05-23/ubuntu/pool/main/r/ros-jazzy-rviz-common/ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb
#
#   sudo apt --fix-broken install
#
#   sudo dpkg -i ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb
#
#
# You must add accell limits to joint limits config file
#
# End effector should have parent link L5, not end effector
#
# In controllers.yaml, for the gripperaction controller, 'joints' --> 'joint'