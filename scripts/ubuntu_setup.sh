#Install ROS Kinetic

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y build-essential git
sudo apt-get install -y ros-kinetic-desktop-full ros-kinetic-joy

sudo rosdep init
rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

#Download repo
mkdir ~/repos
git clone https://github.com/SSModelGit/ModROS.git ~/repos/ModROS
ln -s ~/repos/ModROS/modros ~/catkin_ws/src/modros
ln -s ~/repos/ModROS/modros_examples ~/catkin_ws/src/modros_examples
catkin_make
