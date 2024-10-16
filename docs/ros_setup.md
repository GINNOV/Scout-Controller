# ROS 1.0 SETUP
ROS comes in different flavors, so we have to create an environment that matches the robot's flavor.
[Go Back](../readme.md)

## Environment
This specific version of ROS is 1.0 (noetic)

1. conda create -n ROS python=3.9
2. conda activate ROS
3. conda config --add channels conda-forge
4. conda config --add channels robostack
5. conda config --set channel_priority strict
6. conda install ros-noetic-desktop-full
7. conda install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools

Steps **3** and **4** are only needed if you haven't added them.
The original [instructions](https://atom-robotics-lab.github.io/wiki/markdown/ros/ROS_installation/installation_on_mac.html) has issues, the one above work for sure.

### Test the installation

1. `conda activate ROS`
2. `roscore`
3. in another terminal window, after you activate the ROS environment, run `rviz` tool from command line to have visual control via ros, not as exciting as the linux equivalent but it will give you some bot vibes :-)

**note**
to install additional ROS packages that aren't in the installation, use this command `conda install ros-noetic-"package name here"` - packages are [here](https://robostack.github.io/noetic.html) you won't need those for running what I am sharing.

# Configure Networking
Scout is already running a ROS master node, we need to configure the computer to join that node. We do that using two environment variables. We set where the master is running and which ip address will join the master node

the robot is set on linaro-alip DNS name.

`export ROS_MASTER_URI=http://IP:11311`
`export ROS_IP=your_computer_IP`
`export ROS_HOSTNAME=YOUR_COMPUTER_IP`

if all worked out, type rostopic will show all services exposed by the robot.


