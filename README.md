# Beginner_Tutorials ROS2 ENPM808X

Tutorial Part_Release1.0
Using ROS2 Humble and Object Oriented approach for C++ simple publisher and subscriber program is created in this repository.

Tutorial Part_Release2.0
Modify the publisher subscriber nodes to accomodate a service call such that it changes the published message

## Building and Running Instructions

```bash
# Source to ROS2 HUMBLE
source /opt/ros/humble/setup.bash
# Make your ros2 workspace
mkdir ros2_ws
# Go to the source directory of your ros2 workspace
cd ~/ros2_ws
git clone https://github.com/darshit-desai/beginner_tutorials.git

# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
colcon build --packages-select cpp_pubsub
# After successfull build source the package
. install/setup.bash
# Run the publisher in terminal#1
ros2 run beginner_tutorials talker
# Run the subscriber in terminal#2 (Split the terminal and source ROS2 and the workspace setup.bash)
ros2 run beginner_tutorials listener
# Service call run without launch file in Terminal 3 (Split the terminal and source ROS2 and the workspace setup.bash)
ros2 service call /change_output_server cpp_pubsub/srv/ModOutput "{new_output: Hi I'm Darshit calling the service}"

# Run with the launch file in Terminal 1
ros2 launch cpp_pubsub launch.py freq:=0.5 #Freq needs to be double datatype 
# Service call run with launch file in Terminal 2 (Split the terminal and source ROS2 and the workspace setup.bash)
ros2 service call /change_output_server cpp_pubsub/srv/ModOutput "{new_output: Hi I'm Darshit calling the service}"
```

### CppCheck & CppLint
```bash
# Use the below command for cpp check by moving to root directory of your workspace
cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) --check-config  &> results/cppcheck.txt

# Use the below command for cpp lint by moving to root directory of your workspace 
cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> results/cpplint.txt 

## The results of both are present in results folder insider beginner_tutorials directory
```

## Results Screenshot

Below is the screenshot of the rqt console logging levels:

* Fatal error test
![](results/Fatal_error.png)
* Normal error test
![](results/Serv_Error.png)
* Warning and debug test
![](results/warn_debugmsg.png)
* Service call terminal screenshot
![](results/ServiceCall.png)




    

