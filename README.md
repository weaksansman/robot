# robot
code for rescue robot (i don't have qrcam code bruh)
purpose for this is just ehhh you know just git clone and make program can be use easily and i don't want other people to acces this other than for people who i know


#Require
install ros2 humble from
`https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html`

install micro-ROS agent from 
`https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca`

after you have all requirement do these step
Clone the repo
`git clone https://github.com/weaksansman/robot.git`
Set up your ROS 2 workspace
`mkdir -p ~/ros2_ws/src`          #create folder ~/ros2_wssrc (if already have don't worry it doesn't make another folder ros2_ws and src
`cp -r robot ~/ros2_ws/src/`       #move repo to ~/ros2_ws/src
`cd ~/ros2_ws`                    #go into ~/ros2_ws
`colcon build`                    


Source the workspace
`source install/setup.bash`

you can now safely run node
`ros2 run robot UDP #for esp32`
For ESP32 (start micro-ROS agent first, then):

`ros2 run robot UDPS #for mega`

