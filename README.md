# OpenJTalk ROS 2 managed node
How to Speak Japanese in ROS2! Japanese Text-to-Speech Node using OpenJTalk

## log
```.sh
2023/04/07 append speed param to ros2_openjtalk_interfaces/Talk
2023/04/01 first commit (device_name param is not implemant now)
```

## prepare for python module
```.sh
python3 -m pip install -U pip setuptools
python3 -m pip install -U numpy
python3 -m pip install -U pyaudio
python3 -m pip install -U pyopenjtalk
```

## clone ros2_openjtalk
```.sh
cd ~/dev_ws/src
git clone https://github.com/koki-ogura/ros2_openjtalk.git
```

## build ros2_openjtalk
```.sh
cd ~/dev_ws
colcon build --symlink-install --packages-select ros2_openjtalk_interfaces
. install/local_setup.zsh
colcon build --symlink-install --packages-select ros2_openjtalk
. install/local_setup.zsh
```

## launch ros2_openjtalk node
```.sh
ros2 launch ros2_openjtalk bringup.launch.py
```
Don't worry about the message that ALSA is not set up on your system.

## test via command line (0.1 <= speed <= 2.0)
```.sh
ros2 service call /ros2_openjtalk_node/talk ros2_openjtalk_interfaces/Talk '{message: こんにちは世界, speed: 0.5}'
```

## run ros2_openjtalk test program (can run other pc)
```.sh
ros2 run ros2_openjtalk ros2_openjtalk_test
```

## service
### Talk
```.py
string message
---
bool success
```
