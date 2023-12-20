### Requirement
```
pip3 instlla platformio
```

### Installation
```bash
rosdep install --from-paths . --ignore-src -r -y
catkin bt
```

### Preparation
```bash
sudo adduser $USER dialout
sudo cp ./00-teensy.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Build and write
```bash
cd ./proximity
rosrun rosserial_arduino make_libraries.py lib
platformio run --target upload  # maybe cannot find heades. If so, please launch new terminal and try after source-ing the workspace again
```
NOTE: replace `proximity` by `pressure` if you want to use pressure sensors.

### launch
```bash
roslaunch fingertip_dual_sensor sensor.launch
```
