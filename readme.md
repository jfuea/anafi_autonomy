## system requeriments
tested on Ubuntu 18.04 with ros melodic
## installation
install everything as on the manuals for:
- [sphinx](https://developer.parrot.com/docs/sphinx/installation.html)
- [olympe](https://developer.parrot.com/docs/olympe/installation.html)

check your pip version > 18
then:
```bash
python3 -m pip install clang
python3 -m pip install -r ~/code/parrot-groundsdk/packages/olympe/requirements.txt
python3 -m pip install rospkg
```
copy install/olympe_custom_env.sh to /$HOME/code/parrot-groundsdk or your sdk installation folder

### for vscode
add your /code/parrot-groundsdk/out/olympe-linux/final/usr/lib/python/site-packages to settings.json to get intellisense

## use
start sphinx
```bash
sudo systemctl start firmwared.service
roslaunch anafi_gazebo anafi_start_world.launch
```
start anafi_driver
```bash
source ~/code/parrot-groundsdk/olympe_custom_env.sh
roslaunch anafi_driver anafi_driver.launch
```
### joystick
```bash
roslaunch anafi_teleop teleop.launch
```

## ROS API

The following topics and parameters are available on both the simulation and the real robot.
default prefix `anafi/`

### Subscribed topics

* **`cmd_vel`** ([geometry_msgs/Twist])

    Target velocity of the Drone.
    Only:
    linear.x, linear.y and linear.z (m/s)
    angular.z (r/s) are used.

### Published topics

* **`camera/image_raw`** ([sensor_msgs/Image])

    Unrectified images from the hazard avoidance camera.

