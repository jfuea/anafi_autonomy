## system requeriments
tested on Ubuntu 18.04 with ros melodic
## installation
install everything as on the manuals for:
- [sphinx](https://developer.parrot.com/docs/sphinx/installation.html)
- [olympe](https://developer.parrot.com/docs/olympe/installation.html)
* note: may need to install libclang-dev for compiling
check your pip version > 18
then:
```bash
python3 -m pip install clang
python3 -m pip install -r ~/code/parrot-groundsdk/packages/olympe/requirements.txt
python3 -m pip install rospkg
```
copy install/olympe_custom_env.sh to /$HOME/code/parrot-groundsdk or your sdk installation folder
compile with catkin_make
### python3 cvbridge issue solution
in a separate workspace(e.g. cvbridge_build_ws) follow the [instructions](https://cyaninfinite.com/ros-cv-bridge-with-python-3/)
python3 -m pip install scikit-build
python3 -m pip install opencv-python
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
source ~/tmp/cvbridge_build_ws/install/setup.zsh --extend
roslaunch anafi_driver anafi_driver.launch
```
### joystick
```bash
roslaunch anafi_teleop teleop.launch
```

### know issues
exist a delay in the image delay, a workaround is to launch anafi_driver twice(also in the first time, a requeriment is to show the image topic)
e.g.
```bash
roslaunch anafi_driver anafi_driver.launch
```
```bash
rosrun image_view image_view image:=/anafi/camera/image_raw
```
repeat one time more to get images without delay
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

