## installation
install everything as on the manuals for sphinx and olympe.
then:
```bash
python3 -m pip install -r ~/code/parrot-groundsdk/packages/olympe/requirements.txt
python3 -m pip install rospkg
```
copy anafi_driver/config/olympe_custom_env.sh to /$HOME/code/parrot-groundsdk or your sdk installation folder

### for vscode
add your /code/parrot-groundsdk/out/olympe-linux/final/usr/lib/python/site-packages to settings.json to get intellisense

## use
start sphinx
```bash
source ~/code/parrot-groundsdk/olympe_custom_env.sh
roslaunch anafi_driver anafi_driver.launch
```
