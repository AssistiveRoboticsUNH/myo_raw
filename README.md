# myo_raw

This package is used by the exercise interfaces (both training and client) to run two myos concurrently. At the time of writing this, 
there is one myo named ```Winnie's Myo```. This name is stored in the myo and is accessed at launch.
In ```scripts/controllable_myo_node.py``` when it activates the myos, it looks for the myo named ```Winnie's Myo```
and always designates that particular myo to be the myo on the LOWER ARM. If this name ever changes, change the constant
that is defined at the top of ```scripts/controllable_myo_node.py```. If you do not know the name of your myos,
the program prints the device names at launch automatically. If no name pops up, 
then the device was never named or the firmware was out of date. You can name the myo in Myo's official gui on windows.  

**Things to know**
- Remember, the named myo is assumed to always be on the lower arm
- When calibrating, you must point your arm directly away from your side 
(like, your feet face forward and your whole arm points right)
- In case you cannot run any of the scripts, including python scripts, check the previldge first. Run ```sudo chmod +x filename``` if it is needed.

## scripts
This folder contains some script files.
###controllable_myo_node.py
The main method takes two optional arguments. The first one indicates whether it is for client interface (false by default), and the second one indicates whether it is "myo-only" (false by default). 
If you run the command ```python  controllable_myo_node.py client```, the program will wait for extra IMU sensors (from Yost Engineering, Inc.) to launch first, and only proceeds to launch Myo if the IMU sensors work properly. We suggest NOT use the IMU sensors because they are not stable and unnecessary for the current tasks. Use either ```python  controllable_myo_node.py client myo-only``` or ```python controllable_myo_node.py```. The launch files have been configured accordingly.

###launch.sh
This script launches the exercise interface for trainer with some system configuration. You should execute the script in this file's directory. On the lab laptop, we also created a desktop file so you can click the icon (with a glass image) to run it automatically.
In the script you may need to replace 'wlan2' with another keyword (e.g. 'eth0') depending on how your network is set up. Use the command ```ifconfig``` to check the setup. Also replace ```source ~/myocp/myo/devel/setup.bash``` with the correct path. 

###launch_client.sh
This script launches the exercise interface for client (patient) with some system configuration. You should execute the script in this file's directory. On the lab laptop, we also created a desktop file so you can click the icon (with a Myo armband image) to run it automatically.
In the script you may need to replace 'wlan2' with another keyword (e.g. 'eth0') depending on how your network is set up. Use the command ```ifconfig``` to check the setup. Also replace ```source ~/myocp/myo/devel/setup.bash``` with the correct path. 

##myo_mdp
This folder contains the algorithm implementation. Subfolder ```myo_python``` is for the training program, and ```myo_python_client``` is for the client program. Some of the files in these two subfolders are the same and can be combined. But we seprate them for now.
