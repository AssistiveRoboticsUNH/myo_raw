# myo_raw

This package is used by the exercise interface to run two myos concurrently. At the time of writing this, 
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
