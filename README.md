Some classes in this folder inherit the corresponding classes of python-urx.
Some classes are completely new and written based on the URscript of python-urx.
The secondary monitor of python-urx was rewritten for better use at the APS.
Since the rtde code of python-urx does not support many feature required for 12ID operation, it is replaced with ur-rtde that I modified from an example code distributed by Universal Robot.
<br>
ur-rtde is included.
<br>
robot12idb.py is a code that I made for 12idb operation. IPs are erased. 
<br>
If you would like to tweak your robot, try
Example:
> python TweakRobot.py 164.54.xxx.xxx
