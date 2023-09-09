Designed for use with an MJBOTS moteus controller using an fdcanusb.

Caution: This program allows a sequence of user programmed motor movements to be run in cycles. It is the users responsibility to insure safety so no damage can occur. Remove all obstructions, and run at 1 cycle at first to insure safety.

![Screenshot from 2023-09-08 18-10-07](https://github.com/georgevargas/GUI-APP-for-Moteus-Motor-Control/assets/10259360/d6c92c55-3dd1-4d3a-9b8e-73163b5b1cbc)

https://github.com/georgevargas/GUI-APP-for-Moteus-Motor-Control/assets/10259360/dd57bec8-9009-4d42-a9e1-9305ce02af5a


This is a GUI Application which can be used to create a sequence of movements for multiple moteus controllers. Movements on different motors can be safely overlapped to make things less robotic. This is accomplished through the use of movement delays which can be as short as 0. If the same motor is accessed the software waits for the motor to finish before issuing the next command.

The sequence can then run in cycles. Controller parameters are displayed and may be modified and sent to the controller.

You can then dynamically change the movement parameters to see how to tune the movements.

It is also possible to position to an X,Y coordinate using Inverse Kinematics. It is also possible to display the current X,Y using forward Kinematics.

For example, you can manually move a motor to a position and press the Record Position button. The parameters on the screen will be recorded for that motor. This can be repeated for one or multiple motors.

Then you can press run recorded and the sequence will be executed in any number of cycles.

While running; if the dynamic check box is set, the parameters on the screen will be used instead of the recorded parameters. This allows the tuning of such things as Velocity Limit or Acceleration Limit to see how it can affect the movements. The position plot shows the position and or velocity feedback for the selected motor.

The sequence can be saved to a text file, and later reloaded. The file can be edited to alter the sequence.

Individule motors may also be commanded using position, status or stop buttons.

If the fdcanusb was not installed, Follow the instructions at: https://github.com/mjbots/fdcanusb/blob/master/70-fdcanusb.rules

This consists of doing the following;

copy file 70-fdcanusb.rules to /etc/udev/rules.d folder Then run the following from a terminal:

sudo udevadm control --reload-rules

sudo udevadm trigger --subsystem-match=tty


This program needs to be opened the first time with qt creator so it can be built for your system. After running the program once, a desktop shortcut may be created to run the program after that. The executable file is in a folder qt creates called build-MotorQT_threaded-Desktop-Debug. The executable file is called MotorQT_threaded.

If you don't have qt creator it can be installed as follows;

Install Qt on Ubuntu
https://newbedev.com/install-qt-on-ubuntu

How to Install libqwt-qt5-dev in Ubuntu 18.04
https://www.howtoinstall.me/ubuntu/18-04/libqwt-qt5-dev/

or How to Install qt on Ubuntu 20.04
https://linuxways.net/ubuntu/how-to-install-qt-on-ubuntu-20-04/

install g++13 ubuntu. This code uses c++20 features such as std::format. which is now avalible.

sudo add-apt-repository ppa:ubuntu-toolchain-r/test

sudo apt-get update

sudo apt install gcc-13 gcc-13-base gcc-13-doc g++-13

sudo apt install libstdc++-13-dev libstdc++-13-doc 


At the terminal enter

qtcreator <cr>
 
click open project
 
find MotorQT_threaded.pro
 
click open
 
click Configure Project
 
from the menu
 
select Build then Build All

Make sure the fdcanusb is plugged in to the motor controller and the USB.

when the build is done select from the menu,
 
Build Run, or hit the Play icon on the left side.

Once running the Motor Control section contains Parameters which can be changed using the counter or slider.
 
Typing into the counter is the easiest way to precisely change a value.
 
Use the buttons to command the controller.
 
The Motor combo box allows the selection of a motor ID
 
The Position button will move to the indicated position.
 
Read Status will return the status, Stop Motor will stop the selected motor.
 
Run Forever will run until a motor position limit is reached, or forever.

Limit Max, Limit Min, PID kp. PID kd, and PID ki are parameters on the GUI screen which display moteus controller parameters for the selected motor. These can be edited and sent to the controller with the Write Limits and Write PID buttons.
 
Write limits will send the Limit Min and Limit Max from the GUI to the moteus controller for the selected motor.
 
Write PID will send the PID Parameters from the GUI to the moteus controller for the selected motor.

Conf Write will permanently save any parameters sent to the moteus controller for the selected motor. Note the selected motor should be stopped prior to sending this command.
 
Start Motor will will start the motor at its current position.
 
Set nearest causes the servo to select a whole number of internal motor rotations so that the final position is as close 0 as possible.

Running a sequence of motor movements;
 
Stop a motors so they can be manually moved.
 
Select a motor ID, and set the screen parameters as desired.
 
Move a motor manually to the position desired.
 
Make sure the Move Delay is set large enough for desired the move to complete.
If the delay is less the movement, the software will wait for the movement to complete the next time the motor is used.

Movements on different motors may be overlapped by setting the movement delay to zero or a short delay on intermediat steps.
 
Click the 'Rec Position' button to add the position and all parameters to the list.
 
Repeat this procedure for as many steps as desired.
 
When done click the 'Run Recorded' button and see if it acts as expected.
 
Use Clear Rec if desired to erase the recorded list.
 
Click Stop Cycles to stop cycling through the recorded positions.
 
The Dynamic check box allows the screen parameters to override some recorded parameters for the selected motor. This is useful when tunning movements with different velocities, accelerations for example. Velocity Limit, Accel Limit, Max torque, Feedforward, KP Scale, and KD Scale can be dynamic.

If desired you may save the Recorded list to a text file using the File Save in the menu.
 
You can use File Open to read in a previously saved recorded list.
 
You can edit the text file to change parameters or delete an entire move.
 
Be careful not to remove part of a sequence. It is OK to remove the whole sequence including the sequence number and all parameters.
The sequence number value is ignored but they must be there.
The sequence is executed from top to bottom.
 
mainwindow.h contains an array called Motor_rest_position which can be edited to establish all motor rest positions.
