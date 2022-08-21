 Designed for use with MJBOTS Moteus motors using an fdcanusb.

 Files are located at
https://github.com/georgevargas/GUI-APP-for-Moteus-Motor-Control/tree/master

![image0](https://user-images.githubusercontent.com/10259360/185726576-b0bd35a8-807b-4cd6-8bfe-685f9b3f9978.jpeg)


https://user-images.githubusercontent.com/10259360/185812413-ca915e5a-b62b-4de5-8d11-15c33791a3f6.mov


This is a QT GUI program which allows positions to be manually set for each motor and recorded along with other parameters such as velocity, torque and delay etc.. Then you can play back the sequence in cycles. 

More movements can be added to the sequence without limit.
Movements on different motors may be overlapped by setting the movement delay to zero on intermediat steps.

The sequence can be saved to a file so it can be run by opening the file. The file is in text format so it can be edited to change parameters or delete steps.

Individule motors may also be commanded using position, status or stop buttons.

If the fdcanusb was not installed, Follow the instructions at: https://github.com/mjbots/fdcanusb/blob/master/70-fdcanusb.rules
This consists of doing the following;

copy file 70-fdcanusb.rules to /etc/udev/rules.d folder

Then run the following from a terminal:

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty

This is a QT GUI program which allows positions to be manually set for each motor and recorded along with other parameters such as velocity, torque and delay etc.. Then you can play back sequence in cycles. More movements can be added to the sequence without limit.
The sequence can be saved to a file so it can be run by opening the file.
The file is in text format so it can be edited to change parameters or delete steps.

This program needs to be opened the first time with qt creator so it can be built for your system. After running the program once, a desktop shortcut may be created to run the program after that. The executable file is in a folder qt creates called build-MotorQT_threaded-Desktop-Debug. The executable file is called MotorQT_threaded.

If you don't have qt creator it can be installed as follows;


Install Qt on Ubuntu
https://newbedev.com/install-qt-on-ubuntu

How to Install libqwt-qt5-dev in Ubuntu 18.04
https://www.howtoinstall.me/ubuntu/18-04/libqwt-qt5-dev/

At the terminal enter
qtcreator

click open project

find MotorQT_threaded.pro

click open
click Configure Project

from the menu select Build then Build All

Make sure the fdcanusb is pluged in to the motor controller and the USB.

when the build is done,
select Build Run, or hit the Play icon on the left side.

Once running the Motor Control section contains Parameters which can be changed using the counter or slider. 

Typing into the counter is the easiest way to precisly change a value.

Use the buttons to command the controller.

Motor allows the selection of a motor ID

Go to Stop will move to the indicated stop position.

Read Status will return the status

Stop Motor will stop the selected motor

Run Forever sends NAN as a stop position so it will run until a motor position limit is reached, or forever.

Running a sequence of motor movements.

Stop a motors so they can be manually moved.

Select a motor ID

Move a motor manually to the position desired.

Make sure the Move Delay is set large enough for the move to complete. Movements on different motors may be overlapped by setting the movement delay to zero on intermediat steps.

Click the 'Rec Position' button to add the position and all parameters to the list.

Repeat this procedure for as many steps as desired.

When done click the 'Run Recorded' button and see if it acts as expected.

Use Clear Rec if desired to erase the recorded list.

Click Stop Cycles to stop cycling through the recorded positions.

If desired you may save the Recorded list to a text file using the File Save in the menu.

You can use File Open to read in a previously saved recorded list.

You can edit the text file to change parameters or delete an entire move.

Be careful not to remove part of a sequence. It is OK to remove the whole sequence including the sequence number and all parameters.
It is OK to cut and paste a whole sequence and then change the parameters.
The sequence number value is ignored but they must be there.
The sequence is executed from top to bottom.

note a desktop shortcut may be created to run the program in folder build-MotorQT_threaded-Desktop-Debug. The executable file is called MotorQT_threaded.
