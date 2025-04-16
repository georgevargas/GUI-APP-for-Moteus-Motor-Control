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

There is a Reference Manuel included with the code files;
https://github.com/georgevargas/GUI-APP-for-Moteus-Motor-Control/blob/master/Motius%20GUI%20Reference%20Manual.pdf
