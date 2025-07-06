# This file helps in understanding the bevel_arm.py script.

## Imports:
1. rospy
2. std_msgs.msg
3. sensor_msgs.msg

## Control Flow:
- Creation of a class named Node().

The Node() class manages and publishes PWM commands to control both the steering angles and driving velocities of the rover in both manual and autonomous modes.
1. All the necessary parameters like outbuff, buttons, etc are initialised.
2. joyCallback() is run 

## Subscribers:
- joy_arm
## Publishers:
- stm_write
## Attrubutes and Methods:
- joyCallback(): It is a callback function that reads the joystick values and scales them for PWM range of 255. It also interprets the buttons pressed and factors in their effects. Finally, outbuff is updated for the various joint movements of the bevel.
- run(): While rospy is running, the createMsg() function is run and the message object returned is then published. This is carried out at the rate of 50 Hz.
- createMsg(): It helps in structuring the message object in a better manner using msg.layout and also to include the length of the list msg.data as well as the intention to write the data somewhere.

