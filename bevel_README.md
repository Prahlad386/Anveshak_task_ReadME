# This file helps in understanding the bevel_arm.py script.

## Imports:
1. rospy
2. std_msgs.msg
3. sensor_msgs.msg

## Control Flow:
- Creation of a class named Node().

The Node() class manages and publishes PWM commands to control both the steering angles and driving velocities of the rover in both manual and autonomous modes.
1. All the necessary parameters like outbuff, buttons, etc are initialised.
2. joyCallback() is run and the joystick commands are stored and scaled for PWM pulse to be interpreted. The data is stored in outbuff list.
3. This outbuff list's values are then used to create a structured message using the createMsg() function.
4. This message is then published in the run() function itself.

## Subscribers:
- joy_arm
## Publishers:
- stm_write
## Variables:
- axes: Stores the scaled joystick axes (to 255 for PWM interpretation).
- buttons: Stores combined button values (when a combination of buttons is pressed) to update the outbuff list.
- outbuff: Temporary list to hold joystick values before assigning to self.outbuff.
- rate: Controls how fast the run function while loop runs (50 Hz).
- msg: Message object created from outbuff and published to stm_write.
## Attributes and Methods:
- self.outbuff: Holds 6 control values for different parts of the robotic arm. It is updated in the joyCallback() function.
- self.pub: Publishes the final message object (msg) to topic stm_write.
- msg.data: Holds the 6 integer values (copied from self.outbuff) that represent joint parameters.
- msg.layout: Contains the structured format of the msg.data array.
- msg.layout.data_offset: The amount by which the starting pointer should move to ignore starting data.
- msg.layout.dim: Describes the array’s shape/dimensions (size, stride, label).
- joyCallback(): It is a callback function that reads the joystick values and scales them for PWM range of 255. It also interprets the buttons pressed and factors in their effects. Finally, outbuff is updated for the various joint movements of the bevel.
- run(): While rospy is running, the createMsg() function is run and the message object returned is then published. This is carried out at the rate of 50 Hz.
- createMsg(): It helps in structuring the message object in a better manner using msg.layout and also to include the length of the list msg.data as well as the intention to write the data somewhere.

