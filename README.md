# This file helps in understanding the Full_Potential_Steering_auto.py script.

## Imports:
1. rospy
2. copy
3. Joy(from sensor_msgs.msg)
4. time
5. Int8, Float32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension, Float32MultiArray, Bool (from std_msgs.msg)
6. queue
7. add (from operator)
8. WheelRpm (from traversal.msg)

## Control Flow:
- Creation of a class named Drive().
- 
The Drive() class manages and publishes PWM commands to control both the steering angles and driving velocities of the rover in both manual and autonomous modes.
## Subscribers:
- joy
- enc_auto
- /motion
- rot
## Publishers:
- motor_pwm
- state
## Attrubutes and Methods:
- enc_callback(): Method that is called repeatedly (Callback) and gets the steering encoder values and stores them in self.enc_data list (for each of the 4 wheels) along with directional correction.
- rotinplace_callback(): Callback that receives, stores and prints 0 or 1 whether the bot should rotate in place or not.
- autonomous_motion_callback(): Callback that gets called when the robot is in autonomous mode and then stores the velocity, angular velocity, and the crab roation boolean. These will then be converted into PWM outputs.
- joyCallback() - Callback that gets called when the robot is in manual mode and stores joystick inputs with respect to the rover's drive, steering, and initialises the list for pwm value storage based on whether self.steer_islocked and self.full_potential_islocked are True or False. There is also a provision to switch between autonomous and manual modes.
- spin(): While the rospy node is still running, it calls the main() function every 10 Hz, sleeps for a certain duration and then publishes the final pwm message for the motor.
- main(): Prints certain lines based on the control state and rotation. It also calls the autonomous_control(), steering() and drive() functions. Finally, it updates self.print_ctrl
- autonomous_control(): When the mode is autonomous, it decides whether to rotate in place, go forward or reset based on the self.rotin value. It also ensures these commands are not repeated using the values of self.state_init.
- steering(): It works based on the states of self.steer_islocked and self.full_potential_islocked. It then call steer() or directly sends the PWM signals for the motors for each wheel and looks at the joystick commands or states being used to confirm the movement.
- drive(): It uses the Joystick states to compute the PWM values of velocity and angular velocity and also updates the PWM message object for drive motors.
- steer(): It uses encoder feedback, Proportion control and then publishes the steering PWM values.
 
