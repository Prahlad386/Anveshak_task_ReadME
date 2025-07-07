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

The Drive() class manages and publishes PWM commands to control both the steering angles and driving velocities of the rover in both manual and autonomous modes.
### Autonomous:
1. Initialization:
2. Storing steering encoder values:
3. Storing autonomous pwm values:
4. Deciding movement option for autonomous case:
5. Configuring the steering and managing the steering motion:
6. Configuring the velocity and angular velocity for translation:
7. Publishing the message object of the PWM pulse: This contains the velocity, angular velocity and also the steering parameters.

### Manual:
1. Initialization:
2. Joystick command reception:
3. Storing steering encoder values:
4. Configuring the steering and managing the steering motion:
5. Configuring the velocity and angular velocity for translation:
6. Publishing the message object of the PWM pulse: This contains the velocity, angular velocity and also the steering parameters.
## Subscribers:
- joy
- enc_auto
- /motion
- rot
## Publishers:
- motor_pwm
- state
## Attributes and Methods:
- self.pwm_pub:	Publishes motor PWM data as an Int32MultiArray on the 'motor_pwm' topic.
- self.state_pub:	Publishes current driving state (autonomous/manual) as a Bool on the 'state' topic.
- self.control:	List storing mode labels like 'joystick' and 'autonomous'.
- self.pwm_msg: The ROS message object used to publish motor PWM data.
- self.pwm_msg.layout: Layout object that describes dimensions of the message.
- self.pwm_msg.layout.data_offset: Offset within the data array (typically 0).
- self.pwm_msg.layout.dim: List holding dimension metadata (MultiArrayDimension).
- self.pwm_msg.layout.dim[0].size: Size of the first dimension in the layout.
- self.pwm_msg.layout.dim[0].stride: Stride of the data dimension, same as size.
- self.pwm_msg.layout.dim[0].label: Label for the dimension, usually 'write'.
- self.modeupbtn: Joystick button index to increment the drive mode.
- self.modednbtn: Joystick button index to decrement the drive mode.
- self.fb_axis: Joystick axis index for forward/backward motion.
- self.lr_axis: Joystick axis index for lateral (left/right) motion.
- self.forward_btn: Button to align all wheels forward.
- self.parallel_btn: Button to align wheels for crab (parallel) motion.
- self.rotinplace_btn: Button to enable rotation-in-place movement.
- self.autonomous_btn: Button to toggle between manual and autonomous mode.
- self.steer_unlock_axis: Axis to unlock steering control.
- self.steer_samedir_axis: Axis to steer wheels in the same direction.
- self.steer_oppdir_axis: Axis to steer wheels in opposite directions.
- self.full_potential_unlock_axis: Axis to unlock full potential steering mode.
- self.fl_wheel_axis: Joystick axis to control front-left wheel direction.
- self.fr_wheel_axis: Joystick axis to control front-right wheel direction.
- self.bl_wheel_axis: Joystick axis to control back-left wheel direction.
- self.br_wheel_axis: Joystick axis to control back-right wheel direction.
- self.drive_ctrl: Stores joystick values for forward/backward and lateral motion.
- self.steering_ctrl_locked: Stores button state inputs for locked steering configurations.
- self.curve_opp_str: Additional curvature or strength value for opposite-direction steering.
- self.steer_islocked: Flag indicating whether steering is in locked configuration.
- self.steering_ctrl_unlocked: Stores joystick/button control inputs for unlocked steering.
- self.steering_ctrl_pwm: PWM values for unlocked joystick steering.
- self.full_potential_islocked: Boolean flag indicating whether full potential steering is locked.
- self.full_potential_pwm: PWM control values for each wheel in full potential steering.
- self.mode: Current drive mode (0 to 4), affecting behavior and configuration.
- self.prints_per_iter: Number of control loop iterations between each debug print.
- self.print_ctrl: Counter to track and control when to print debug data.
- self.steering_complete: Indicates whether the steering action is complete.
- self.d_arr: Drive multipliers array for different drive modes.
- self.s_arr: Steering multipliers for different drive modes (typically all same).
- self.enc_data: Current encoder values for each wheel.
- self.initial_enc_data: Encoder values stored at startup for relative calculation.
- self.initial_value_received: Boolean flag to indicate if the initial encoder reading has been received.
- self.kp_steer: Proportional constant used in steering PID control.
- self.qsize: Size of velocity and omega smoothing queues.
- self.vel_prev: Queue to store recent velocity values for smoothing.
- self.omega_prev: Queue to store recent angular velocity (omega) values.
- self.start_time: Time when the run started, used for time-based checks.
- self.time_thresh: Time threshold (in seconds) for decision making.
- self.error_thresh: Steering error threshold (in degrees) for control accuracy.
- self.rotinplace: Indicates whether the rover is performing a rotation-in-place.
- self.crab_rotate: Indicates whether the rover is performing a crab + rotate maneuver.
- self.autonomous_vel: Linear velocity set during autonomous operation.
- self.autonomous_omega: Angular velocity set during autonomous operation.
- self.rotin: Flag or counter related to rotation logic in autonomous mode.
- self.state: Current operation mode: False = manual, True = autonomous.
- self.state_init: Initialization state list used for state transitions or startup logic. |
- enc_callback(): Method that is called repeatedly (Callback) and gets the steering encoder values and stores them in self.enc_data list (for each of the 4 wheels) along with directional correction.
- rotinplace_callback(): Callback that receives, stores and prints 0 or 1 whether the bot should rotate in place or not.
- autonomous_motion_callback(): Callback that gets called when the robot is in autonomous mode and then stores the velocity, angular velocity, and the crab roation boolean. These will then be converted into PWM outputs.
- joyCallback() - Callback that gets called when the robot is in manual mode and stores joystick inputs with respect to the rover's drive, steering, and initialises the list for pwm value storage based on whether self.steer_islocked and self.full_potential_islocked are True or False. There is also a provision to switch between autonomous and manual modes.
- spin(): While the rospy node is still running, it calls the main() function every 10 Hz, sleeps for a certain duration and then publishes the final pwm message for the motor.
- main(): Prints certain lines based on the control state and rotation. It also calls the autonomous_control(), steering() and drive() functions. Finally, it updates self.print_ctrl
- autonomous_control(): When the mode is autonomous, it decides whether to rotate in place, go forward or reset based on the self.rotin value. It also ensures these commands are not repeated using the values of self.state_init.
- steering(): It works based on the states of self.steer_islocked and self.full_potential_islocked. It then calls steer() or directly sends the PWM signals for the motors for each wheel and looks at the joystick commands or states being used to confirm the movement.
- drive(): It uses the Joystick states to compute the PWM values of velocity and angular velocity (along with their average values) and also updates the PWM message object for drive motors.
- steer(): It uses encoder feedback, Proportion control and then publishes the steering PWM values.
 
