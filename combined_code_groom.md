# This file helps in understanding the Full_Potential_Steering_auto.py script.

## Imports:
1. `rospy`
2. `copy`
3. `gps_data`(from `navigation.msg`)
4. `time`
5. `numpy`
6. `imutils`
7. `WheelRpm` (from `traversal.msg`)
8. `traversal.srv`
9. `Bool` (from `std_msgs.msg`)
10. `pyrealsense2`
11. `threading`
12. `std_msgs.msg`
13. `Image`, `Imu`, `NavSatFix` from `sensor_msgs.msg`
14. `YOLO` (from `ultralytics`)
15. `CvBridge` (from `cv_bridge`)
16. `numpy`
17. `pyrealsense2`
18. `Annotator` (from `ultralytics.utils.plotting`)
19. `defaultdict` (from `collections`)
20. `statistics`
21. `cv2`
22. `open3d`
23. `csv`

## Control Flow:
- Creation of a class named `ZedDepth()`.

The `Drive()` class.


## Flow of Execution:


 
## Subscribers:
- `state`
- `/zed2i/zed_node/imu/data`
- `enc_arm`
- `/gps_coordinates`
- `/zed2i/zed_node/rgb/image_rect_color`
- `/zed2i/zed_node/depth/depth_registered`
## Publishers:
- `stm_write`
- `motion`
- `gps_bool`
- `rot`
## Attributes and Methods: 
| `self.bridge` | dhwvh |
- `self.depth`|
- `self.color_image`|
- `self.depth_image`
- `self.latest_xmin`
- `self.latest_ymin`
- `self.latest_xmax`
- `self.latest_ymax`
- `self.annotated_image`
- `self.results= None`
- `self.first_few`
- `self.yaw_initialization_done`
- `self.templatel`
- `self.templater`
- `self.z_angle`
- `self.turn`
- `self.circle_dist`
- `self.dist_thresh`
- `self.angle_thresh`
- `self.kp`
- `self.kp_rot`
- `self.kp_straight_rot`
        self.distance = 10.0
        self.time_bool=False
        for i in range(5):
            print("hey! self.distance = 10", self.distance)
        self.direction = "Not Available"
        self.current_latitude = 0.0
        self.current_longitude = 0.0
        self.ret = False
        self.initial_yaw = 0.0
        self.rotate_angle = 90
        self.angles_dict = defaultdict(list)
        self.searchcalled = False
        self.latlong = defaultdict(list)
        self.latlong[0] = "latitude"
        self.latlong[1] = "longitude"
        self.arrow_numbers = 1
        self.gpscalled = 0
        self.depth_image=None
        self.bridge = CvBridge()
        self.drift_correction_const = 32

        # bag
        #        self.num=i
        #        filename = "imu_data_"+str(self.num)+".bag"
        #        self.bag=rosbag.Bag(filename,'w')
        self.state = False
        self.rot = 0 #No steering mode 
        self.initial_drift_angle = 0

        # search alg by turning realsense
        self.enc_data = 0
        self.start_time = time.time()
        self.time_thresh = 20
        self.time_thresh_rot = 5
        self.pub = rospy.Publisher("stm_write", std_msgs.Int32MultiArray, queue_size=10)
        self.init = False
        self.start_angle = 50
        self.angle_thresh = 4
        # self.manjari = False
        self.count_arrow = 0
        self.image_avbl = False
        self.pls_call_rot_once=False
        
        self.base_index=1        #For base rotation
        self.base_rot_dir=1
        
1. `enc_callback()`: Method that is called repeatedly (Callback) and gets the steering encoder values and stores them in `self.enc_data` list (for each of the 4 wheels) along with directional correction.
2. `rotinplace_callback()`: Callback that receives, stores and prints 0 or 1 whether the bot should rotate in place or not.
3. `autonomous_motion_callback()`: Callback that gets called when the robot is in autonomous mode and then stores the velocity, angular velocity, and the crab roation boolean. These will then be converted into PWM outputs.
4. `joyCallback()`: Callback that gets called when the robot is in manual mode and stores joystick inputs with respect to the rover's drive, steering, and initialises the list for pwm value storage based on whether `self.steer_islocked` and `self.full_potential_islocked` are `True` or `False`. There is also a provision to switch between autonomous and manual modes.
5. `spin()`: While the rospy node is still running, it calls the `main()` function every `10 Hz`, sleeps for a certain duration and then publishes the final pwm message for the motor.
6. `main()`: Prints certain lines based on the control state and rotation. It also calls the `autonomous_control()`, `steering()` and `drive()` functions. Finally, it updates `self.print_ctrl`
7. `autonomous_control()`: When the mode is autonomous, it decides whether to rotate in place, go forward or reset based on the `self.rotin` value. It also ensures these commands are not repeated using the values of `self.state_init`.
8. `steering()`: It works based on the states of `self.steer_islocked` and `self.full_potential_islocked`. It then calls `steer()` or directly sends the PWM signals for the motors for each wheel and looks at the joystick commands or states being used to confirm the movement.
9. `drive()`: It uses the Joystick states to compute the PWM values of velocity and angular velocity (along with their average values) and also updates the PWM message object for drive motors.
10. `steer()`: It uses encoder feedback, Proportion control and then publishes the steering PWM values.
