import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix,Imu,LaserScan
from std_msgs.msg import Int32,Float32MultiArray


HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025
MAX_SPEED = 7.0

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__names = ["left motor 1",  "left motor 2",  "left motor 3", 
            "right motor 1", "right motor 2", "right motor 3"]
    
        self.__motors=[]
        #get motor tags
        for i in range(0,6):
            self.__motors.append(self.__robot.getDevice(self.__names[i]))
            self.__motors[i].setPosition(float('inf'))
            self.__motors[i].setVelocity(0)

        #keyboard
        
        self.__keyboard = self.__robot.keyboard
        self.__keyboard.enable(self.__timestep)
        self.__keyboard2= self.__robot.keyboard
        self.__keyboard2.enable(self.__timestep)
        
        
        #gps
        self.__gps= self.__robot.getDevice("gps")
        self.__gps.enable(self.__timestep)

        #compass
        self.__compass= self.__robot.getDevice("compass")
        self.__compass.enable(self.__timestep)

        self.__target_twist = Twist()
        
        #lidar
        self.__lidar= self.__robot.getDevice("lidar")
        self.__lidar.enable(25)

        #accc
        self.__acc= self.__robot.getDevice("accelerometer")
        self.__acc.enable(self.__timestep)

        #gyro
        self.__gyro= self.__robot.getDevice("gyro")
        self.__gyro.enable(self.__timestep)
        
        rclpy.init(args=None)
        self.mode=0
        
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__publisherGPS = self.__node.create_publisher(NavSatFix, 'gps_data', 10)
        self.__publisherCompass=self.__node.create_publisher(Imu, 'compass_data', 10)
        self.__publisherLidar= self.__node.create_publisher(LaserScan, 'scan', 10)
        self.__publisherMode= self.__node.create_publisher(Int32, 'integer_topic', 10)
        self.__publisherAcc = self.__node.create_publisher(Float32MultiArray, 'acceleration', 10)
        self.__publisherGyro = self.__node.create_publisher(Float32MultiArray, 'gyro', 10)
        self.timer_ = self.__node.create_timer(0.5, self.publish_data)
        self.__node.get_logger().info("\nYou can drive this robot:\n  Press 'Q' to change the autopilot to the keyboard mode \nSelect the 3D window and use WASD:\n Press 'Q' to return to the autopilot mode\n Press 'P' to get the robot position\n")
        self.__node.get_logger().info(f"timestep {self.__timestep}")  


    

    def __cmd_vel_callback(self, twist):
        
        self.__target_twist=twist
        
    def publish_data(self):
        # Obtener la posición actual del GPS en Webots
        gps_position = self.__gps.getValues()
        compass_values=self.__compass.getValues()
        lidar_data=self.__lidar.getRangeImage()
        # Crear un mensaje NavSatFix con la información de la posición
        msgGPS = NavSatFix()
        msgGPS.latitude = gps_position[0]
        msgGPS.longitude = gps_position[1]
        msgGPS.altitude = gps_position[2]
        #print(f"position: {{{gps_position[0]}, {gps_position[1]}}}")  
        msgCompass=Imu()
        msgCompass.header.frame_id = 'compass'
        msgCompass.orientation.x = compass_values[0]
        msgCompass.orientation.y = compass_values[1]
        msgCompass.orientation.z = compass_values[2]


        msgLidar = LaserScan()
        msgLidar.header.frame_id = 'lidar'        
        msgLidar.ranges = lidar_data
        msgLidar.angle_increment = float(self.__lidar.getHorizontalResolution())
        msgLidar.range_min = self.__lidar.getMinRange()
        msgLidar.range_max = self.__lidar.getMaxRange()

        msgMode= Int32()
        msgMode.data=self.mode

        msgAcc=Float32MultiArray()
        msgAcc.data=self.__acc.getValues()

        msgGyro=Float32MultiArray()
        msgGyro.data=self.__gyro.getValues()
        
        # Publicar el mensaje
        #print(f'Compass orientation: ({msgCompass.orientation.w}, {msgCompass.orientation.x}, {msgCompass.orientation.y}, {msgCompass.orientation.z})')
        self.__publisherGPS.publish(msgGPS)
        self.__publisherCompass.publish(msgCompass)
        self.__publisherLidar.publish(msgLidar)
        self.__publisherMode.publish(msgMode)
        self.__publisherAcc.publish(msgAcc)
        self.__publisherGyro.publish(msgGyro)
        
    def check_keyboard(self):
        speeds = [0.0, 0.0]
        LEFT, RIGHT = range(2)       
        global old_key
        key = self.__keyboard.getKey()       
        X, Y= range(2)
        if key >= 0:
            if key == ord('W') and self.mode==1:
                speeds[LEFT] = MAX_SPEED
                speeds[RIGHT] = MAX_SPEED
                
            elif key == ord('S') and self.mode==1:
                speeds[LEFT] = -MAX_SPEED
                speeds[RIGHT] = -MAX_SPEED                
            elif key == ord('D') and self.mode==1:
                speeds[LEFT] = MAX_SPEED
                speeds[RIGHT] = -MAX_SPEED                
            elif key == ord('A') and self.mode==1:
                speeds[LEFT] = -MAX_SPEED
                speeds[RIGHT] = MAX_SPEED                
            elif key == ord('P'):
                if key!= old_key:
                    gps_position= self.__gps.getValues()
                    self.__node.get_logger().info(f'position: {{{gps_position[X]}, {gps_position[Y]}}}')                    
            elif key == ord('Q'):
                if key!= old_key:
                    if self.mode == 0 or self.mode ==2:
                        self.mode = 1
                        self.__node.get_logger().info('mode changed to keyboard')
                    else:
                        self.mode = 0
                        self.__node.get_logger().info('mode changed to autopilot')  
            elif key == ord('L'):
                if key!= old_key:
                    if self.mode == 0:
                        self.mode = 2
                        self.__node.get_logger().info('mode autopilot changed to lidar')
                        
                    else:
                        self.mode = 0
                        self.__node.get_logger().info('mode returned to autopilot')     
        old_key=key
        if self.mode==1:
            self.robot_set_speed(speeds[LEFT], speeds[RIGHT])
        
        
    def work_autopilot(self):
        forward_speedR = self.__target_twist.linear.x
        forward_speedL = self.__target_twist.linear.y
        self.robot_set_speed(forward_speedL,forward_speedR)
        
         

    def robot_set_speed(self, left, right):        
        for i in range(3):
            self.__motors[i].setVelocity(left)
            self.__motors[i + 3].setVelocity(right)    
                   

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)               
        if(self.mode==0 or 2):
            self.work_autopilot()            
        self.check_keyboard()
        
        