import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu, LaserScan, Joy
from std_msgs.msg import Int32, Float32MultiArray

# Velocidad máxima del robot
MAX_SPEED = 7.0
# Incremento de posición del motor de la camara
MOTOR_POSITION_INCREMENT = 0.005

class MyRobotDriver:
    def init(self, webots_node, properties):
        # Inicialización del nodo Webots y obtención de dispositivos
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__names = ["left motor 1",  "left motor 2",  "left motor 3",
            "right motor 1", "right motor 2", "right motor 3"]
        self._current_position_controller=0
        self._current_position_keyboard=0        
        self.__motors=[]
        
        # Obtener los dispositivos de los motores
        for i in range(0,6):
            self.__motors.append(self.__robot.getDevice(self.__names[i]))
            self.__motors[i].setPosition(float('inf'))
            self.__motors[i].setVelocity(0)

        # Habilitar el teclado
        self.__keyboard = self.__robot.keyboard
        self.__keyboard.enable(self.__timestep)        
        
        # Obtener el dispositivo GPS y habilitarlo
        self.__gps= self.__robot.getDevice("gps")
        self.__gps.enable(self.__timestep)

        # Obtener el dispositivo Compass y habilitarlo
        self.__compass= self.__robot.getDevice("compass")
        self.__compass.enable(self.__timestep)

        self.__target_twist = Twist()

        # Obtener el dispositivo LIDAR y habilitarlo
        self.__lidar= self.__robot.getDevice("lidar")
        self.__lidar.enable(25)
        self.__lidar.enablePointCloud()

        # Obtener el dispositivo Accelerometer y habilitarlo
        self.__acc= self.__robot.getDevice("accelerometer")
        self.__acc.enable(self.__timestep)

        # Obtener el dispositivo Gyro y habilitarlo
        self.__gyro= self.__robot.getDevice("gyro")
        self.__gyro.enable(self.__timestep)

        # Obtener el dispositivo Camera y habilitarlo
        self.__camera= self.__robot.getDevice("camera")
        self.__camera.enable(25)
        self.__camera_motor= self.__robot.getDevice("camera motor")
    
        # Inicializar el nodo ROS 2
        rclpy.init(args=None)
        self.mode=0
        
        # Crear el nodo y los publicadores/suscriptores ROS
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Joy,'joy', self.__joy_callback,10)
        
        self.__publisherGPS = self.__node.create_publisher(NavSatFix, 'gps_data', 10)
        self.__publisherCompass=self.__node.create_publisher(Imu, 'compass_data', 10)
        self.__publisherLidar= self.__node.create_publisher(LaserScan, 'scan', 10)
        self.__publisherMode= self.__node.create_publisher(Int32, 'integer_topic', 10)
        self.__publisherAcc = self.__node.create_publisher(Float32MultiArray, 'acceleration', 10)
        self.__publisherGyro = self.__node.create_publisher(Float32MultiArray, 'gyro', 10)       
        self.timer_ = self.__node.create_timer(0.04, self.publish_data)
        self.__node.get_logger().info("\nYou can drive this robot:\n  Press 'Q' to change to the keyboard mode \n Press 'R' to change to the controller mode \nSelect the 3D window and use WASD: \n Press N and M to move the camera\n Press 'Q' to return to the autopilot mode\n Press 'P' to get the robot position\n")
        
    def __cmd_vel_callback(self, twist):
        # Callback para recibir el mensaje de velocidad
        self.__target_twist=twist

    def __joy_callback(self, msg):
        # Callback para recibir el mensaje de joystick
        axes=[0.0,0.0,0.0]
        axes[0]= msg.axes[0]*MAX_SPEED
        axes[1] = msg.axes[1]*MAX_SPEED
        axes[2] = msg.axes[3]
      
        forward_speed = axes[1]
        turn_speed = axes[0]
        self.controller_left_speed = forward_speed - turn_speed
        self.controller_right_speed = forward_speed + turn_speed
        self._current_position_controller += (axes[2]*0.01)
        self.controller_camera= self._current_position_controller
       
    def publish_data(self):
        # Recoger los datos de los sensores 
        gps_position = self.__gps.getValues()
        compass_values=self.__compass.getValues()
        lidar_data=self.__lidar.getRangeImage()
        acc_data=self.__acc.getValues()
        gyro_data=self.__gyro.getValues()             

        msgGPS = NavSatFix()
        msgGPS.longitude= gps_position[0]
        msgGPS.latitude= gps_position[1]
        msgGPS.altitude = gps_position[2]     

        msgCompass=Imu()
        msgCompass.header.frame_id = 'compass'
        msgCompass.orientation.x = compass_values[0]
        msgCompass.orientation.y = compass_values[1]
        msgCompass.orientation.z = compass_values[2]

        msgLidar = LaserScan()
        msgLidar.header.frame_id = 'lidar'        
        msgLidar.ranges = lidar_data
        
        msgMode= Int32()
        msgMode.data=self.mode

        msgAcc=Float32MultiArray()
        msgAcc.data=acc_data

        msgGyro=Float32MultiArray()
        msgGyro.data=gyro_data
        
        self.__publisherCompass.publish(msgCompass)
        self.__publisherGPS.publish(msgGPS)
        self.__publisherLidar.publish(msgLidar)
        self.__publisherMode.publish(msgMode)
        self.__publisherAcc.publish(msgAcc)
        self.__publisherGyro.publish(msgGyro)
       
    def check_keyboard(self):
        # Verificar el teclado para controlar el robot
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
            elif key == ord('A'):
                speeds[LEFT] = -MAX_SPEED
                speeds[RIGHT] = MAX_SPEED   
            elif key == ord('M'):
                self._current_position_keyboard -= MOTOR_POSITION_INCREMENT            
            elif key == ord('N'):
                self._current_position_keyboard += MOTOR_POSITION_INCREMENT             
            elif key == ord('P'):
                if key!= old_key:
                    gps_position= self.__gps.getValues()
                    self.__node.get_logger().info(f'position: {{{gps_position[X]}, {gps_position[Y]}}}')                    
            elif key == ord('Q'):
                if key!= old_key:
                    if self.mode != 1:
                        self.mode = 1
                        self.__node.get_logger().info('mode changed to keyboard')
                        self._current_position_keyboard=self._current_position_controller
                    elif self.mode==1:
                        self.mode = 0
                        self.__node.get_logger().info('mode changed to autopilot')  
            elif key == ord('L'):
                if key!= old_key:
                    if self.mode != 2:
                        self.mode = 2
                        self.__node.get_logger().info('mode autopilot changed to lidar')
                        
                    elif self.mode==2:
                        self.mode = 0
                        self.__node.get_logger().info('mode returned to autopilot')  
            elif key == ord('R'):
                if key!= old_key:
                    if self.mode != 3:
                        self.mode = 3
                        self.__node.get_logger().info('mode autopilot changed to controller')
                        self._current_position_controller=self._current_position_keyboard
                    elif self.mode==3:
                        self.mode = 0
                        self.__node.get_logger().info('mode returned to autopilot')     
        old_key=key        
        if self.mode==1:
            self.robot_set_speed(speeds[LEFT], speeds[RIGHT])
            self.move_camera(self._current_position_keyboard)   

    def move_camera(self,position):        
        # Mover la cámara a una posición específica
        self.__camera_motor.setPosition(position)  

    def work_autopilot(self):
        # Modo de trabajo del piloto automático
        forward_speedR = self.__target_twist.linear.y
        forward_speedL = self.__target_twist.linear.x
        self.robot_set_speed(forward_speedL,forward_speedR)         

    def robot_set_speed(self, left, right):
        # Establecer la velocidad de los motores del robot
        for i in range(3):
            self.__motors[i].setVelocity(left)
            self.__motors[i + 3].setVelocity(right)  
        
    def check_controller(self):
        # Verificar el controlador para controlar el robot
        self.robot_set_speed(self.controller_left_speed,self.controller_right_speed) 
        self.move_camera(self.controller_camera)
        
    def step(self):
        # Método principal que se ejecuta en cada paso de tiempo
        rclpy.spin_once(self.__node, timeout_sec=0)       
        
        if(self.mode==0 or 2):
            self.work_autopilot()
        if(self.mode==3):
           self.check_controller()
        
        self.check_keyboard()
       

                

