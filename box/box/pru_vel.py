import rclpy
import math
import random
from rclpy.node import Node
from sensor_msgs.msg import Imu,NavSatFix,LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32,Float32MultiArray



# Define the list of target points
TARGET_POINTS_SIZE = 13
DISTANCE_TOLERANCE = 1.5
MAX_SPEED = 7.0
TURN_COEFFICIENT = 4.0

# Define a vector structure
class Vector:
    def __init__(self, u, v):
        self.u = u
        self.v = v

# Configuración de la trayectoria aleatoria
change_direction_time = 8  # segundos
change_direction_counter = change_direction_time
random_angle = 0

# Define the target positions
targets = [
    Vector(-4.209318, 9.147717),
    Vector(0.946812, 9.404304),
    Vector(0.175989, -1.784311),
    Vector(-2.805353, -8.829694),
    Vector(-3.846730, -15.602851),
    Vector(-4.394915, -24.550777),
    Vector(-1.701877, -33.617226),
    Vector(-4.394915, -24.550777),
    Vector(-3.846730, -15.602851),
    Vector(-2.805353, -8.829694),
    Vector(0.175989, -1.784311),
    Vector(0.946812, 9.404304),
    Vector(-7.930821, 6.421292)
]

current_target_index = 0
# Enumerate component indices
X, Y, Z, ALPHA = range(4)

class PruVel(Node):
    
    def __init__(self):
        super().__init__('pru_vel')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.subscriptionGPS=self.create_subscription(NavSatFix,'gps_data',self.gps_data_callback,10)
        self.subscriptionCompass = self.create_subscription(Imu, 'compass_data', self.compass_data_callback, 10)
        self.subscriptionLidar = self.create_subscription(LaserScan,'scan', self.lidar_callback,10)
        self.subscriptionMode_ = self.create_subscription(Int32, 'integer_topic', self.mode_callback, 10)
        self.subscriptionAcc = self.create_subscription(Float32MultiArray, 'acceleration', self.acc_callback, 10)
        self.subscriptionGyro = self.create_subscription(Float32MultiArray, 'gyro', self.gyro_callback, 10)
        timer_period=5.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.mode=0
        self.position=NavSatFix()             
        self.compass=Imu()
        self.speed=Twist()
     
        
    def timer_callback(self):
        
        self.get_logger().info(f'modo: {self.mode}')
        if self.mode == 0:            
            self.run_autopilot()
            
        elif self.mode==2:
            self.run_lidar()

        self.get_logger().info(f'velsmod0: {self.speed.linear.x }, {self.speed.linear.y}')  
        self.__publisher.publish(self.speed)
        
        #print('Sended command velocity message:', command_message)
           
    def gps_data_callback(self, msg):
        # Obtener la posición y orientación del mensaje PoseStamped
        print("Function called")
        self.position = msg
        print('GPS data received:')
        print('Latitude:', msg.latitude)
        print('Longitude:', msg.longitude)
        print('Altitude:', msg.altitude)
                 
    def compass_data_callback(self, data):
        # Obtener la posición y orientación del mensaje PoseStamped
        print("Function called")
        self.compass = data
        print(f'Compass orientation: ({data.orientation.w}, {data.orientation.x}, {data.orientation.y}, {data.orientation.z})')

    def mode_callback(self, message):
        self.mode=message.data

    def acc_callback(self, msg):
        self.acc=msg.data[0]
        
    def gyro_callback(self, msg):
        self.gyro=msg.data[1]
        
    def run_autopilot(self):
        #prepare the speed array
        speeds = [0.0, 0.0]

        M_PI=math.pi
         #read gps position and compass values
        
        #compute the 2D position of the robot and its orientation
        position = Vector(self.position.latitude, self.position.longitude)

        #compute the direction and the distance to the target
        direction=Vector(0,0)
        global current_target_index
        minus(direction, (targets[current_target_index]), position)
        distance = norm(direction)
        normalize(direction)

        #compute the error angle
        robot_angle = math.atan2(self.compass.orientation.x, self.compass.orientation.y)
        target_angle = math.atan2(direction.v, direction.u)
        beta = modulus_double(target_angle - robot_angle, 2.0 * M_PI) - M_PI

        # move singularity*
        if (beta > 0):
            beta = M_PI - beta
        else:
            beta = -beta - M_PI

        #a target position has been reached
        if (distance < DISTANCE_TOLERANCE):
        
            index_char = "th"
            if (current_target_index == 0):
                index_char = "st"
            elif (current_target_index == 1):
                index_char = "nd"
            elif (current_target_index == 2):
                index_char = "rd"
            print(f"{current_target_index + 1}{index_char} target reached")        
            current_target_index+=1
            current_target_index %= TARGET_POINTS_SIZE
  
        #move the robot to the next target
        else:
            speeds[0] = MAX_SPEED - M_PI + TURN_COEFFICIENT * beta
            speeds[1] = MAX_SPEED - M_PI - TURN_COEFFICIENT * beta
  

        #set the motor speeds
        self.speed= Twist()
        self.speed.linear.x = speeds[0]
        self.speed.linear.y = speeds[1]
        
    def run_lidar(self):
        lidar_data = self.lidar.ranges        
        lidar_size = int(self.lidar.angle_increment)
        lidar_max=self.lidar.range_min
        lidar_min=self.lidar.range_max
        acc_data=self.acc
        gyro_data=self.gyro
        global change_direction_counter
        global change_direction_time
        global random_angle
        obstacle_detected = False
        left_speed= MAX_SPEED
        right_speed=MAX_SPEED

        # Detección de obstáculos
        for i in range(0, lidar_size):
            if lidar_data[i] < lidar_max and lidar_data[i] > lidar_min:
                obstacle_detected = True
                break

        # Si hay un obstáculo, girar para evitarlo
        if obstacle_detected:
            left_speed= MAX_SPEED
            left_speed=-MAX_SPEED
        # Si no hay obstáculos, avanzar recto o seguir la trayectoria aleatoria
        else:
            # Cambio de dirección aleatorio 
            
            random_angle = random.uniform(-math.pi/2, math.pi/2)  
            left_speed=(left_speed * (1 - random_angle))
            right_speed=(right_speed * (1 + random_angle))

            # Cálculo de la velocidad de las ruedas para seguir la trayectoria
            roll = acc_data
            pitch = gyro_data
            if abs(roll) > 0.5 or abs(pitch) > 0.5:
                left_speed=(0.5 * left_speed)
                right_speed=(0.5 * right_speed)
            
        self.speed.linear.x = left_speed
        self.speed.linear.y = right_speed

def norm(v):
        return math.sqrt(v.u * v.u + v.v * v.v)

def normalize(v):
        n = norm(v)
        v.u /= n
        v.v /= n


def minus(v, v1, v2):
        v.u = v1.u - v2.u
        v.v = v1.v - v2.v

def modulus_double(a, m):
        div = int(a / m)
        r = a - div * m
        if r < 0.0:
            r += m
        return r
        
    
def main(args=None):
    rclpy.init(args=args)
    pruVel = PruVel()
    rclpy.spin(pruVel)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pruVel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()