# Importa las bibliotecas necesarias
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float32MultiArray

# Define el tamaño de los puntos objetivo y otras constantes.
TARGET_POINTS_SIZE = 13
DISTANCE_TOLERANCE = 2.0
MAX_SPEED = 7.0
TURN_COEFFICIENT = 4.0
OBSTACLE_THRESHOLD = 1.0
SLOPE_THRESHOLD = 0.2
EVADE_FRAMES = 60
SENSOR_THRESHOLD = 900
SPEED_FACTOR=0.025

# Define una estructura de vector para almacenar las posiciones.
class Vector:
    def __init__(self, u, v):
        self.u = u
        self.v = v

# Define las posiciones objetivo. 
# Cada una es una instancia de la clase Vector definida anteriormente.
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

class PruVel(Node):  # Define la clase del nodo ROS2.
    
    def __init__(self):  # Define el constructor de la clase.
        super().__init__('auto_alg')  # Llama al constructor de la clase base.

        # Crea publicadores y suscriptores para diferentes tipos de mensajes.
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.subscriptionGPS = self.create_subscription(NavSatFix,'gps_data',self.gps_data_callback,10)
        self.subscriptionCompass = self.create_subscription(Imu, 'compass_data', self.compass_data_callback, 10)
        self.subscriptionLidar = self.create_subscription(LaserScan,'scan', self.lidar_callback,10)
        self.subscriptionMode_ = self.create_subscription(Int32, 'integer_topic', self.mode_callback, 10)
        self.subscriptionAcc = self.create_subscription(Float32MultiArray, 'acceleration', self.acc_callback, 10)
        self.subscriptionGyro = self.create_subscription(Float32MultiArray, 'gyro', self.gyro_callback, 10)
       
        # Crea un temporizador para llamar a la función timer_callback cada 0.04 segundos.
        timer_period=0.04
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Inicializa las variables de los datos del sensor.
        self.mode=0
        self.position=NavSatFix()             
        self.compass=Imu()
        self.speed=Twist()
        self.lidar=LaserScan()      
        self.acc=0
        self.gyro=0
        self.evasion_count=0
        self.turn_direction = None  # Almacenará la dirección en la que el robot debe girar
        self.previous_position = None
        self.time_at_previous_position = None
        self.position_difference = Vector(0, 0) 
        self.last_check_time = self.get_clock().now().seconds_nanoseconds()[0]

    def timer_callback(self):  # Esta función se llama cada vez que el temporizador se activa.       
        if self.mode == 0:  # Si el modo es 0, ejecuta el piloto automático.           
            self.run_autopilot()            
        elif self.mode==2:  # Si el modo es 2, ejecuta el control de lidar.
            self.run_lidar()
        # Publica la velocidad calculada en el tema 'cmd_vel'.
        self.__publisher.publish(self.speed)

    def gps_data_callback(self, msg):  # Esta función se llama cada vez que se recibe un mensaje en el tema 'gps_data'.        
        self.position = msg  # Actualiza la posición del robot.

    def compass_data_callback(self, data):  # Se llama cuando se recibe un mensaje en el tema 'compass_data'.        
        self.compass = data  # Actualiza la orientación del robot.

    def lidar_callback(self, msg):  # Se llama cuando se recibe un mensaje en el tema 'scan'.
        self.lidar=msg  # Actualiza los datos del lidar.

    def mode_callback(self, message):  # Se llama cuando se recibe un mensaje en el tema 'integer_topic'.
        self.mode=message.data  # Actualiza el modo.

    def acc_callback(self, msg):  # Se llama cuando se recibe un mensaje en el tema 'acceleration'.
        self.acc=msg.data[0]  # Actualiza la aceleración.

    def gyro_callback(self, msg):  # Se llama cuando se recibe un mensaje en el tema 'gyro'.
        self.gyro=msg.data[1]  # Actualiza el giroscopio.

    def run_autopilot(self):  # Esta función implementa el piloto automático del robot.
        # Preparar el array de velocidades
        speeds = [0.0, 0.0]        
        M_PI=math.pi
        acc_data=self.acc
        gyro_data=self.gyro
                       
        # Calcular la posición 2D del robot y su orientación
        position = Vector(self.position.longitude, self.position.latitude)                
        # Calcular la dirección y la distancia al objetivo
        direction=Vector(0,0)
        global current_target_index
        minus(direction, (targets[current_target_index]), position)
        distance = norm(direction)
        normalize(direction)

        # Calcular el ángulo de error
        robot_angle = math.atan2(self.compass.orientation.x, self.compass.orientation.y)        
        target_angle = math.atan2(direction.v, direction.u)
        beta = modulus_double(target_angle - robot_angle, 2.0 * M_PI) - M_PI
    
        # Mover singularity*
        if (beta > 0):
            beta = M_PI - beta
        else:
            beta = -beta - M_PI                
    
        # Comprobar si se ha alcanzado una posición objetivo       
        if (distance < DISTANCE_TOLERANCE):
            index_char = "th"
            if (current_target_index == 0):
                index_char = "st"
            elif     (current_target_index == 1):
                index_char = "nd"
            elif (current_target_index == 2):
                index_char = "rd"
            self.get_logger().info(f"{current_target_index + 1}{index_char} target reached")        
            current_target_index+=1
            current_target_index %= TARGET_POINTS_SIZE

        # Mover el robot hacia el siguiente objetivo
        else:
            speeds[0] = MAX_SPEED - M_PI + TURN_COEFFICIENT * beta
            speeds[1] = MAX_SPEED - M_PI - TURN_COEFFICIENT * beta
         
            # Cálculo de la velocidad de las ruedas para seguir la trayectoria
            roll = acc_data
            pitch = gyro_data
            adjustment_factor = 0.5

            # Rangos máximos y mínimos para los datos del giroscopio y acelerómetro
            max_roll = 2.0  
            min_roll = -2.0
            max_pitch = 2.0
            min_pitch = -2.0

            # Normalizar roll y pitch a un rango de 0 a 1
            normalized_roll = (roll - min_roll) / (max_roll - min_roll)
            normalized_pitch = (pitch - min_pitch) / (max_pitch - min_pitch)

            # Calcular el factor de velocidad basado en la magnitud de roll y pitch
            speed_factor = 1 - (adjustment_factor * max(normalized_roll, normalized_pitch))   
            # Actualizar las velocidades
            if pitch > 0.15 or pitch < -0.15:
                speeds[0] *= speed_factor
                speeds[1] *= speed_factor           
                
            self.speed.linear.x = speeds[0]
            self.speed.linear.y = speeds[1]

    def run_lidar(self):  # Esta función implementa la lógica de control del lidar.
         # Preparar el array de velocidades
        speeds = [0.0, 0.0]
        M_PI = math.pi
        acc_data=self.acc
        gyro_data=self.gyro
       
        # Obtener los datos del LIDAR
        lidar_data = self.lidar.ranges
        obstacle_detected=False   
        middle_index = len(lidar_data) // 2        
        left_sector = lidar_data[:middle_index]
        right_sector = lidar_data[middle_index:]       
        left_obstacle_count = sum(1 for range in left_sector if range < OBSTACLE_THRESHOLD)
        right_obstacle_count = sum(1 for range in right_sector if range < OBSTACLE_THRESHOLD)  

            
         # Comprobar si hay un obstáculo
        
        if left_obstacle_count > len(left_sector) * 0.1 or right_obstacle_count > len(right_sector) * 0.1:
                obstacle_detected = True
                if self.evasion_count==0:                  
                    self.evasion_count=EVADE_FRAMES
        if self.turn_direction is None:           
            if left_obstacle_count < right_obstacle_count:
                self.turn_direction = "left"
            elif left_obstacle_count > right_obstacle_count:
                self.turn_direction = "right"
              
        # Reducir el contador de evasión, pero no permitir que sea negativo
        self.evasion_count = max(0, self.evasion_count - 1)
        if self.evasion_count == 0:
            self.turn_direction = None
        # Calcular la posición 2D del robot y su orientación
        position = Vector(self.position.longitude, self.position.latitude)

        # Calcular la dirección y la distancia al objetivo
        direction = Vector(0,0)
        global current_target_index
        minus(direction, (targets[current_target_index]), position)
        distance = norm(direction)
        normalize(direction)

        # Calcular el ángulo de error
        robot_angle = math.atan2(self.compass.orientation.x, self.compass.orientation.y)
        target_angle = math.atan2(direction.v, direction.u)
        beta = modulus_double(target_angle - robot_angle, 2.0 * M_PI) - M_PI

        # Mover singularity*
        if (beta > 0):
            beta = M_PI - beta
        else:
            beta = -beta - M_PI       

        # Comprobar si se ha alcanzado una posición objetivo
        if (distance < DISTANCE_TOLERANCE):
            index_char = "th"
            if (current_target_index == 0):
                index_char = "st"
            elif (current_target_index == 1):
                index_char = "nd"
            elif (current_target_index == 2):
                index_char = "rd"
            self.get_logger().info(f"{current_target_index + 1}{index_char} target reached")
            current_target_index += 1
            current_target_index %= TARGET_POINTS_SIZE

        # Mover el robot hacia el siguiente objetivo
        else:
            if obstacle_detected or self.evasion_count > 0:                
                    if self.turn_direction == "left":
                        speeds[0] = -MAX_SPEED*SPEED_FACTOR*self.evasion_count
                        speeds[1] = MAX_SPEED*SPEED_FACTOR*self.evasion_count
                    elif self.turn_direction =="right":
                        speeds[0] = MAX_SPEED*SPEED_FACTOR*self.evasion_count
                        speeds[1] = -MAX_SPEED*SPEED_FACTOR*self.evasion_count

            else:
                speeds[0] = MAX_SPEED - M_PI + TURN_COEFFICIENT * beta
                speeds[1] = MAX_SPEED - M_PI - TURN_COEFFICIENT * beta


       
            # Cálculo de la velocidad de las ruedas para seguir la trayectoria
            roll = acc_data
            pitch = gyro_data
            adjustment_factor = 0.5
            # Rangos máximos y mínimos para los datos del giroscopio y acelerómetro
            max_roll = 2.0  
            min_roll = -2.0
            max_pitch = 2.0
            min_pitch = -2.0

            # Normalizar roll y pitch a un rango de 0 a 1
            normalized_roll = (roll - min_roll) / (max_roll - min_roll)
            normalized_pitch = (pitch - min_pitch) / (max_pitch - min_pitch)

            # Calcular el factor de velocidad basado en la magnitud de roll y pitch
            speed_factor = 1 - (adjustment_factor * max(normalized_roll, normalized_pitch))   
            # Actualizar las velocidades
            if pitch > 0.15 or pitch < -0.15:
                speeds[0] *= speed_factor
                speeds[1] *= speed_factor      
            self.speed.linear.x = speeds[0]
            self.speed.linear.y = speeds[1]
            
def norm(v):  # Calcula la norma de un vector.
    return math.sqrt(v.u * v.u + v.v * v.v)

def normalize(v):  # Normaliza un vector.
    n = norm(v)
    v.u /= n
    v.v /= n

def minus(v, v1, v2):  # Resta dos vectores.
    v.u = v1.u - v2.u
    v.v = v1.v - v2.v

def modulus_double(a, m):  # Calcula el módulo de un número.
    div = int(a / m)
    r = a - div * m
    if r < 0.0:
        r += m
    return r

def main(args=None):  # Función principal.
    rclpy.init(args=args)  # Inicializa ROS2.
    pruVel = PruVel()  # Crea una instancia de la clase PruVel.
    rclpy.spin(pruVel)  # Entra en el ciclo de procesamiento de ROS2.
    pruVel.destroy_node()  # Destruye el nodo cuando se termina el ciclo.
    rclpy.shutdown()  # Apaga ROS2.

# Comprueba si este script se está ejecutando directamente. Si es así, llama a la función principal.
if __name__ ==    '__main__':
        main()  # Llama a la función main si este archivo se está ejecutando directamente.

          
        
