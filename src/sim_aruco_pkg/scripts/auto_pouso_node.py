import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from aruco_ros.msg import MarkerArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time

class ArucoLandController(Node):
    def __init__(self):
        super().__init__('aruco_land_controller')

        # QoS Profile para dados de sensor
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Subscribers ---
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile)
        # Assine o tópico do ArUco da câmera inferior
        self.aruco_sub = self.create_subscription(MarkerArray, '/aruco/markers', self.aruco_cb, 10)

        # --- Publishers ---
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # --- Service Clients ---
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # --- Variáveis de Estado ---
        self.current_state = State()
        self.target_seen = False
        self.target_pose = None
        self.drone_state = 'STARTUP' # STARTUP, TAKEOFF, SEARCHING, CENTERING, LANDING

        # PID gains (precisam de ajuste)
        self.kp = 0.5

        self.timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info('Controlador de pouso ArUco iniciado.')

    def state_cb(self, msg):
        self.current_state = msg

    def aruco_cb(self, msg):
        if len(msg.markers) > 0:
            self.target_seen = True
            # Pega a pose do primeiro marcador detectado
            # NOTA: A pose é relativa à câmera!
            self.target_pose = msg.markers[0].pose.pose
        else:
            self.target_seen = False

    def set_mode(self, mode):
        # ... (implementação similar à resposta anterior) ...

    def arm(self):
        # ... (implementação similar à resposta anterior) ...

    def takeoff(self):
        # Para decolar em modo GUIDED, é mais fácil usar setpoint_position
        # Este exemplo focará na centralização
        self.get_logger().info('Decolagem manual necessária para este exemplo. Mude para GUIDED e decole.')
        self.drone_state = 'SEARCHING' # Assumimos que decolou

    def main_loop(self):
        if self.drone_state == 'SEARCHING':
            if self.target_seen:
                self.get_logger().info('Alvo encontrado! Mudando para modo de centralização.')
                self.drone_state = 'CENTERING'
            else:
                # Lógica de busca: voar em um padrão
                # Por simplicidade, vamos apenas pairar
                pass

        elif self.drone_state == 'CENTERING':
            if not self.target_seen:
                self.get_logger().warn('Alvo perdido! Voltando para busca.')
                self.drone_state = 'SEARCHING'
                return

            # Lógica do PID
            error_x = self.target_pose.position.y # Erro no eixo X do drone é a posição Y do marcador na câmera
            error_y = self.target_pose.position.x # Erro no eixo Y do drone é a posição X do marcador na câmera

            # A câmera inferior está rotacionada, então os eixos podem precisar de ajuste
            # Este é um exemplo simplificado

            cmd_vel = TwistStamped()
            cmd_vel.header.stamp = self.get_clock().now().to_msg()
            cmd_vel.twist.linear.x = self.kp * error_x
            cmd_vel.twist.linear.y = self.kp * error_y
            cmd_vel.twist.linear.z = 0.0 # Mantenha a altitude por enquanto

            # Se o erro for pequeno, comece a descer
            if abs(error_x) < 0.1 and abs(error_y) < 0.1:
                self.get_logger().info('Alinhado! Começando a descida.')
                cmd_vel.twist.linear.z = -0.2 # Desce
                # Checar a altitude para pouso final
                if self.target_pose.position.z < 0.5: # Se o marcador está a menos de 50cm
                    self.drone_state = 'LANDING'
            
            self.vel_pub.publish(cmd_vel)

        elif self.drone_state == 'LANDING':
            self.get_logger().info('Comandando pouso final.')
            self.set_mode('LAND')
            self.timer.cancel() # Para o loop


def main(args=None):
    rclpy.init(args=args)
    controller = ArucoLandController()
    # Para este exemplo, o controle começa após a decolagem manual
    # Em um script completo, você automatizaria a decolagem
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()