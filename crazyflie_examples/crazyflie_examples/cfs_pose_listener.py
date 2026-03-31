import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from crazyflie_interfaces.srv import GoTo # Importiamo il servizio GoTo
from crazyflie_interfaces.srv import Stop
import math

class MultiDroneController(Node):

    def __init__(self):
        super().__init__('cf_multi_controller')
        
        self.drone_states = {}
        
        # Gestiamo lo stato di emergenza per evitare di inviare 100 comandi al secondo
        # Key: int (id), Value: bool (True se è già in stato di stop)
        self.emergency_flags = {}
        
        # Client per inviare il comando di Hover (GoTo relativo)
        self.hover_clients = {}
        
        self.active_subscriptions = {}
        self.max_cf_id = 9
        
        # Timer discovery (ogni 2s)
        self.discovery_timer = self.create_timer(2.0, self.discovery_callback)
        
        # Timer controllo (30Hz)
        self.control_timer = self.create_timer(1.0/30.0, self.control_loop)
        
        self.get_logger().info('Controller avviato. In attesa di droni...')

    def discovery_callback(self):
        topic_names_and_types = self.get_topic_names_and_types()
        active_topic_names = [t[0] for t in topic_names_and_types]

        for i in range(self.max_cf_id + 1):
            pose_topic = f'/cf_{i}/pose'
            # Usiamo il servizio GoTo invece di Stop o Twist
            goto_service_name = f'/cf_{i}/go_to'
            drone_name = f'cf_{i}'
            
            if pose_topic in active_topic_names and i not in self.active_subscriptions:
                
                # A. Subscriber Posizione
                self.create_dynamic_subscription(i, pose_topic)
                
                # B. Client per Hover (GoTo)
                cli = self.create_client(GoTo, goto_service_name)
                self.hover_clients[i] = cli
                
                # Inizializziamo il flag di emergenza a False
                self.emergency_flags[i] = False
                
                self.get_logger().info(f'{drone_name} agganciato. Client GoTo pronto.')
                self.active_subscriptions[i] = True

    def create_dynamic_subscription(self, cf_id, topic_name):
        self.create_subscription(
            PoseStamped,
            topic_name,
            lambda msg, _id=cf_id: self.pose_callback(msg, _id),
            10
        )

    def pose_callback(self, msg, cf_id):
        self.drone_states[cf_id] = msg.pose

    def control_loop(self):
        MIN_SAFE_DISTANCE = 0.40  # 30 cm soglia di collisione
        
        for my_id, my_pose in self.drone_states.items():
            
            # Verifichiamo se abbiamo il client pronto
            client = self.hover_clients.get(my_id)
            if not client: continue

            collision_detected = False
            
            # --- CONTROLLO COLLISIONI ---
            for other_id, other_pose in self.drone_states.items():
                if my_id == other_id: continue
                
                dx = my_pose.position.x - other_pose.position.x
                dy = my_pose.position.y - other_pose.position.y
                dz = my_pose.position.z - other_pose.position.z
                
                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                
                if distance < MIN_SAFE_DISTANCE:
                    collision_detected = True
                    # Logghiamo solo se è un nuovo rilevamento per non intasare la console
                    if not self.emergency_flags.get(my_id, False):
                        self.get_logger().warning(
                            f'PERICOLO: {my_id} <-> {other_id} ({distance:.2f}m). STOP!',
                        )
                    break

            # --- ESECUZIONE AZIONE (HOVER) ---
            
            # Caso 1: Rilevata collisione e NON abbiamo ancora mandato lo stop
            if collision_detected and not self.emergency_flags.get(my_id, False):
                
                if client.service_is_ready():
                    # Creiamo la richiesta GoTo
                    req = GoTo.Request()
                    req.group_mask = 0
                    req.relative = True  # IMPORTANTE: Relativo alla posizione corrente
                    req.goal = Point(x=0.0, y=0.0, z=0.0) # 0 metri di spostamento = stai fermo
                    req.yaw = 0.0 # Mantieni yaw corrente (o relativo 0)
                    
                    # Durata breve per rendere l'arresto immediato
                    req.duration = rclpy.duration.Duration(seconds=0.5).to_msg()
                    
                    # Invia comando
                    client.call_async(req)
                    
                    # Segna che questo drone è stato fermato
                    self.emergency_flags[my_id] = True
            
            # Caso 2: Pericolo finito (opzionale)
            # Se vuoi che il drone possa riprendere a muoversi se si allontanano,
            # devi resettare il flag. Nota: Il drone resterà in hover finché non riceve
            # un altro comando da un altro script.
            elif not collision_detected and self.emergency_flags.get(my_id, False):
                self.get_logger().info(f'Pericolo rientrato per cf_{my_id}. Flag resettato.', once=True)
                self.emergency_flags[my_id] = False

def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()