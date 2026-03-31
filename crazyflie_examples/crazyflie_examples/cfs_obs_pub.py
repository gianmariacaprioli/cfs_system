import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

class DynamicObstaclePublisher(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_publisher')
        
        # Dizionario per salvare i publisher creati al volo
        # Formato: {'/obs_1/pose': oggetto_publisher}
        self.publishers_dict = {}
        
        # Iscrizione al topic /tf (dove passano tutti i frame 3D)
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            100
        )
        
        self.get_logger().info("Nodo avviato. In attesa di frame chiamati 'obs_...' su /tf")

    def tf_callback(self, msg):
        for transform in msg.transforms:
            child_frame = transform.child_frame_id
            
            # Controlliamo se è un ostacolo (es. 'obs_1', 'obs_2_mocap')
            if 'obs_' in child_frame:
                
                # Puliamo il nome per avere un topic elegante (rimuoviamo eventuali suffissi di mocap)
                clean_name = child_frame.replace('_mocap', '')
                topic_name = f'/{clean_name}/pose'
                
                # 1. CREAZIONE DINAMICA: Se il topic non esiste, creiamo il publisher
                if topic_name not in self.publishers_dict:
                    self.get_logger().info(f"Nuovo ostacolo rilevato: {clean_name}. Creo topic: {topic_name}")
                    self.publishers_dict[topic_name] = self.create_publisher(PoseStamped, topic_name, 100)
                
                # 2. ASSEMBLAGGIO MESSAGGIO: Convertiamo il Transform in PoseStamped
                pose_msg = PoseStamped()
                pose_msg.header = transform.header
                
                # (Opzionale) Forza il frame di riferimento a 'world' se Motive usa nomi strani
                # pose_msg.header.frame_id = 'world' 
                
                pose_msg.pose.position.x = transform.transform.translation.x
                pose_msg.pose.position.y = transform.transform.translation.y
                pose_msg.pose.position.z = transform.transform.translation.z
                #pose_msg.pose.orientation = transform.transform.rotation
                
                # 3. PUBBLICAZIONE
                self.publishers_dict[topic_name].publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstaclePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()