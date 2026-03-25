#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import sys
import os
import glob
import time

def evaluate_polynomial(coeffs, t):
    """
    Valuta un polinomio P(t) = c0 + c1*t + c2*t^2 + ...
    I coefficienti devono essere in ordine di potenza crescente [c0, c1, c2...]
    """
    return sum(c * (t ** i) for i, c in enumerate(coeffs))

class TrajectoryVisualizer(Node):
    def __init__(self, target_dir, frame_id="world"):
        super().__init__('trajectory_visualizer')
        
        self.target_dir = target_dir
        #"/root/ros2_ws/src/crazyflie_examples/crazyflie_examples/data"
        self.frame_id = frame_id
        
        # Dizionario per tracciare i file e i relativi publisher
        # Struttura: { 'percorso_completo': { 'publisher': pub, 'path_msg': msg, 'mtime': last_modified } }
        self.visualizations = {} 

        self.get_logger().info(f"Avvio monitoraggio cartella: {self.target_dir}")
        if not os.path.exists(self.target_dir):
            self.get_logger().warn(f"Attenzione: La cartella {self.target_dir} non esiste ancora. Attendo creazione...")

        # Timer a 1Hz: controlla nuovi file E pubblica i messaggi
        self.timer = self.create_timer(1.0, self.timer_callback)

    def load_path_msg(self, csv_file):
        """Legge un CSV, calcola i punti e restituisce un messaggio nav_msgs/Path."""
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        
        try:
            # Controllo base per evitare di leggere file vuoti mentre vengono creati
            if os.path.getsize(csv_file) == 0:
                return None

            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                fieldnames = reader.fieldnames
                
                # Verifica che sia un CSV valido (deve avere la colonna Duration)
                if not fieldnames or 'Duration' not in fieldnames:
                    return None

                for row in reader:
                    duration = float(row['Duration'])
                    
                    # Estrai i coefficienti per x, y, z (da potenza 0 a 7)
                    coeffs_x = [float(row[f'x^{i}']) for i in range(8)]
                    coeffs_y = [float(row[f'y^{i}']) for i in range(8)]
                    coeffs_z = [float(row[f'z^{i}']) for i in range(8)]
                    
                    # Campionamento della curva
                    dt = 0.05 # Risoluzione temporale (50ms)
                    t = 0.0
                    while t <= duration:
                        x = evaluate_polynomial(coeffs_x, t)
                        y = evaluate_polynomial(coeffs_y, t)
                        z = evaluate_polynomial(coeffs_z, t)
                        
                        pose = PoseStamped()
                        pose.header.frame_id = self.frame_id
                        pose.pose.position.x = x
                        pose.pose.position.y = y
                        pose.pose.position.z = z
                        pose.pose.orientation.w = 1.0 # Orientamento neutro
                        
                        path_msg.poses.append(pose)
                        t += dt
            return path_msg

        except Exception as e:
            # Può capitare se il file è bloccato in scrittura
            self.get_logger().warn(f"Impossibile leggere il file {os.path.basename(csv_file)}: {e}")
            return None

    def check_for_updates(self):
        """Scansiona la cartella per file nuovi o modificati."""
        if not os.path.exists(self.target_dir):
            return

        # Cerca tutti i file .csv nella cartella target
        found_files = glob.glob(os.path.join(self.target_dir, "*.csv"))

        for file_path in found_files:
            try:
                current_mtime = os.path.getmtime(file_path)
            except OSError:
                continue # File cancellato o inaccessibile

            # Determina se è un file nuovo o modificato
            is_new = file_path not in self.visualizations
            is_modified = not is_new and current_mtime > self.visualizations[file_path]['mtime']

            if is_new or is_modified:
                new_path_msg = self.load_path_msg(file_path)
                
                if new_path_msg:
                    if is_new:
                        # Logica per file NUOVO
                        filename_clean = os.path.splitext(os.path.basename(file_path))[0]
                        # Sanifica il nome per usarlo come topic ROS (niente spazi o punti)
                        filename_clean = filename_clean.replace(".", "_").replace(" ", "_")
                        
                        topic_name = f"/trajectory_path/{filename_clean}"
                        pub = self.create_publisher(Path, topic_name, 10)
                        
                        self.visualizations[file_path] = {
                            'publisher': pub,
                            'path_msg': new_path_msg,
                            'mtime': current_mtime,
                            'topic': topic_name
                        }
                        self.get_logger().info(f"[NUOVO] Trovato {os.path.basename(file_path)} -> Topic: {topic_name}")
                    else:
                        # Logica per file AGGIORNATO
                        self.visualizations[file_path]['path_msg'] = new_path_msg
                        self.visualizations[file_path]['mtime'] = current_mtime
                        self.get_logger().info(f"[AGGIORNATO] Ricaricato {os.path.basename(file_path)}")

    def timer_callback(self):
        # 1. Controlla il filesystem
        self.check_for_updates()

        # 2. Pubblica i messaggi correnti
        current_time = self.get_clock().now().to_msg()
        
        for file_path, item in self.visualizations.items():
            msg = item['path_msg']
            if msg:
                msg.header.stamp = current_time
                item['publisher'].publish(msg)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Uso: python3 visualize_trajectory.py <percorso_cartella_csv>")
        return
    
    target_dir = sys.argv[1]
    
    node = None
    try:
        node = TrajectoryVisualizer(target_dir)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Errore critico: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()