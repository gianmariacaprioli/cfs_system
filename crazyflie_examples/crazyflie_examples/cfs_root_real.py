#!/usr/bin/env python
import numpy as np, rclpy, tf_transformations, os, time, signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped, PoseStamped
from crazyflie_interfaces.msg import LogDataGeneric
from crazyflie_py import Crazyswarm
from pathlib import Path
from functools import partial

class DirectCmdVelListener(Node):
    def __init__(self, cfs):
        super().__init__('direct_cmd_vel_listener')
        self.start_t = time.time()
        self.is_landing = False # <--- FLAG per fermare i log e i comandi
        
        # Dizionario per memorizzare gli oggetti Crazyflie
        self.cfs = {cf.prefix: cf for cf in cfs}
        
        # Inizializzazione dinamica delle memorie
        self.logs = {}
        self.poses = {}
        self.yaws = {}
        self.statuses = {}
        self.subs = [] 
        
        for prefix in self.cfs.keys():
            clean_prefix = prefix.lstrip('/')
            
            self.logs[prefix] = {'odom': [], 'ctrl': [], 'status': []}
            self.poses[prefix] = np.zeros(3)
            self.yaws[prefix] = 0.0
            self.statuses[prefix] = {'m1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'vbat': 0.0}
            
            # 1. Comandi di Velocità
            sub_vel = self.create_subscription(
                TwistStamped, f'/{clean_prefix}/cmd_vel', 
                partial(self.direct_cb, prefix=prefix), qos_profile_sensor_data)
            
            # 2. Posizione
            sub_pose = self.create_subscription(
                PoseStamped, f'/{clean_prefix}/pose', 
                partial(self.cb_pose, prefix=prefix), 10)
            
            # 3. Status Hardware
            sub_status = self.create_subscription(
                LogDataGeneric, f'/{clean_prefix}/drone_status', 
                partial(self.cb_status, prefix=prefix), 10)
                
            self.subs.extend([sub_vel, sub_pose, sub_status])

    def cb_pose(self, msg, prefix):
        self.poses[prefix] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        )
        self.yaws[prefix] = yaw

    def cb_status(self, msg, prefix):
        if len(msg.values) >= 5:
            self.statuses[prefix]['m1'] = msg.values[0]
            self.statuses[prefix]['m2'] = msg.values[1]
            self.statuses[prefix]['m3'] = msg.values[2]
            self.statuses[prefix]['m4'] = msg.values[3]
            self.statuses[prefix]['vbat'] = msg.values[4]

    def direct_cb(self, msg, prefix):
        # Se abbiamo avviato l'atterraggio, IGNORA i comandi esterni e NON loggare più
        if self.is_landing:
            return 

        v = np.array([msg._twist._linear.x, msg._twist._linear.y, msg._twist._linear.z])
        
        self.cfs[prefix].cmdVelocityWorld(v, 0.0) 
        
        t = time.time() - self.start_t
        self.logs[prefix]['ctrl'].append([t, *v, 0.0])
        self.logs[prefix]['odom'].append([t, *self.poses[prefix], self.yaws[prefix]])
        
        s = self.statuses[prefix]
        self.logs[prefix]['status'].append([t, s['m1'], s['m2'], s['m3'], s['m4'], s['vbat']])

def main():
    swarm = Crazyswarm()
    all_cfs = swarm.allcfs.crazyflies
    
    if len(all_cfs) == 0:
        print("Errore: Nessun drone trovato nel file YAML.")
        return
        
    print(f"Rilevati {len(all_cfs)} droni pronti al volo.")
    node = DirectCmdVelListener(all_cfs)
    
    # ==========================================
    # TRUCCO: Intercettiamo il Ctrl+C PRIMA di ROS2
    # ==========================================
    def sigint_handler(sig, frame):
        if not node.is_landing:
            print("\n[INFO] Arresto manuale intercettato (Ctrl+C)! Inizio atterraggio morbido...")
            node.is_landing = True
            
    # Associa la nostra funzione personalizzata al segnale di interruzione
    signal.signal(signal.SIGINT, sigint_handler)
    
    print("Ascolto Diretto in corso... Premere Ctrl+C per fermare, atterrare e salvare i log.")
    
    try:
        # FASE 1: VOLO NORMALE (Gira finché non premiamo Ctrl+C)
        while rclpy.ok() and not node.is_landing:
            rclpy.spin_once(node, timeout_sec=0.1)
            
        # FASE 2: ATTERRAGGIO MORBIDO (Parte solo se node.is_landing è diventato True)
        if node.is_landing and rclpy.ok():
            z_threshold = 0.05       
            landing_speed = -0.10    
            max_landing_time = 10.0  
            start_landing_t = time.time()

            while rclpy.ok() and (time.time() - start_landing_t) < max_landing_time:
                all_landed = True 
                
                for prefix, cf in node.cfs.items():
                    current_z = node.poses[prefix][2]
                    
                    if current_z > z_threshold:
                        all_landed = False 
                        try:
                            cf.cmdVelocityWorld(np.array([0.0, 0.0, landing_speed]), 0.0)
                        except Exception:
                            pass
                    else:
                        try:
                            cf.cmdVelocityWorld(np.zeros(3), 0.0)
                        except Exception:
                            pass
                
                if all_landed:
                    print("[INFO] Atterraggio completato per tutti i droni.")
                    break
                
                rclpy.spin_once(node, timeout_sec=0.1)

    finally:
        # FASE 3: SPEGNIMENTO SICURO E SALVATAGGIO
        for prefix, cf in node.cfs.items():
            try:
                cf.cmdVelocityWorld(np.zeros(3), 0.0)
            except Exception:
                pass
                
        print("[INFO] Salvataggio CSV in corso...")
        d = Path.home() / 'cfs_ws/cfs_real/traj_logs'
        os.makedirs(d, exist_ok=True)
        for prefix, data in node.logs.items():
            clean_name = prefix.lstrip('/')
            if data['odom']:
                np.savetxt(d / f'{clean_name}_direct_odom.csv', data['odom'], delimiter=',', header='t,x,y,z,yaw', comments='')
            if data['ctrl']:
                np.savetxt(d / f'{clean_name}_direct_ctrl.csv', data['ctrl'], delimiter=',', header='t,vx,vy,vz,omega', comments='')
            if data['status']:
                np.savetxt(d / f'{clean_name}_direct_status.csv', data['status'], delimiter=',', header='t,m1,m2,m3,m4,vbat', comments='')
        
        print("[INFO] Dati salvati con successo. Uscita.")
        node.destroy_node()

if __name__ == '__main__': 
    main()