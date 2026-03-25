#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys
import os
import subprocess
import glob # Utile per trovare/cancellare file specifici

# Import librerie custom
from .vrp_solver import solve_vrp 
from .trajectory_client import generate_trajectory

class TrajectorySelector(Node):

    def __init__(self):
        super().__init__('trajectory_selector')
        self.publisher_ = self.create_publisher(PoseStamped, '/cf_0/cmd_trajectory', 10)
        self.get_logger().info("Nodo CfsManager avviato.")

        # Cartella dove il solver salva i risultati
        self.output_data_dir = "/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data"

        # Configurazione Menu
        self.traj_options = {
            "1": ("Scenario 1 (Depot 0,0)", "coords.csv", "demands_case1.csv"),
            "2": ("Scenario 2 (Depot 2,0)", "coords.csv", "demands_case2.csv"),
            "3": ("Scenario 3 (Depot 1,-2)", "coords.csv", "demands_case3.csv"),
        }
        
        self.show_menu_and_execute()

    def show_menu_and_execute(self):
        while rclpy.ok():
            print("\n=== Seleziona Scenario VRP ===")
            for k, (name, _, _) in self.traj_options.items():
                print(f"[{k}] {name}")
            print("[0] Esci")
            
            choice = input("Scelta scenario: ")
            
            if choice == "0" or choice.lower() == "q":
                print("Uscita.")
                break

            if choice not in self.traj_options:
                print("Scelta non valida.")
                continue

            try:
                print("\Insert Drone's Capacity (Q): ")
                q_input = input("Capacity (default 10): ")
                q = int(q_input) if q_input else 10

                print("Insert Drone's Number (N): ")
                n_input = input("Number (max 4): ")
                n = int(n_input) if n_input else 4
            except ValueError:
                print("Errore: Inserire numeri validi.")
                continue

            # 1. Recupera i nomi dei file
            description, coords_file, demands_file = self.traj_options[choice]
            self.get_logger().info(f"execute: {description} with Q={q}, N={n}")

            # 2. Chiama il solver
            try:
                solve_vrp(coords_file, demands_file, q, n)
            except Exception as e:
                self.get_logger().error(f"Solver Error: {e}")
                continue
            
            # depot_coordinate =glob.glob(os.path.join(self.output_data_dir,f"/{demands_file}.csv"))
            # for i,q in enumerate(depot_coordinate):
            #     if q == 0:

            # 3. Pulizia vecchie traiettorie per sicurezza
            # Cancella i vecchi traj_vehicle_*.csv per non eseguire file di run precedenti
            old_traj_files = glob.glob(os.path.join(self.output_data_dir, "traj_vehicle_*.csv"))
            for f in old_traj_files:
                os.remove(f)

            # 4. Trova le rotte generate dal solver
            routes_vehicles = []
            if os.path.exists(self.output_data_dir):
                for f in os.listdir(self.output_data_dir):
                    if 'route_vehicle' in f and f.endswith('.csv'):
                        routes_vehicles.append(f)
                routes_vehicles.sort()
            
            if not routes_vehicles:
                self.get_logger().warn("Nessun percorso generato dal solver.")
                continue

            # Liste per l'esecuzione finale
            final_traj_files = []
            final_cf_ids = []

            # 5. Genera Traiettorie e prepara la lista Droni
            for route_file in routes_vehicles:
                # Esempio nome file: route_vehicle_0.csv
                # Estraiamo l'ID numerico: '0'
                try:
                    vehicle_id_str = route_file.replace('route_vehicle_', '').replace('.csv', '')
                    vehicle_id = int(vehicle_id_str)
                except ValueError:
                    self.get_logger().error(f"Nome file non valido: {route_file}")
                    continue

                # Definisci nomi file
                path_route = os.path.join(self.output_data_dir, route_file)
                traj_filename = f'traj_vehicle_{vehicle_id}.csv'
                path_traj = os.path.join(self.output_data_dir, traj_filename)
                
                print(f"Generazione traj per drone {vehicle_id}...")
                generate_trajectory(path_route, path_traj)

                # Aggiungi alle liste di esecuzione SOLO se generato ora
                final_traj_files.append(traj_filename)
                
                # Associa l'ID del crazyflie (es. 0 -> cf_0)
                cf_name = f"cf_{vehicle_id}"
                final_cf_ids.append(cf_name)

            # Ordina le liste per sicurezza (0, 1, 2...)
            # Zip, sort by filename, unzip
            if final_traj_files:
                zipped = sorted(zip(final_traj_files, final_cf_ids))
                final_traj_files, final_cf_ids = zip(*zipped)
                
                # Converti in liste modificabili
                final_traj_files = list(final_traj_files)
                final_cf_ids = list(final_cf_ids)

            # 6. Esecuzione Fisica/Simulata sui droni specifici
            self.execute_trajectory(final_traj_files, final_cf_ids)

    def execute_trajectory(self, trajectory_files, cf_ids):
        if not trajectory_files:
            return

        print(f'LANCIO ESECUZIONE SU: {cf_ids}')
        # print(f'FILES: {trajectory_files}')
        
        # Comando: python3 -m crazyflie_examples.cfs_waypoints traj1 traj2 cf0 cf1
        cmd = ["python3", "-m", "crazyflie_examples.cfs_waypoints"] + trajectory_files + cf_ids
        
        try:
            subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Errore esecuzione: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySelector()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()