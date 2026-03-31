#!/usr/bin/env python3

import sys
import os
import glob
import rclpy
from rclpy.node import Node
from crazyflie_interfaces.srv import StartMission

# Import librerie custom
# Assicurati che questi import siano corretti rispetto alla struttura delle cartelle
from .vrp_solver import solve_vrp 
from .trajectory_client import generate_trajectory

class MissionClient(Node):
    def __init__(self):
        super().__init__('mission_client')
        self.cli = self.create_client(StartMission, 'start_mission')
        
        # Timeout aumentato per sicurezza all'avvio
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('In attesa del Server Flight...')

        # Cartella dove il solver salva i risultati
        self.output_data_dir = "/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data"

        # Configurazione Menu
        self.traj_options = {
            "1": ("Scenario 1 (Depot 0,0)", "coords.csv", "demands_case1.csv"),
            "2": ("Scenario 2 (Depot 2,0)", "coords.csv", "demands_case2.csv"),
            "3": ("Scenario 3 (Depot 1,-2)", "coords.csv", "demands_case3.csv"),
        }
        self.req = StartMission.Request()
        
        # NOTA: Ho rimosso la chiamata a show_menu_and_execute() da qui.
        # Lo chiameremo esplicitamente dal main.

    def send_request_and_wait(self, files, ids):
        """
        Invia la richiesta e blocca finché non riceve risposta.
        """
        self.req.trajectory_files = files
        self.req.drone_ids = ids 
        
        self.get_logger().info(f"Invio richiesta missione per droni: {ids}")
        
        future = self.cli.call_async(self.req)
        
        # QUI usiamo la tecnica che volevi: aspettare il risultato usando il nodo corrente
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()
    
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
                print("\nInsert Drone's Capacity (Q): ")
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
            self.get_logger().info(f"Eseguo: {description} con Q={q}, N={n}")

            # 2. Chiama il solver
            try:
                # Assicurati che solve_vrp gestisca i percorsi correttamente
                solve_vrp(coords_file, demands_file, q, n)
            except Exception as e:
                self.get_logger().error(f"Solver Error: {e}")
                continue
            
            # 3. Pulizia vecchie traiettorie
            old_traj_files = glob.glob(os.path.join(self.output_data_dir, "traj_vehicle_*.csv"))
            for f in old_traj_files:
                try:
                    os.remove(f)
                except OSError:
                    pass

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
                try:
                    vehicle_id_str = route_file.replace('route_vehicle_', '').replace('.csv', '')
                    vehicle_id = int(vehicle_id_str)
                except ValueError:
                    continue

                path_route = os.path.join(self.output_data_dir, route_file)
                traj_filename = f'traj_vehicle_{vehicle_id}.csv'
                path_traj = os.path.join(self.output_data_dir, traj_filename)
                
                # Generazione .csv per Crazyswarm
                generate_trajectory(path_route, path_traj)

                final_traj_files.append(traj_filename)
                cf_name = f"cf_{vehicle_id}"
                final_cf_ids.append(cf_name)

            # Ordina le liste
            if final_traj_files:
                zipped = sorted(zip(final_traj_files, final_cf_ids))
                final_traj_files, final_cf_ids = zip(*zipped)
                final_traj_files = list(final_traj_files)
                final_cf_ids = list(final_cf_ids)

            # 6. Esecuzione Fisica
            # Usiamo SELF, non creiamo un nuovo client!
            try:
                response = self.send_request_and_wait(final_traj_files, final_cf_ids)
                self.get_logger().info(f"Missione conclusa: {response.message}")
            except Exception as e:
                self.get_logger().error(f"Errore chiamata servizio: {e}")


def main(args=None):
    # 1. Inizializza ROS (UNA SOLA VOLTA)
    rclpy.init(args=args)
    
    # 2. Crea il nodo
    client_node = MissionClient()
    
    try:
        # 3. Lancia la logica del menu. 
        # Poiché dentro usiamo spin_until_future_complete, il flusso rimarrà qui
        # finché l'utente non preme '0' per uscire dal while loop.
        client_node.show_menu_and_execute()
    except KeyboardInterrupt:
        pass
    finally:
        # 4. Pulizia
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()