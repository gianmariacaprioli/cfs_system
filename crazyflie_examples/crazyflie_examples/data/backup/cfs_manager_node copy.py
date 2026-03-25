#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from .vrp_solver import solve_vrp
from .trajectory_client import generate_trajectory

from crazyflie_py import Crazyswarm
# from  crazyflie_examples.cfs_waypoints.py

import subprocess
import sys
import os

class TrajectorySelector(Node):

    def __init__(self):
        super().__init__('trajectory_selector')
        self.publisher_ = self.create_publisher(PoseStamped, '/cf_0/cmd_trajectory', 10)
        self.get_logger().info("Nodo TrajectorySelector avviato.")


        self.traj_options = {
            "1": ("15 Q4 v1", "1"),
            "2": ("11 Q4 v1", "2"),
            "3": ("15 Q4 v2", "3"),
        }
        self.pathdir = "/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples"
        self.show_menu_and_execute()

    def show_menu_and_execute(self):
        while rclpy.ok():
            print("\n=== Seleziona la traiettoria da eseguire ===")
            for k, (name, _) in self.traj_options.items():
                print(f"[{k}] {name}")
            choice = input("Inserisci il numero della traiettoria: ")
            print("[0] Esci")


            if choice == "0" or choice.lower() == "q":
                print("Uscita dal programma.")
                break

            if choice not in self.traj_options:
                print("Scelta non valida. Uscita.")
                continue


            traj_name, traj_func = self.traj_options[choice]
            self.get_logger().info(f"Esecuzione traiettoria: {traj_name}")
            solve_vrp(choice)

            routes_vechicles = []
            i=0
            # dirs=directories
            for (root, dirs, file) in os.walk(self.pathdir):
                for f in file:
                    if 'route_vehicle' in f:
                        i = i+1
                        routes_vechicles.append(f)

            routes_vechicles.sort()
            # print(f'{routes_vechicles}')
            # routes = [1,2,3,4]

            for j,k in enumerate(routes_vechicles):
                path1 = f'/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data/route_vehicle_{j}.csv'
                path2 = f'/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data/traj_vehicle_{j}.csv'
                print(j)
                # print(f'{path1}         {path2}')
                generate_trajectory(path1,path2)


            trajectory_vehicles = []
            i=0
            # dirs=directories
            for (root, dirs, file) in os.walk(self.pathdir):
                for f in file:
                    if 'traj_vehicle' in f:
                        i = i+1
                        trajectory_vehicles.append(f)
            trajectory_vehicles.sort()
            print(trajectory_vehicles)
            cf = ["cf_0","cf_1","cf_2","cf_3"]
            # for j,k in enumerate(routes_vechicles):
            self.execute_trajectory(trajectory_vehicles,cf)



            # traj_names = ["traj_vehicle_1.csv","traj_vehicle_2.csv","traj_vehicle_3.csv","traj_vehicle_4.csv"]
            # self.execute_trajectory(traj_names)

    def execute_trajectory(self, trajectory,cf):
        print(f'BEGINNING TRAJECTORY {trajectory}')
        subprocess.run(["python3", "-m", "crazyflie_examples.cfs_waypoints"] + trajectory + cf)

def main(args=None):

    rclpy.init(args=args)
    node = TrajectorySelector()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
