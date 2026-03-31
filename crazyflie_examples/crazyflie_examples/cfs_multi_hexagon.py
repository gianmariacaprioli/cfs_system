#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from crazyflie_py import Crazyswarm
import numpy as np
import tf_transformations
from functools import partial
import math
import time
import os
from pathlib import Path

class SwarmPoseListener(Node):
    def __init__(self, all_cfs):
        super().__init__('swarm_pose_listener')
        self.drone_states = {} 
        self.has_data = {} 
        self.subs = [] 

        for cf in all_cfs:
            clean_prefix = cf.prefix.lstrip('/')
            topic_name = f'/{clean_prefix}/pose'
            
            self.drone_states[cf.prefix] = {'pos': np.array([0.0, 0.0, 0.0]), 'yaw': 0.0}
            self.has_data[cf.prefix] = False
            
            sub = self.create_subscription(PoseStamped, topic_name, partial(self.callback, prefix=cf.prefix), 10)
            self.subs.append(sub)

    def callback(self, msg, prefix):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = msg.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.drone_states[prefix]['pos'] = pos
        self.drone_states[prefix]['yaw'] = euler[2]
        self.has_data[prefix] = True 

    def get_state(self, prefix): return self.drone_states.get(prefix, None)
    def is_ready(self, prefix): return self.has_data.get(prefix, False)

def generate_hexagon_waypoints(radius=1.0, rotation_deg=0.0):
    waypoints = [np.array([0.0, 0.0, 0.0])] # Centro
    for i in range(6):
        rad = math.radians(i * 60 + rotation_deg)
        waypoints.append(np.array([radius * math.cos(rad), radius * math.sin(rad), 0.0]))
    rad = math.radians(rotation_deg)
    waypoints.append(np.array([radius * math.cos(rad), radius * math.sin(rad), 0.0])) # Chiudi figura
    waypoints.append(np.array([0.0, 0.0, 0.0])) # Centro
    return waypoints

def execute_mission(timeHelper, swarm, rate=30):
    listener_node = SwarmPoseListener(swarm.allcfs.crazyflies)

    for i, cf in enumerate(swarm.allcfs.crazyflies):
        try: cf.setGroupMask(1 << i)
        except: pass

    print("Attesa stabilità MOCAP (3 secondi)...")
    wait_start = time.time()
    while (time.time() - wait_start) < 3.0:
        rclpy.spin_once(listener_node, timeout_sec=0.1)

    initial_positions = {}; drone_waypoints = {}; drone_wp_index = {}; drone_finished = {}; drone_heights = {} 
    logs = {}; prev_errors = {}; integral_errors = {}
    
    HEX_RADIUS = 1.0; TOLERANCE = 0.15 

    for i, cf in enumerate(swarm.allcfs.crazyflies):
        prefix = cf.prefix
        drone_wp_index[prefix] = 0; drone_finished[prefix] = False
        logs[prefix] = {'odom': [], 'ref': [], 'err': [], 'ctrl': []}
        prev_errors[prefix] = np.zeros(3); integral_errors[prefix] = np.zeros(3)

        if listener_node.is_ready(prefix):
            initial_positions[prefix] = listener_node.get_state(prefix)['pos']
            print(f"{prefix}: Home Reale -> {initial_positions[prefix]}")
        else:
            initial_positions[prefix] = np.array(cf.initialPosition)
            print(f"WARN {prefix}: Uso Config Home -> {initial_positions[prefix]}")

        if i % 2 == 0:
            drone_waypoints[prefix] = generate_hexagon_waypoints(HEX_RADIUS, 0.0)
            drone_heights[prefix] = 0.5
        else:
            drone_waypoints[prefix] = generate_hexagon_waypoints(HEX_RADIUS, 30.0)
            drone_heights[prefix] = 1.0

    Kp = np.array([0.8, 0.8, 0.5]); Ki = np.array([0.05, 0.05, 0.1]); Kd = np.array([0.2, 0.2, 0.2])   
    MAX_INTEGRAL = 0.5; MAX_VEL = 0.1      

    start_time = timeHelper.time()
    
    # --- LOOP VOLO ---
    while not timeHelper.isShutdown():
        rclpy.spin_once(listener_node, timeout_sec=0)
        t_now = timeHelper.time() - start_time
        
        if all(drone_finished.values()): break

        for cf in swarm.allcfs.crazyflies:
            prefix = cf.prefix
            if drone_finished[prefix]:
                cf.cmdVelocityWorld(np.zeros(3), 0.0); continue

            state = listener_node.get_state(prefix)
            curr_pos = state['pos']; curr_yaw = state['yaw']
            
            wps = drone_waypoints[prefix]; idx = drone_wp_index[prefix]
            if idx >= len(wps): drone_finished[prefix] = True; continue

            target_pos = initial_positions[prefix] + wps[idx] + np.array([0.0, 0.0, drone_heights[prefix]])
            
            if np.linalg.norm(target_pos - curr_pos) < TOLERANCE:
                integral_errors[prefix] = np.zeros(3); drone_wp_index[prefix] += 1; continue

            dt = 1.0 / rate
            error = target_pos - curr_pos
            integral_errors[prefix] = np.clip(integral_errors[prefix] + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL)
            derivative = (error - prev_errors[prefix]) / dt
            prev_errors[prefix] = error
            
            cmd_vel = np.clip((Kp * error) + (Ki * integral_errors[prefix]) + (Kd * derivative), -MAX_VEL, MAX_VEL)
            
            yaw_error = 0.0 - curr_yaw
            while yaw_error > math.pi: yaw_error -= 2*math.pi
            while yaw_error < -math.pi: yaw_error += 2*math.pi
            cmd_yaw_rate = np.clip(0.3 * yaw_error, -0.5, 0.5)

            cf.cmdVelocityWorld(cmd_vel, cmd_yaw_rate)

            logs[prefix]['odom'].append([t_now, *curr_pos, curr_yaw]); logs[prefix]['ref'].append([t_now, *target_pos])
            logs[prefix]['err'].append([t_now, *error]); logs[prefix]['ctrl'].append([t_now, *cmd_vel, cmd_yaw_rate])

        timeHelper.sleepForRate(rate)

    # --- DISCESA DOLCE ---
    print("Discesa dolce verso Z=0.3m...")
    descent_start = timeHelper.time(); DESCENT_DUR = 4.0
    for p in integral_errors: integral_errors[p] = np.zeros(3)

    while not timeHelper.isShutdown() and (timeHelper.time() - descent_start) <= DESCENT_DUR:
        rclpy.spin_once(listener_node, timeout_sec=0)
        t_now = timeHelper.time() - start_time
        
        for cf in swarm.allcfs.crazyflies:
            prefix = cf.prefix
            state = listener_node.get_state(prefix)
            curr_pos = state['pos']; curr_yaw = state['yaw']
            
            target_land = initial_positions[prefix].copy()
            target_land[2] = 0.3  
            
            dt = 1.0 / rate; error = target_land - curr_pos
            integral_errors[prefix] = np.clip(integral_errors[prefix] + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL)
            derivative = (error - prev_errors[prefix]) / dt
            prev_errors[prefix] = error
            
            cmd_vel = np.clip((Kp * error) + (Ki * integral_errors[prefix]) + (Kd * derivative), -0.2, 0.2)
            cf.cmdVelocityWorld(cmd_vel, 0.0)
            
            logs[prefix]['odom'].append([t_now, *curr_pos, curr_yaw]); logs[prefix]['ref'].append([t_now, *target_land])
            logs[prefix]['err'].append([t_now, *error]); logs[prefix]['ctrl'].append([t_now, *cmd_vel, 0.0])
            
        timeHelper.sleepForRate(rate)

    # --- KILL SWITCH MOTORI ---
    print("Spegnimento Motori.")
    for cf in swarm.allcfs.crazyflies: cf.emergency()

    # --- SALVATAGGIO LOGS ---
    log_dir = Path.home() / 'cfs_ws/cfs_real/traj_logs'; os.makedirs(log_dir, exist_ok=True)
    for prefix, data in logs.items():
        clean_name = prefix.lstrip('/')
        if data['odom']: np.savetxt(log_dir / f'{clean_name}_odom.csv', data['odom'], delimiter=',', header='t,x,y,z,yaw', comments='')
        if data['ref']: np.savetxt(log_dir / f'{clean_name}_ref.csv', data['ref'], delimiter=',', header='t,x,y,z', comments='')
        if data['err']: np.savetxt(log_dir / f'{clean_name}_err.csv', data['err'], delimiter=',', header='t,ex,ey,ez', comments='')
        if data['ctrl']: np.savetxt(log_dir / f'{clean_name}_ctrl.csv', data['ctrl'], delimiter=',', header='t,vx,vy,vz,omega', comments='')
    listener_node.destroy_node()

def main():
    swarm = Crazyswarm(); execute_mission(swarm.timeHelper, swarm, rate=100)
if __name__ == '__main__': main()