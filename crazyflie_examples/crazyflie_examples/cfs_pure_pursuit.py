#!/usr/bin/env python

from pathlib import Path
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
import os
import math
import tf_transformations
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# --- LISTENER POSIZIONE ---

class PoseListener(Node):
    def __init__(self, drone_prefix):
        clean_prefix = drone_prefix.lstrip('/')
        super().__init__(f'pose_listener_{clean_prefix}')
        
        topic_name = f'{drone_prefix}/pose'
        if not topic_name.startswith('/'):
            topic_name = '/' + topic_name   
            
        self.current_yaw = 0.0
        self.position = np.array([0.0,0.0,0.0])
        self.sub = self.create_subscription(PoseStamped, topic_name, self.callback, 10)

        
    def callback(self, msg):
        q = msg.pose.orientation

        self.position[0] = msg.pose.position._x
        self.position[1] = msg.pose.position._y
        self.position[2] = msg.pose.position._z
        print(f'POSITION VECTOR: x: {self.position[0]:.2f}  y:{self.position[0]:.2f}    z:{self.position[0]:.2f}')

        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = euler[2]

def executeTrajectory(timeHelper, cf, trajpath, rate=50, offset=np.zeros(3)):
    traj = Trajectory()
    traj.loadcsv(trajpath)
    pose_listener = PoseListener(cf.prefix)

    # --- PARAMETRI PID POSIZIONE ---
    Kp = np.array([0.8, 0.8, 0.5]) 
    Ki = np.array([0.0, 0.0, 0.0]) 
    Kd = np.array([0.4, 0.4, 0.3])

    # --- PARAMETRI YAW GUIDANCE ---
    K_OMEGA = 1.0          
    MAX_YAW_RATE = 2.0     

    FOCUS_X = 0.0
    FOCUS_Y = 0.0

    prev_error = np.zeros(3)
    integral_err = np.zeros(3)
    MAX_INTEGRAL_TERM = 0.5

    log_odom, log_ref, log_err, log_ctrl = [], [], [], []
    
    start_time = timeHelper.time()
    LOOKHAEADTIME = 0.2
    last_print_time = 0.0
    
    print(f"Avvio Traiettoria con Steering Law: sin(err)/r")

    while not timeHelper.isShutdown():
        rclpy.spin_once(pose_listener, timeout_sec=0)
        t = timeHelper.time() - start_time
        if t > traj.duration: break

        # 1. Traiettoria
        e_now = traj.eval(t)
        e_lookhaead = traj.eval(min(t+LOOKHAEADTIME,traj.duration))
        # target_pos = e_now.pos + np.array(cf.initialPosition) + offset
        target_pos = e_lookhaead.pos + np.array(cf.initialPosition) + offset
        # drone_pos = np.array(cf.position) 
        drone_pos = pose_listener.position

        print(f'POSIZIONE DRONE:    {drone_pos}')
        
        # 2. PID Posizione
        error = target_pos - drone_pos
        dt = 1.0 / rate
        
        integral_err += error * dt
        integral_err = np.clip(integral_err, -MAX_INTEGRAL_TERM, MAX_INTEGRAL_TERM)
        derivative = (error - prev_error) / dt
        prev_error = error
        
        v_pid = (Kp * error) + (Ki * integral_err) + (Kd * derivative)
        v_pid = np.clip(v_pid, -0.3, 0.3)
        cmd_vel = e_now.vel + v_pid

        # =========================================================
        #        3. STEERING LAW
        # =========================================================
        
        # diff_x = target_pos[0] - drone_pos[0]
        # diff_y = target_pos[1] - drone_pos[1]
        diff_x = FOCUS_X - drone_pos[0]
        diff_y = FOCUS_Y - drone_pos[1]
        r = math.sqrt(diff_x**2 + diff_y**2) 

        omega_rate_cmd = 0.0
        
        current_yaw = pose_listener.current_yaw

        if r > 0.1: 
            point_direction = math.atan2(diff_y, diff_x)
            
            # Calcolo errore con offset di PI
            yaw_error = point_direction - current_yaw + math.pi
            
            raw_omega = (math.sin(yaw_error) / r)*math.sqrt(cmd_vel[0]**2 + cmd_vel[1]**2)
            
            omega_rate_cmd = K_OMEGA * raw_omega
            omega_rate_cmd = np.clip(omega_rate_cmd, -MAX_YAW_RATE, MAX_YAW_RATE)

            if (t - last_print_time) > 0.5:
                deg_cur = math.degrees(current_yaw)
                deg_err = math.degrees(yaw_error)
                print(f"R:{r:.2f} Cur:{deg_cur:4.0f}° Cmd:{omega_rate_cmd:.2f}")
                last_print_time = t

        # Invio Comando
        cf.cmdVelocityWorld(cmd_vel, omega_rate_cmd)

        # 4. Logging 
        log_odom.append([t, drone_pos[0], drone_pos[1], drone_pos[2], current_yaw])
        log_ref.append([t, *e_now.pos, *e_now.vel])
        log_err.append([t, *error])
        log_ctrl.append([t, *cmd_vel, omega_rate_cmd])

        timeHelper.sleepForRate(rate)

    # --- SALVATAGGIO ---
    data_dir = Path('/root/cfs_ws/configs/traj_logs')
    print(f"Salvataggio logs in: {data_dir}...")
    os.makedirs(data_dir, exist_ok=True)
    
    np.savetxt(data_dir / 'log_odometry.csv', log_odom, delimiter=',', header='t,x,y,z,yaw', comments='')
    np.savetxt(data_dir / 'log_reference.csv', log_ref, delimiter=',', header='t,rx,ry,rz,rvx,rvy,rvz', comments='')
    np.savetxt(data_dir / 'log_errors.csv',   log_err,  delimiter=',', header='t,ex,ey,ez', comments='')
    np.savetxt(data_dir / 'log_control.csv',  log_ctrl, delimiter=',', header='t,vx,vy,vz,omega', comments='')
    
    pose_listener.destroy_node()

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    rate = 50.0 
    Z = 0.5

    cf.takeoff(targetHeight=Z, duration=2.5)
    timeHelper.sleep(3.0)
    cf.goTo([1.0, 0.0, 0.0], 0.0, duration=3.0, relative=True)
    timeHelper.sleep(3.0)

    input_traj_path = Path(__file__).parent / 'data/circle_trajectory_ros.csv'
    
    if not input_traj_path.exists():
        print(f"ERRORE: File non trovato: {input_traj_path}")
        return

    executeTrajectory(timeHelper, cf, input_traj_path, rate, offset=np.array([0.0, 0.0, 0.0]))

    cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=2.5)
    timeHelper.sleep(3.0)

if __name__ == '__main__':
    main()