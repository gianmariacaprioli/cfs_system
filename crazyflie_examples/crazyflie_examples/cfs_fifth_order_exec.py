#!/usr/bin/env python

from pathlib import Path
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
import os

def executeTrajectory(timeHelper, cf, trajpath, rate=100, offset=np.zeros(3)):
    traj = Trajectory()
    traj.loadcsv(trajpath)

    # --- 1. SETUP PID ---
    # Tuning suggerito: Inizia basso e alza Kp se il tracking è lento
    Kp = np.array([0.3, 0.3, 0.005]) 
    Ki = np.array([0.0, 0.0, 0.0])
    Kd = np.array([0.1, 0.1, 0.005])

    prev_error = np.zeros(3)
    integral_err = np.zeros(3)

    # --- 2. SETUP LOGGING (Liste per accumulo dati) ---
    log_odom = []    # t, x, y, z
    log_ref = []     # t, rx, ry, rz, rvx, rvy, rvz, rax, ray, raz
    log_err = []     # t, ex, ey, ez
    log_ctrl = []    # t, cmd_vx, cmd_vy, cmd_vz (Velocità totale inviata)

    start_time = timeHelper.time()
    
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > traj.duration:
            break
        
        # --- A. ACQUISIZIONE DATI ---
        # Posizione Reale (Odometria)
        odom_pos = np.array(cf.position)
        
        # Riferimento (Eval del polinomio)
        e = traj.eval(t)
        
        # Target Reale (Polinomio + Offset iniziale)
        target_pos = e.pos + np.array(cf.initialPosition) + offset

        # --- B. CALCOLO ERRORE E PID ---
        pos_err = target_pos - odom_pos
        dt = 1.0/rate

        integral_err += pos_err * dt # Accumulo integrale
        derivative_err = (pos_err - prev_error) / dt

        prev_error = pos_err

        # Calcolo Correzione PID
        vel_correction = (Kp * pos_err) + (Ki * integral_err) + (Kd * derivative_err) 
        
        # Saturazione correzione (Max 1 m/s per sicurezza)
        vel_correction = np.clip(vel_correction, -0.5, 0.5)

        # --- C. COMANDO MOTORI (Feedforward + Feedback) ---
        # Sommiamo la velocità ideale del polinomio (e.vel) alla correzione PID
        # cmd_vel = e.vel + vel_correction
        cmd_vel = e.vel + vel_correction

        
        cf.cmdVelocityWorld(
            cmd_vel,    
            0.00,
        )

        # --- D. LOGGING ---
        # Salviamo tutto in liste (veloce) per scrivere su disco solo alla fine
        
        # 1. Odometria
        log_odom.append([t, odom_pos[0], odom_pos[1], odom_pos[2]])
        
        # 2. Riferimento (quello che dice la funzione eval)
        log_ref.append([t, *e.pos, *e.vel, *e.acc])
        
        # 3. Errori
        log_err.append([t, *pos_err])
        
        # 4. Controllo (Velocità finale inviata al drone)
        log_ctrl.append([t, *cmd_vel])

        timeHelper.sleepForRate(rate)

    # --- E. SALVATAGGIO DATI SU DISCO ---
    # Percorso richiesto
    data_dir = Path('/root/cfs_ws/configs/traj_logs')
    
    print(f"Salvataggio logs in: {data_dir} ...")
    os.makedirs(data_dir, exist_ok=True) # Crea la cartella se non esiste

    # Salvataggio CSV con header
    np.savetxt(data_dir / 'log_odometry.csv', log_odom, delimiter=',', header='t,x,y,z', comments='')
    np.savetxt(data_dir / 'log_reference.csv', log_ref, delimiter=',', header='t,rx,ry,rz,rvx,rvy,rvz,rax,ray,raz', comments='')
    np.savetxt(data_dir / 'log_errors.csv',   log_err,  delimiter=',', header='t,ex,ey,ez', comments='')
    np.savetxt(data_dir / 'log_control.csv',  log_ctrl, delimiter=',', header='t,cmd_vx,cmd_vy,cmd_vz', comments='')
    
    print("Salvataggio completato.")

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    rate = 30.0 # Frequenza controllo (Hz)
    Z = 0.5     # Altezza di decollo

    # Decollo
    cf.takeoff(targetHeight=Z, duration=2.0)
    timeHelper.sleep(2.5)

    # Esecuzione Traiettoria
    # Assicurati che questo file esista!
    input_traj_path = Path(__file__).parent / 'data/full_trajectory_ros.csv'
    
    executeTrajectory(timeHelper, cf, input_traj_path, rate, offset=np.array([0.0, 0.0, 0.0]))

    # Atterraggio
    cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=2.0)
    timeHelper.sleep(2.0)

if __name__ == '__main__':
    main()