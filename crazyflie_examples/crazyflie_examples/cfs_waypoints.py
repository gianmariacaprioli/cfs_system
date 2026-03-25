#!/usr/bin/env python

from pathlib import Path
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
import sys

DATA_PATH = Path('/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data')

# Default lists
trajectory_name_def = ['traj_vehicle_0.csv','traj_vehicle_1.csv','traj_vehicle_2.csv','traj_vehicle_3.csv']
cf_def = ["cf_0","cf_1","cf_2","cf_3"]

# Stati della macchina a stati
STATE_IDLE = 0
STATE_MOVING_TO_CENTER = 1
STATE_EXECUTING_TRAJ = 2
STATE_RETURNING_HOME = 3
STATE_DONE = 4

def main(trajectory_names=trajectory_name_def, cf_names=cf_def):
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # --- CONFIGURAZIONE ---
    TRIALS = 2           
    TIMESCALE = 1.0    
    Z_START_TRAJ = 1.0      # Altezza al centro (0,0,1.0)
    Z_HOME = 0.5            # Altezza riposo (Home)
    
    # --- TEMPISTICHE ---
    DURATION_GOTO_CENTER = 7.0  # Tempo per andare dalla Home al Centro
    BUFFER_TIME = 1.0           # Tempo di stop al centro prima di startare la traj
    DURATION_RETURN_HOME = 7.0  # Tempo per tornare dal Centro alla Home
    
    # [NUOVO] Ritardo di sicurezza aggiuntivo
    # Quanti secondi aspettare DOPO che il drone precedente ha iniziato la traiettoria
    # prima di far muovere il drone successivo.
    # Aumenta questo valore per distanziare di più i droni.
    DELAY_AFTER_TRAJ_START = 8.0 

    if len(trajectory_names) != len(cf_names):
        print("Errore: Lunghezza liste non corrispondente.")
        return

    # 1. Caricamento Dati
    flight_data = []
    
    print("--- 1. Caricamento Traiettorie ---")
    for cf_name, traj_name in zip(cf_names, trajectory_names):
        if cf_name not in allcfs.crazyfliesByName:
            print(f"{cf_name} ignorato.")
            continue
        
        cf = allcfs.crazyfliesByName[cf_name]
        file_path = DATA_PATH / traj_name
        
        if not file_path.exists():
            print(f" File mancante: {traj_name}")
            return

        traj = Trajectory()
        traj.loadcsv(file_path)
        cf.uploadTrajectory(0, 0, traj)
        
        flight_data.append({
            'cf': cf,
            'traj_duration': traj.duration,
            'name': traj_name,
            'home_pos': np.array(cf.initialPosition) + np.array([0, 0, Z_HOME]),
            'state': STATE_IDLE,
            'schedule': {} 
        })

    # ORDINAMENTO: Dal più LENTO al più VELOCE
    flight_data.sort(key=lambda x: x['traj_duration'], reverse=True)
    
    # --- 2. DECOLLO INIZIALE ---
    print(f"\n--- Decollo di gruppo a {Z_HOME}m ---")
    allcfs.takeoff(targetHeight=Z_HOME, duration=2.5)
    timeHelper.sleep(3.0)

    # --- 3. LOOP PROVE ---
    for trial in range(TRIALS):
        print(f"\n================ TRIAL {trial + 1} ================")
        
        # --- A. SCHEDULING ---
        current_clock_time = 0.0
        max_mission_time = 0.0

        for i, item in enumerate(flight_data):
            item['state'] = STATE_IDLE
            
            # 1. Start Viaggio (Dipende dal clock attuale)
            t_start_move = current_clock_time
            
            # 2. Start Traiettoria (dopo viaggio + buffer)
            t_start_traj = t_start_move + DURATION_GOTO_CENTER + BUFFER_TIME
            
            # 3. Start Ritorno (dopo traj)
            scaled_traj_time = item['traj_duration'] / TIMESCALE
            t_start_return = t_start_traj + scaled_traj_time
            
            # 4. Done
            t_done = t_start_return + DURATION_RETURN_HOME
            
            item['schedule'] = {
                'move': t_start_move,
                'traj': t_start_traj,
                'return': t_start_return,
                'done': t_done
            }
            
            if t_done > max_mission_time:
                max_mission_time = t_done
            
            # --- MODIFICA CHIAVE QUI SOTTO ---
            # Il prossimo drone partirà quando questo ha iniziato la traj + UN RITARDO DI SICUREZZA
            current_clock_time = t_start_traj + DELAY_AFTER_TRAJ_START
            
            print(f"[{item['cf'].prefix}] Move: {t_start_move:.1f}s -> TrajStart: {t_start_traj:.1f}s -> NextDroneStarts: {current_clock_time:.1f}s")

        # --- B. ESECUZIONE ---
        trial_timer = 0.0
        STEP = 0.1
        
        print("\n--- Start Sequenza ---")
        
        while trial_timer <= max_mission_time + 1.0:
            
            for item in flight_data:
                cf = item['cf']
                sched = item['schedule']
                state = item['state']
                
                # TRIGGER: Vai al Centro
                if state == STATE_IDLE and trial_timer >= sched['move']:
                    center_pos = np.array([0.0, 0.0, Z_START_TRAJ])
                    print(f"[{trial_timer:.1f}s] {cf.prefix} >> Va al centro {center_pos}")
                    cf.goTo(center_pos, 0.0, DURATION_GOTO_CENTER, relative=False, groupMask=0)
                    item['state'] = STATE_MOVING_TO_CENTER

                # TRIGGER: Inizia Traiettoria
                elif state == STATE_MOVING_TO_CENTER and trial_timer >= sched['traj']:
                    print(f"[{trial_timer:.1f}s] {cf.prefix} ** START Traiettoria")
                    cf.startTrajectory(0, timescale=TIMESCALE, relative=True, groupMask=0)
                    item['state'] = STATE_EXECUTING_TRAJ
                
                # TRIGGER: Torna a Casa
                elif state == STATE_EXECUTING_TRAJ and trial_timer >= sched['return']:
                    print(f"[{trial_timer:.1f}s] {cf.prefix} << Fine Traj. Torna a Home {item['home_pos']}")
                    cf.goTo(item['home_pos'], 0.0, DURATION_RETURN_HOME, relative=False, groupMask=0)
                    item['state'] = STATE_RETURNING_HOME

                # TRIGGER: Finito
                elif state == STATE_RETURNING_HOME and trial_timer >= sched['done']:
                    item['state'] = STATE_DONE

            timeHelper.sleep(STEP)
            trial_timer += STEP

        print("\nTutti i droni sono tornati alla base.")
        timeHelper.sleep(2.0)

    # --- ATTERRAGGIO ---
    print("\n--- Atterraggio Finale ---")
    allcfs.land(targetHeight=0.05, duration=2.5, groupMask=0)
    timeHelper.sleep(3.0)

if __name__ == '__main__':
    if len(sys.argv) > 1:
        args = sys.argv[1:]
        n = len(args) // 2
        main(args[:n], args[n:])
    else:
        main(trajectory_name_def, cf_def)