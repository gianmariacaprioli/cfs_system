#!/usr/bin/env python3

from crazyflie_interfaces.srv import StartMission
import rclpy
from rclpy.node import Node
from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
from pathlib import Path
import numpy as np
import os
import sys

# IMPORTAZIONE FUNZIONE ESTERNA
try:
    from crazyflie_examples.trajectory_client import generate_trajectory
except ImportError:
    sys.path.append("/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples")
    try:
        from trajectory_client import generate_trajectory
    except ImportError:
        print("ERRORE: generate_trajectory non trovata.")

# --- CONFIGURAZIONE ---
DATA_DIR_STR = "/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data"
DATA_PATH = Path(DATA_DIR_STR)

# Parametri
TIMESCALE = 1.0    
Z_START_TRAJ = 1.0      

# DEFINIZIONE HOME RELATIVA
X_HOME = 0.3
Y_HOME = 0.3
Z_HOME = 0.3

Z_WAITING = 1.0           
MIN_SAFE_DISTANCE = 0.45  
EVASION_Z_OFFSET = 0.3    # Offset verticale relativo (+/- 0.3m)

# Tempi
DURATION_GOTO_CENTER = 8.0
DURATION_GOTO_WAIT = 5.0  
DURATION_GOTO_RESUME_WP = 5.0 
DURATION_RETURN_HOME = 6.0
DELAY_AFTER_TRAJ_START = 7.0 
DURATION_GOTO_EVASION = 6.0   

# Stati
STATE_IDLE = 0
STATE_MOVING_TO_CENTER = 1
STATE_EXECUTING_TRAJ = 2
STATE_RETURNING_HOME = 3    
STATE_DONE = 4

# Stati di Emergenza
STATE_EVASION_HOLD = 90     
STATE_IN_QUEUE = 91         
STATE_RESUMING_POSITION = 92 # Fase di discesa alla Z nominale
STATE_EXECUTING_RESUMED = 93 
STATE_EMERGENCY_LANDING = 99

class StartSystem(Node):

    def __init__(self):
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs

        super().__init__('mission_server')
        
        self.srv = self.create_service(
            StartMission, 
            'start_mission', 
            self.mission_callback
        )
        
        if not os.path.exists(DATA_DIR_STR):
            self.get_logger().error(f"ERRORE GRAVE: Cartella dati non trovata in {DATA_DIR_STR}")
        else:
            self.get_logger().info(f">>> SERVER PRONTO. Lettura/Scrittura in: {DATA_DIR_STR}")

    def check_safety(self, flight_data):
        positions = [d['cf'].position for d in flight_data]
        num_drones = len(flight_data)
        
        for i in range(num_drones):
            if flight_data[i]['state'] in [STATE_DONE, STATE_EMERGENCY_LANDING]:
                continue
            for j in range(i + 1, num_drones):
                if flight_data[j]['state'] in [STATE_DONE, STATE_EMERGENCY_LANDING]:
                    continue

                # Se entrambi sono già in gestione emergenza, ignora
                if flight_data[i]['state'] >= 90 and flight_data[j]['state'] >= 90:
                    continue

                pos_i = np.array(positions[i])
                pos_j = np.array(positions[j])
                dist = np.linalg.norm(pos_i - pos_j)
                
                if dist < MIN_SAFE_DISTANCE:
                    self.get_logger().warn(f"!!! COLLISIONE {dist:.2f}m: {flight_data[i]['prefix']} <-> {flight_data[j]['prefix']}")
                    
                    z_i = pos_i[2]
                    z_j = pos_j[2]
                    
                    # Logica offset Z: chi è più alto sale, chi è più basso scende
                    if z_i > z_j:
                        offset_i, offset_j = +EVASION_Z_OFFSET, -EVASION_Z_OFFSET
                    else:
                        offset_i, offset_j = -EVASION_Z_OFFSET, +EVASION_Z_OFFSET

                    self.trigger_evasive_maneuver(flight_data[i], offset_i)
                    self.trigger_evasive_maneuver(flight_data[j], offset_j)

    def trigger_evasive_maneuver(self, drone_data, z_offset_relative):
        """
        1. Calcola Punto Medio (XY).
        2. Sposta drone a Punto Medio + Offset Z.
        3. Salva Punto Medio + Z Nominale come start per il ricalcolo.
        """
        cf = drone_data['cf']
        current_pos = np.array(cf.position) 
        
        traj_name = drone_data.get('traj_filename', "")
        route_filename = traj_name.replace("traj_", "route_")
        full_path = os.path.join(DATA_DIR_STR, route_filename)
        
        target_pos = current_pos 
        
        if os.path.exists(full_path):
            try:
                waypoints = np.loadtxt(full_path, delimiter=",", skiprows=1)
                if waypoints.ndim == 1: waypoints = np.array([waypoints])

                if len(waypoints) > 1:
                    wp_coords = waypoints[:, 0:2]
                    curr_xy = current_pos[0:2]

                    dists = np.linalg.norm(wp_coords - curr_xy, axis=1)
                    closest_idx = np.argmin(dists)
                    
                    # Segmento A -> B
                    if closest_idx == 0:
                        idx_a, idx_b = 0, 1
                    else:
                        idx_a, idx_b = closest_idx - 1, closest_idx
                    
                    p_a = wp_coords[idx_a]
                    p_b = wp_coords[idx_b]
                    
                    # --- PUNTO MEDIO ---
                    mid_x = (p_a[0] + p_b[0]) / 2.0
                    mid_y = (p_a[1] + p_b[1]) / 2.0
                    
                    # Z per l'evasione (con offset per evitare botto)
                    evasion_z = max(current_pos[2] + z_offset_relative, 0.1)
                    target_pos = np.array([mid_x, mid_y, evasion_z])
                    
                    # --- DATI PER RICALCOLO ---
                    # Il ricalcolo partirà dal punto medio, ma alla quota corretta (1.0m)
                    drone_data['resume_start_coords'] = np.array([mid_x, mid_y, Z_START_TRAJ])
                    drone_data['resume_wp_idx'] = idx_b
                    
                    direction = "SU" if z_offset_relative > 0 else "GIÙ"
                    self.get_logger().warn(f"[{drone_data['prefix']}] Evasione {direction} -> Midpoint Offset: {target_pos}")
            except Exception as e:
                self.get_logger().error(f"Errore calcolo midpoint: {e}")

        # Esegui GoTo Evasivo
        cf.goTo(target_pos, 0.0, DURATION_GOTO_EVASION, relative=False)
        drone_data['state'] = STATE_EVASION_HOLD

    def generate_resume_trajectory(self, drone_data):
        """
        Genera CSV:
        Riga 1: resume_start_coords (Midpoint @ Z=1.0)
        Righe 2..N: Waypoint rimanenti
        """
        try:
            resume_idx = drone_data.get('resume_wp_idx', 0)
            start_coords = drone_data.get('resume_start_coords', None)
            prefix = drone_data['prefix']
            
            traj_name = drone_data.get('traj_filename', "")
            route_filename = traj_name.replace("traj_", "route_")
            full_path = os.path.join(DATA_DIR_STR, route_filename)
            
            if not os.path.exists(full_path): return None
            if start_coords is None: return None

            all_waypoints = np.loadtxt(full_path, delimiter=",", skiprows=1)
            if all_waypoints.ndim == 1: all_waypoints = np.array([all_waypoints])
            
            # Se siamo oltre la fine, ritorna 0.0
            if resume_idx >= len(all_waypoints):
                return 0.0

            remaining_wps = all_waypoints[resume_idx:, :]
            
            temp_in = os.path.join(DATA_DIR_STR, f"resume_in_{prefix}.csv")
            temp_out = os.path.join(DATA_DIR_STR, f"resume_out_{prefix}.csv")
            
            self.get_logger().info(f"[{prefix}] Generazione Traj da Midpoint {start_coords} verso WP #{resume_idx}")

            with open(temp_in, 'w') as f:
                # 1. SCRIVI PUNTO DI PARTENZA (Midpoint @ Z Nominale)
                # NOTA: Niente Header "x,y,z" per compatibilità server C++
                f.write(f"{start_coords[0]:.4f},{start_coords[1]:.4f},{start_coords[2]:.4f}\n")
                
                # 2. SCRIVI PUNTI RIMANENTI
                for row in remaining_wps:
                    if len(row) >= 2:
                        x, y = float(row[0]), float(row[1])
                        z = float(Z_START_TRAJ)
                        f.write(f"{x:.4f},{y:.4f},{z:.4f}\n")
                f.flush()
                os.fsync(f.fileno())

            # Chiama Server
            generate_trajectory(temp_in, temp_out, v_max=0.6, a_max=0.4)
            
            if not os.path.exists(temp_out):
                self.get_logger().error(f"Errore: output {temp_out} mancante.")
                return None

            traj = Trajectory()
            traj.loadcsv(Path(temp_out))
            drone_data['cf'].uploadTrajectory(1, 0, traj)
            
            return traj.duration

        except Exception as e:
            self.get_logger().error(f"Errore generazione resume: {e}")
            return None

    def get_last_waypoint_from_route(self, traj_filename):
        route_filename = traj_filename.replace("traj_", "route_")
        full_path = os.path.join(DATA_DIR_STR, route_filename)
        fallback_wp = np.random.uniform(-0.5, 0.5, 2) 

        if not os.path.exists(full_path): return fallback_wp

        try:
            data = np.loadtxt(full_path, delimiter=",", skiprows=1)
            if data.ndim == 1: last_wp = data[0:2]
            else: last_wp = data[-1, 0:2]
            return last_wp
        except Exception: return fallback_wp

    def mission_callback(self, request, response):
        self.get_logger().info(f"START MISSIONE: Droni={request.drone_ids}")

        try:
            flight_data = []

            for cf_name, traj_name in zip(request.drone_ids, request.trajectory_files):
                if cf_name not in self.allcfs.crazyfliesByName: continue

                cf = self.allcfs.crazyfliesByName[cf_name]
                traj_path = DATA_PATH / traj_name
                
                if not traj_path.exists():
                     self.get_logger().error(f"File mancante: {traj_path}")
                     continue

                traj = Trajectory()
                traj.loadcsv(traj_path)
                cf.uploadTrajectory(0, 0, traj)

                home_pos = np.array(cf.initialPosition) + np.array([X_HOME, Y_HOME, Z_HOME])

                flight_data.append({        
                    'cf': cf,
                    'traj_filename': traj_name, 
                    'traj_duration': traj.duration,
                    'prefix': cf_name,
                    'home_pos': home_pos,
                    'state': STATE_IDLE,
                    'resume_wp_idx': 0,
                    'resume_start_coords': np.array([0.0, 0.0, Z_START_TRAJ]),
                    'timers': {}
                })

            if not flight_data: raise ValueError("Nessun drone valido.")

            flight_data.sort(key=lambda x: x['traj_duration'], reverse=True)

            current_stagger = 0.0
            for item in flight_data:
                t_move = current_stagger
                t_traj = t_move + DURATION_GOTO_CENTER + 2.0
                item['timers']['start_move'] = t_move
                item['timers']['start_traj'] = t_traj
                item['timers']['end_traj'] = t_traj + (item['traj_duration'] / TIMESCALE)
                current_stagger = t_traj + DELAY_AFTER_TRAJ_START

            print("--- Decollo ---")
            self.allcfs.takeoff(targetHeight=Z_HOME, duration=2.5)
            self.timeHelper.sleep(3.0)

            master_clock = 0.0
            STEP = 0.1 
            mission_running = True
            landing_queue = [] 

            print("--- Inizio Loop ---")
            
            while mission_running:
                self.check_safety(flight_data)
                
                # --- GESTIONE RIENTRO DA EVASIONE ---
                active_resumers = any(d['state'] in [STATE_RESUMING_POSITION, STATE_EXECUTING_RESUMED] for d in flight_data)

                if not active_resumers and len(landing_queue) > 0:
                    next_drone = landing_queue.pop(0)
                    
                    print(f"[{master_clock:.1f}] {next_drone['prefix']} >> Uscita Coda. Preparo Traiettoria...")
                    
                    duration = self.generate_resume_trajectory(next_drone)
                    
                    if duration and duration > 0.1:
                        # 1. Recupera il punto di partenza (Midpoint @ 1.0m)
                        target_wp = next_drone['resume_start_coords']
                        
                        print(f"[{master_clock:.1f}] {next_drone['prefix']} >> GoTo Punto di Ripresa (Reset Z): {target_wp}")
                        next_drone['cf'].goTo(target_wp, 0.0, DURATION_GOTO_RESUME_WP, relative=False)
                        
                        next_drone['resume_traj_duration'] = duration
                        next_drone['timers']['resume_wp_reached'] = master_clock + DURATION_GOTO_RESUME_WP
                        next_drone['state'] = STATE_RESUMING_POSITION
                    else:
                        print(f"[{master_clock:.1f}] {next_drone['prefix']} >> Missione finita. A casa.")
                        next_drone['cf'].goTo(next_drone['home_pos'], 0.0, DURATION_RETURN_HOME, relative=False)
                        next_drone['timers']['done_time'] = master_clock + DURATION_RETURN_HOME
                        next_drone['state'] = STATE_RETURNING_HOME

                all_done = True
                
                for item in flight_data:
                    st = item['state']
                    cf = item['cf']
                    timers = item['timers']

                    if st != STATE_DONE:
                        all_done = False

                    if st == STATE_EVASION_HOLD:
                        item['state'] = STATE_IN_QUEUE
                        landing_queue.append(item)
                        continue
                    
                    if st == STATE_IN_QUEUE:
                        continue

                    # --- FLUSSO NORMALE ---
                    if st == STATE_IDLE:
                        if master_clock >= timers['start_move']:
                            center_pos = np.array([0.0, 0.0, Z_START_TRAJ])
                            cf.goTo(center_pos, 0.0, DURATION_GOTO_CENTER, relative=False)
                            item['state'] = STATE_MOVING_TO_CENTER

                    elif st == STATE_MOVING_TO_CENTER:
                        if master_clock >= timers['start_traj']:
                            cf.startTrajectory(0, timescale=TIMESCALE, relative=True)
                            item['state'] = STATE_EXECUTING_TRAJ

                    elif st == STATE_EXECUTING_TRAJ:
                        if master_clock >= timers['end_traj']:
                            print(f"[{master_clock:.1f}] {item['prefix']} || Fine Traj. A Casa.")
                            cf.goTo(item['home_pos'], 0.0, DURATION_RETURN_HOME, relative=False)
                            item['timers']['done_time'] = master_clock + DURATION_RETURN_HOME
                            item['state'] = STATE_RETURNING_HOME

                    # --- FLUSSO RECUPERO ---
                    # 1. Il drone si abbassa dal punto di evasione al punto medio (Z=1.0)
                    elif st == STATE_RESUMING_POSITION:
                        if master_clock >= item['timers']['resume_wp_reached']:
                            dur = item.get('resume_traj_duration', 0.0)
                            print(f"[{master_clock:.1f}] {item['prefix']} >> START Traj Ricalcolata ({dur:.1f}s)")
                            cf.startTrajectory(1, timescale=1.0, relative=False)
                            item['timers']['resume_traj_end'] = master_clock + dur
                            item['state'] = STATE_EXECUTING_RESUMED

                    # 2. Esegue la nuova traiettoria
                    elif st == STATE_EXECUTING_RESUMED:
                        if master_clock >= item['timers']['resume_traj_end']:
                            print(f"[{master_clock:.1f}] {item['prefix']} || Recupero Finito. A casa.")
                            cf.goTo(item['home_pos'], 0.0, DURATION_RETURN_HOME, relative=False)
                            item['timers']['done_time'] = master_clock + DURATION_RETURN_HOME
                            item['state'] = STATE_RETURNING_HOME

                    # --- RIENTRO ---
                    elif st == STATE_RETURNING_HOME:
                        if master_clock >= item['timers']['done_time']:
                            print(f"[{master_clock:.1f}] {item['prefix']} -- Atterrato.")
                            item['state'] = STATE_DONE

                if all_done:
                    print("Tutti i droni hanno completato la missione.")
                    mission_running = False

                self.timeHelper.sleep(STEP)
                master_clock += STEP

            print("--- Atterraggio Finale ---")
            self.allcfs.land(targetHeight=0.04, duration=3.0)
            self.timeHelper.sleep(3.0)

            response.success = True
            response.message = "Missione completata."
            
        except Exception as e:
            self.get_logger().error(f"ERRORE CRITICO: {e}")
            self.allcfs.land(targetHeight=0.04, duration=3.0)
            response.success = False
            response.message = str(e)

        return response

def main(args=None):
    server_node = StartSystem()
    try:
        rclpy.spin(server_node)
    except KeyboardInterrupt:
        pass
    finally:
        pass

if __name__ == '__main__':
    main()