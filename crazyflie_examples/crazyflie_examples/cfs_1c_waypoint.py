#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np


def main():
    swarm =Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cfs0 = swarm.allcfs.crazyfliesByName['cf_0']
    print(f'{cfs0}')

    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / 'data/traj_square.csv')

    # enable logging
    allcfs.setParam('usd.logging', 1)

    TRIALS = 1
    TIMESCALE = 1.0
    # for i in range(TRIALS):
    #     for cf in allcfs.crazyflies:
    cfs0.uploadTrajectory(0, 0, traj1)

    cfs0.takeoff(targetHeight=3.0, duration=1.5)
    timeHelper.sleep(2.5)
        # for cf in allcfs.crazyflies:
    pos = np.array(cfs0.initialPosition) + np.array([0, 0, 3.0])
    print("il contenuto di pos è:" + f'{pos}')
    print(f'{cfs0}')

    cfs0.goTo(pos, 0, 4.0)
    timeHelper.sleep(2.5)

    cfs0.startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
    # allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
    # timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
    print(f'{cfs0}')

    cfs0.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)

    # disable logging
    cfs0.setParam('usd.logging', 0)

if __name__ == '__main__':
    main()



#!/usr/bin/env python

# from pathlib import Path
# from crazyflie_py import Crazyswarm
# from crazyflie_py.uav_trajectory import Trajectory
# import sys

# DATA_PATH = Path('/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data')

# # Default per test
# trajectory_name_def = ['traj_vehicle_0.csv','traj_vehicle_1.csv','traj_vehicle_2.csv','traj_vehicle_3.csv']
# cf_def = ["cf_0","cf_1","cf_2","cf_3"]

# def main(trajectory_names=trajectory_name_def, cf_names=cf_def):
#     swarm = Crazyswarm()
#     timeHelper = swarm.timeHelper
#     allcfs = swarm.allcfs

#     # --- CONFIGURAZIONE ---
#     TIMESCALE = 1.0
#     TAKEOFF_HEIGHT = 1.5 # Richiesto: 1.5 mt
#     TRAJECTORY_ID = 0    # Usiamo lo stesso ID per tutti per avviarli insieme

#     if len(trajectory_names) != len(cf_names):
#         print("❌ Errore: Liste di lunghezza diversa")
#         return

#     # Variabile per tracciare la durata massima tra tutte le traiettorie
#     max_duration = 0.0

#     print("--- 1. Caricamento e Upload Traiettorie ---")
    
#     # 1. LOOP DI CARICAMENTO
#     # Carichiamo ogni CSV sul rispettivo drone, ma tutti nello SLOT 0
#     for cf_name, traj_name in zip(cf_names, trajectory_names):
#         if cf_name not in allcfs.crazyfliesByName:
#             print(f"⚠️ {cf_name} saltato (non presente)")
#             continue
            
#         cf = allcfs.crazyfliesByName[cf_name]
        
#         file_path = DATA_PATH / traj_name
#         if not file_path.exists():
#             print(f"❌ File {traj_name} mancante!")
#             return

#         traj = Trajectory()
#         traj.loadcsv(file_path)
        
#         # Carica la traiettoria sul drone nello slot ID 0
#         cf.uploadTrajectory(TRAJECTORY_ID, 0, traj)
        
#         print(f"✅ {cf_name} -> Pronta {traj_name} (Durata: {traj.duration:.2f}s)")
        
#         # Aggiorniamo la durata massima per sapere quanto aspettare dopo
#         if traj.duration > max_duration:
#             max_duration = traj.duration

#     # Abilita logging
#     allcfs.setParam('usd.logging', 1)

#     # --- 2. DECOLLO DI GRUPPO ---
#     print(f"\n--- 2. Decollo simultaneo a {TAKEOFF_HEIGHT}m ---")
    
#     # Questo comando viene inviato a TUTTI i droni contemporaneamente
#     allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=3.0)
#     timeHelper.sleep(3.5) # Attesa completamento decollo

#     # --- 3. ESECUZIONE SINCRONA ---
#     print("\n--- 3. Avvio traiettorie (Broadcast) ---")
#     print(f"Esecuzione in corso per {max_duration:.1f} secondi...")

#     # relative=True: La traiettoria parte dalla posizione corrente (1.5m)
#     # relative=False: Il drone cercherà di andare alle coordinate esatte del CSV (che spesso partono da 0)
#     # Di solito si usa relative=True per evitare scatti verso il basso.
#     allcfs.startTrajectory(TRAJECTORY_ID, timescale=TIMESCALE, relative=True)

#     # Attendiamo che la traiettoria PIÙ LUNGA finisca
#     timeHelper.sleep(max_duration / TIMESCALE + 2.0)

#     # --- 4. ATTERRAGGIO ---
#     print("\n--- 4. Atterraggio ---")
#     allcfs.land(targetHeight=0.06, duration=3.0)
#     timeHelper.sleep(3.5)

#     allcfs.setParam('usd.logging', 0)


# if __name__ == '__main__':
#     if len(sys.argv) > 1:
#         args = sys.argv[1:]
#         if len(args) % 2 != 0:
#              print("❌ Errore argomenti dispari.")
#              sys.exit(1)
#         n = len(args) // 2
#         main(args[:n], args[n:])
#     else:
#         main(trajectory_name_def, cf_def)


# #!/usr/bin/env python

# from pathlib import Path

# from crazyflie_py import Crazyswarm
# from crazyflie_py.uav_trajectory import Trajectory
# import numpy as np


# def main():
#     swarm =Crazyswarm()
#     timeHelper = swarm.timeHelper
#     allcfs = swarm.allcfs
#     cfs1 = swarm.allcfs.crazyfliesByName['cf_1']
#     print(f'{cfs1}')

#     traj1 = Trajectory()
#     traj1.loadcsv(Path(__file__).parent / 'data/traj_square.csv')

#     # enable logging
#     allcfs.setParam('usd.logging', 1)

#     TRIALS = 1
#     TIMESCALE = 1.0
#     # for i in range(TRIALS):
#     #     for cf in allcfs.crazyflies:
#     cfs1.uploadTrajectory(0, 0, traj1)

#     cfs1.takeoff(targetHeight=1.0, duration=2.0)
#     timeHelper.sleep(2.5)
#         # for cf in allcfs.crazyflies:
#     pos = np.array(cfs1.initialPosition) + np.array([0, 0, 1.0])
#     print("il contenuto di pos è:" + f'{pos}')
#     print(f'{cfs1}')

#     cfs1.goTo(pos, 0, 2.0)
#     timeHelper.sleep(2.5)

#     cfs1.startTrajectory(0, timescale=TIMESCALE)
#     timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
#     # allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
#     # timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
#     print(f'{cfs1}')

#     cfs1.land(targetHeight=0.06, duration=2.0)
#     timeHelper.sleep(3.0)

#     # disable logging
#     cfs1.setParam('usd.logging', 0)


# if __name__ == '__main__':
#     main()
