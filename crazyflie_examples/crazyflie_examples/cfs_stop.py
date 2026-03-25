# File: group_mask.py (o come preferisci chiamarlo)
from crazyflie_py import Crazyflie
import time

# Creiamo una funzione che accetta il nodo "padre" e il nome del drone
def esegui_missione(ros_node, nome_drone='cf_0'):
    
    # 1. Inizializzazione MANUALE di Crazyflie
    # Richiede: (nodo_ros, nome_drone, dizionario_parametri)
    # Passiamo None ai parametri se non servono configurazioni specifiche
    cf = Crazyflie(ros_node, nome_drone, None)

    # 2. Ora puoi usare i comandi del drone normalmente
    print(f"Invio comandi a {nome_drone}...")
    
    # Esempio del tuo codice originale
    cf.setGroupMask(0b00001001)
    
    # NOTA: Non usare timeHelper (che è di Crazyswarm). 
    # Usa time.sleep() o il rate del nodo ROS.
    
    cf.takeoff(targetHeight=0.5, duration=3.0, groupMask=1)
    time.sleep(3.0) 
    
    cf.land(targetHeight=0.02, duration=3.0)
    time.sleep(3.0)