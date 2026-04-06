# File: group_mask.py (o come preferisci chiamarlo)
from crazyflie_py import Crazyflie
import time

def esegui_missione(ros_node, nome_drone='cf_0'):
    
    cf = Crazyflie(ros_node, nome_drone, None)

    print(f"Invio comandi a {nome_drone}...")
    
    cf.setGroupMask(0b00001001)
       
    cf.takeoff(targetHeight=0.5, duration=3.0, groupMask=1)
    time.sleep(3.0) 
    
    cf.land(targetHeight=0.02, duration=3.0)
    time.sleep(3.0)