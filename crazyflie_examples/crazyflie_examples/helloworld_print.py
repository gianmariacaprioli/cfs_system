#!/usr/bin/env python

from pathlib import Path
import rclpy

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
import sys

# trajectory_name = ['traj_vehicle_0.csv','traj_vehicle_1.csv','traj_vehicle_2.csv','traj_vehicle_3.csv']
# cf = ["cf_0","cf_1","cf_2","cf_3"]

# def main(trajectory_name=trajectory_name,cf=cf):

def main(trajectory_names,cf_names):

    TRIALS = 2
    TIMESCALE = 1.0
    Z = 1.0
    # trajectory_name = ['traj_vehicle_1.csv','traj_vehicle_2.csv','traj_vehicle_3.csv','traj_vehicle_4.csv']
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)

    for i,(cfs,idcf) in enumerate(allcfs.crazyfliesByName.items()):
        print('sono nel for' + f'   {i}        {cf_names}')
        # if str(i+1) in trajectory_name[i]:
        #     print(idcf)
        #     print(f'{i+1}     ' + f'{trajectory_name[i]}' )
    
    

    k=0
    for i in allcfs.crazyflies:
        data_path = Path('/root/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/data')
        file_path = data_path / f'{trajectory_names[k]}'
        traj = Trajectory()
        # traj1.loadcsv(Path(__file__).parent / f'data/{trajectory_name}.csv')
        traj.loadcsv(file_path)
        # pos = np.array(i.initialPosition) + np.array([0, 0, Z])
        i.uploadTrajectory(0, 0, traj)
        k =k+1

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)

    for j in range(TRIALS):
        for i in allcfs.crazyflies:
            pos = np.array([0,0,1.0])
            i.goTo(pos, 0, 2.0, relative=False, groupMask=1)
            timeHelper.sleep(5)
            i.startTrajectory(0, timescale=TIMESCALE)
            timeHelper.sleep(5)
    swarm.input.waitUntilButtonPressed()

    allcfs.land(targetHeight=0.02, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)


if __name__ == '__main__':
    if len(sys.argv) > 1:
        # sys.argv[1:] contiene tutti gli argomenti dopo il nome del modulo
        args = sys.argv[1:]

        # Prima metà = trajectories
        # Seconda metà = CF
        n = len(args) // 2
        trajectory_names = args[:n]
        cf_names = args[n:]

    else:
        print("❌ Errore: specificare il nome della traiettoria (es: traj_square)")
        sys.exit(1)

    main(trajectory_names,cf_names)