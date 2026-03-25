#!/usr/bin/env python

from pathlib import Path

from crazyflie_py import Crazyswarm
from crazyflie_py.uav_trajectory import Trajectory
import numpy as np
import sys


def main(trajectory_name):

    swarm = Crazyswarm()
    allcfs = swarm.allcfs
    timeHelper = swarm.timeHelper
    cfs = swarm.allcfs.crazyflies
    cf1 = cfs[0]
    cf2 = cfs[1]

    cf1.setGroupMask(1)
    cf2.setGroupMask(2)

    # allcfs = swarm.allcfs
    # cfs1 = swarm.allcfs.crazyfliesByName['cf_1']
    print(f'{cf1}')

    traj1 = Trajectory()
    traj1.loadcsv(Path(__file__).parent / f'data/{trajectory_name}.csv')

    # enable logging
    allcfs.setParam('usd.logging', 1)

    TRIALS = 1
    TIMESCALE = 1.0
    # for i in range(TRIALS):
    #     for cf in allcfs.crazyflies:
    allcfs.uploadTrajectory(0, 0, traj1)

    # cfs.takeoff(targetHeight=1.0, duration=2.0, groupMask = 1)
    swarm.allcfs.takeoff(targetHeight=1.0, duration=2.0, groupMask=2)  # solo cf1 decolla
    timeHelper.sleep(2.5)
        # for cf in allcfs.crazyflies:
    pos = np.array(cfs.initialPosition) + np.array([0, 0, 1.0])
    print("il contenuto di pos è:" + f'{pos}')
    print(f'{cfs}')

    #cfs.goTo(pos, 0, 2.0, groupMask = 0)
    swarm.allcfs.goTo(pos, 0, 2.0, groupMask = 2)
    timeHelper.sleep(2.5)

    # cfs.startTrajectory(0, timescale=TIMESCALE, groupMask = 1)
    swarm.allcfs.startTrajectory(0, timescale=TIMESCALE, groupMask = 2)
    timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
    # allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
    # timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
    print(f'{cfs}')

    # cfs.land(targetHeight=0.06, duration=2.0, groupMask = 1)
    swarm.allcfs.land(targetHeight=0.06, duration=2.0, groupMask = 2)
    timeHelper.sleep(3.0)

    # disable logging
    cfs.setParam('usd.logging', 0)



if __name__ == '__main__':
    if len(sys.argv) > 1:
        trajectory_name = sys.argv[1]
    else:
        print("❌ Errore: specificare il nome della traiettoria (es: traj_square)")
        sys.exit(1)

    main(trajectory_name)