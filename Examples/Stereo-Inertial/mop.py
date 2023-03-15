import numpy as np
import re, sys
import argparse
import ast

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot(name):
    data = []
    with open(name + '.txt', 'r') as f:
        for line in f:
            if not line: continue
            if 'nan' in line: continue
            data.append(ast.literal_eval(line.strip()))
    print(f'Found {len(data)} data-points')

    data = [np.array(d, dtype=np.float64) for d in data]
    data = np.array(data)
    
    fig = plt.figure(figsize=(4,4))
    ax = fig.add_subplot(111, projection='3d')  
    plotted = 0
    
    # up is 2, right is 0, backward is 1

    # Calc and plots the absolute value of plotted points
    absfunc = np.vectorize(abs)
    # data[:,0] = absfunc(data[:,0])    
    # data[:,1] = absfunc(data[:,1])    
    # data[:,2] = absfunc(data[:,2])    

    # ax.plot(data[-100:,0], data[-100:,1], data[-100:,2], '--bo')
    ax.plot(data[:,0], data[:,1], data[:,2], '--bo')

    ax.set_title(f"{plotted} Map"+('s' if plotted > 1 else ''))
    ax.set_xlabel('$X$')
    ax.set_ylabel('$Y$')
    ax.set_zlabel('$Z$')

    xScale = ax.get_xlim()[1]-ax.get_xlim()[0]
    yScale = ax.get_ylim()[1]-ax.get_ylim()[0]
    zScale = ax.get_zlim()[1]-ax.get_zlim()[0]
    #print(xScale, yScale, zScale)
    
    if xScale > yScale:
        ax.set_ylim(list(np.array(ax.get_ylim())*(xScale/yScale)))
        ax.set_zlim(list(np.array(ax.get_zlim())*(xScale/zScale)))
    else:
        ax.set_xlim(list(np.array(ax.get_xlim())*(yScale/xScale)))
        ax.set_zlim(list(np.array(ax.get_zlim())*(yScale/zScale)))

    xScale = ax.get_xlim()[1]-ax.get_xlim()[0]
    yScale = ax.get_ylim()[1]-ax.get_ylim()[0]
    zScale = ax.get_zlim()[1]-ax.get_zlim()[0]
    #print(xScale, yScale, zScale)
    plt.show()



parser = argparse.ArgumentParser()
parser.add_argument('-v', '--vel', default=False, action='store_true')
parser.add_argument('-a', '--accel', default=False, action='store_true')
parser.add_argument('-gr', '--gravity_rot', default=False, action='store_true')
parser.add_argument('-gy', '--gyro', default=False, action='store_true')
args = parser.parse_args()

# Call func
if args.accel:
    plot('accel')
if args.vel:
    plot('velocity')    
if args.gravity_rot:
    plot('gravity_rot')
if args.gyro:
    plot('gyro')

