import numpy as np
import re, sys
import argparse

# required libraries : argparse, numpy, matplotlib

parser = argparse.ArgumentParser()
parser.add_argument('-g', '--graph-only', action='store_true')
parser.add_argument('-f', '--file', default='alpaca')
parser.add_argument('-a', '--alternate', default=False, action='store_true')
args = parser.parse_args()

filename = args.file

numberPattern = r'\-?[0-9]+(\.[0-9]+)?(e?\-?[0-9]+)?'

# prefix = 'The new translation from addposetopose is:'
prefix = 'The current pose from addposetopose is:'

pose = re.compile(prefix+r'\s+'+numberPattern)
number = re.compile(numberPattern)
numberOnly = re.compile('^'+numberPattern+'$')
data = []
copyNum = 0
lineHistory = []
class pair:
    def __init__(self):
        self.start = -1
        self.end = -1
valsToTrack = {'mbImuInitialized':pair(), 'bInitialize':pair(), 'mbMapUpdated':pair()}
if not args.graph_only:
    with open(filename+'.txt', 'r') as f:
        moo = False
        for k,line in enumerate(f):
            k += 1
            for val in valsToTrack:
                if line.strip() == val+": 0":
                    valsToTrack[val].start = k
                    if valsToTrack[val].end != -1:
                        print(val,"1->0",valsToTrack[val].end,valsToTrack[val].start)
                        # valsToTrack[val].start = -1
                        valsToTrack[val].end = -1
                        moo = False
                elif line.strip() == val+": 1":
                    valsToTrack[val].end = k
                    if valsToTrack[val].start != -1:
                        print(val,"0->1",valsToTrack[val].start,valsToTrack[val].end)
                        valsToTrack[val].start = -1
                        # valsToTrack[val].end = -1
            if all(valsToTrack[val].start == -1 and valsToTrack[val].end != -1 for val in valsToTrack) and not moo:
                print("All 1")
                moo = True
            if re.match(r'Creation of new map with id: [0-9]+',line.strip()):
                print("new map",k)                
            if args.alternate:
                if copyNum > 0:
                    ret = number.search(line)
                    data[-1].append(float(str(ret.group(0)).strip()))
                    copyNum -= 1
                    if len(data) > 1 and len(data[-1]) == 3:
                        a = np.array(data[-2])
                        b = np.array(data[-1])
                        c = 0.3
                        if np.linalg.norm(a-b) > c:
                            if (abs(np.linalg.norm(a)-np.linalg.norm(b)) < c) and (abs(np.max(a) - np.max(b)) < c*2):
                                print(f"AXIS CHANGE!. HERE IS K: {k}")
                else:
                    if pose.match(line):
                        ret = number.search(line)
                        data.append([float(str(ret.group(0)).strip())])
                        copyNum = 2
            else:
                if re.match(r'\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-+', line):
                    toApp = []
                    try:
                        i = -1
                        while -i < len(lineHistory):
                            if lineHistory[i].strip() == "TRANSLATION":
                                i += 1
                                break
                            i -= 1
                        for j in range(i,i+3):
                            ret = numberOnly.match(lineHistory[j].strip())
                            val = float(str(ret.group(0)).strip())
                            toApp.append(val)
                        data.append(toApp)
                    except:
                        pass
                        # print('The line: ', end='')
                        # print(line.strip())
                        # print(lineHistory[-3:])
                        # raise


            lineHistory.append(line)

    with open(filename+'_clean.txt', 'w') as f:
        for d in data:
            f.write(str(d)+'\n')

    print(f'Converted {len(data)} data-points')
else:
    import ast
    with open(filename+'_clean.txt', 'r') as f:
        for line in f:
            data.append(ast.literal_eval(line.strip()))
    print(f'Found {len(data)} data-points')

# sys.exit(0)

def lpf(data, count):
    data_filtered = []
    for i in range(count-1): #have points stay same whose index is less than count
        data_filtered.append(np.array(data[i]))

    for i in range(count-1, len(data)):
        sum = [0,0,0]
        window = data[i-(count-1) : i]
        for point in window:
            for j in range(3):
                sum[j] += point[j]

        avg = [0,0,0]
        for j in range(3):
            avg[j] = sum[j] / count

        data_filtered.append(np.array(avg))
    return data_filtered


dataSplit = [[]]
data_filtered = []
LPF_WINDOW = 3

for d in data:
    if all(p == 0 for p in d):
        if len(dataSplit[-1]) != 0:
            dataSplit.append([])
    else:
        dataSplit[-1].append(d)

dataSplit = [np.array(data, dtype=np.float64) for data in dataSplit]

for data_map in dataSplit:
    data_map = list(data_map)
    if (len(data_map) < LPF_WINDOW): continue

    filtered_map = lpf(data_map, LPF_WINDOW)
    print("filtered: ", len(filtered_map))
    data_filtered.append(filtered_map)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure(figsize=(4,4))
ax = fig.add_subplot(111, projection='3d')  
ax.scatter(0,0,0, color='r', label='Origin') # plot the origin
plotted = 0

print(np.shape(dataSplit))
print(np.shape(data_filtered))

data_filtered = [np.array(data, dtype=np.float64) for data in data_filtered]

POINT_THRESH = 0.0005
for i,data_ in enumerate(dataSplit):
    if len(data_) < 20:
        continue
    # up is 2, right is 0, backward is 1
    stdX = np.std(data_[:,0])
    stdY = np.std(data_[:,1])
    stdZ = np.std(data_[:,2])
    if max(stdX, stdY, stdZ) >= POINT_THRESH:
        ax.plot(data_[:,0], data_[:,1], data_[:,2], label=f'Path {i}')
        ax.scatter(data_[-1,0],data_[-1,1],data_[-1,2], color='g', label=f'End Point {i}: {data_[-1,:]}') # plot the end point
        plotted += 1

# for map_count in range(len(data_filtered)):
#     data_map = data_filtered[map_count]
#     for i in range (2, len(data_map)): 
#         vec1 = data_map[i] - data_map[i-1]
#         vec1_norm = np.linalg.norm(vec1) # normalized vec1
#         if (vec1_norm != 0): 
#             vec1 = vec1 / vec1_norm
#         vec2 = data_map[i-2] - data_map[i-1]
#         vec2_norm = np.linalg.norm(vec2)
#         if (vec2_norm != 0): 
#             vec2 = vec2 / vec2_norm

#         if (vec1_norm < 0.1 or vec2_norm < 0.1): continue

#         if (vec1_norm != 0 and vec2_norm != 0 and np.abs(np.dot(vec1, vec2)) < 0.5):
#             print("potential flip: ", vec1, " , ", vec2)
#             print("between points: ", dataSplit[map_count][i-2], " , ", dataSplit[map_count][i-1], " , ", dataSplit[map_count][i]) # order: oldest to newest

#             # points go in order: yellow, fuchsia, cyan (order: oldest to newest)
#             for j in range(3):
#                 color = 'cyan' # j = 0
#                 if j == 1:
#                     color = 'fuchsia'
#                 elif j == 2:
#                     color="y"
                # ax.scatter(data_map[i-j][0], data_map[i-j][1], data_map[i-j][2], color=color)


ax.set_title(f"{plotted} Map"+('s' if plotted > 1 else ''))
# ax.legend(loc=(0,0))
ax.set_xlabel('$X$')
ax.set_ylabel('$Y$')
ax.set_zlabel('$Z$')

xScale = ax.get_xlim()[1]-ax.get_xlim()[0]
yScale = ax.get_ylim()[1]-ax.get_ylim()[0]
zScale = ax.get_zlim()[1]-ax.get_zlim()[0]
print(xScale, yScale, zScale)
if xScale > yScale:
    ax.set_ylim(list(np.array(ax.get_ylim())*(xScale/yScale)))
    ax.set_zlim(list(np.array(ax.get_zlim())*(xScale/zScale)))
else:
    ax.set_xlim(list(np.array(ax.get_xlim())*(yScale/xScale)))
    ax.set_zlim(list(np.array(ax.get_zlim())*(yScale/zScale)))

xScale = ax.get_xlim()[1]-ax.get_xlim()[0]
yScale = ax.get_ylim()[1]-ax.get_ylim()[0]
zScale = ax.get_zlim()[1]-ax.get_zlim()[0]
print(xScale, yScale, zScale)

# zRange = ax.get_zlim()[1]-ax.get_zlim()[0]
# minRange = 0.5
# if zRange < minRange:
#     m = np.mean(data[:,2])
#     ax.set_zlim([m-minRange/2, m+minRange/2])

plt.show()
