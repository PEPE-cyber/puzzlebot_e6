import time
import pandas as pd
import numpy as np
from math import cos, sin, pi, atan2, atan


def sign(x): return (1, -1)[x < 0]


class Lidar:

    def __init__(self):
        pass
times = []
maxTime = 0
# Replace 'filename.csv' with your file path
data = pd.read_csv('data2.csv', encoding='utf-8')
for i in range(400, len(data)):
    # GET POSE
    x = float(data['y'][i])
    y = float(data['x'][i])
    theta = float(data['theta'][i]) * 180 / pi

    angles = [(float(i) - theta) %
              360 for i in str(data['angles'][i]).strip().split(',') if i != '']
    ranges = [float(i) / 10 for i in str(data['distances']
                                         [i]).strip().split(',') if i != '']
    startTime = time.time()
    ranges = [range for _, range in sorted(zip(angles, ranges))]
    angles.sort()
    # display the corners in a polar plot

    # Find the local max of the ranges
    vertexLines = []
    for i in range(1, len(ranges) - 1):
        scan_x = ranges[i] * cos(angles[i] * pi / 180) + x
        scan_y = ranges[i] * sin(angles[i] * pi / 180) + y
        if ranges[i] > ranges[i - 1] and ranges[i] > ranges[i + 1]:
            vertexLines.append({
                'start': {
                    'x': scan_x,
                    'y': scan_y,
                },
                'scanAngle': angles[i],
                'used': False,
                'points': [],
            })

    # check the first and last vertex
    if ranges[-1] > ranges[-2] and ranges[-1] > ranges[0]:
        scan_x = ranges[-1] * cos(angles[-1] * pi / 180) + x
        scan_y = ranges[-1] * sin(angles[-1] * pi / 180) + y
        vertexLines.append({
            'start': {
                'x': scan_x,
                'y': scan_y,
            },
            'scanAngle': angles[-1],
            'used': False,
            'points': [],
        })

    vertexLines.sort(key=lambda x: x['scanAngle'])
    # add points to the vertex
    index = 1
    for angle, rng in zip(angles, ranges):
        scan_x = rng * cos(angle * pi / 180) + x
        scan_y = rng * sin(angle * pi / 180) + y
        if index < len(vertexLines):
            if not angle < vertexLines[index]['scanAngle']:
                index += 1
            if index < len(vertexLines):
                vertexLines[index - 1]['points'].append({
                    'x': scan_x,
                    'y': scan_y,
                    'range': rng,
                    'angle': angle,
                })
            else:
                vertexLines[-1]['points'].append({
                    'x': scan_x,
                    'y': scan_y,
                    'range': rng,
                    'angle': angle,
                })
        else:
            vertexLines[-1]['points'].append({
                'x': scan_x,
                'y': scan_y,
                'range': rng,
                'angle': angle,
            })

    for i, vertex in enumerate(vertexLines):
        if i == len(vertexLines) - 1:
            x2 = vertexLines[0]['start']['x']
            y2 = vertexLines[0]['start']['y']
        else:
            x2 = vertexLines[i + 1]['start']['x']
            y2 = vertexLines[i + 1]['start']['y']
        angle = atan2(y2 - vertex['start']['y'], x2 - vertex['start']['x'])
        if angle < 0:
            angle += pi
        vertexLines[i]['end'] = {
            'x': x2,
            'y': y2,
        }
        vertexLines[i]['angle'] = angle
        vertexLines[i]['x_mean'] = sum(
            [point['x'] for point in vertex['points']]) / len(vertex['points'])
        vertexLines[i]['y_mean'] = sum(
            [point['y'] for point in vertex['points']]) / len(vertex['points'])

    # Find the corners
    sameLineCorners = []
    threshold = pi * 0.15
    for i, vertex in enumerate(vertexLines):
        if vertex['used']:
            continue
        vertex['used'] = True
        lineCorners = [vertex]
        # find the angle of the line
        angle0 = vertex['angle']
        # compare the angle of the line with the angle of the next vertex
        j = (i + 1) % len(vertexLines)
        # print("line: ", len(sameLineCorners))
        while j != i:
            vertex2 = vertexLines[j]
            if not vertex2['used']:
                angle1 = vertex2['angle']
                if angle1 < 0:
                    angle1 += pi
                # print("Compered v2:", "start:", f"({vertex['start']['x']},{vertex['start']['y']})", "end:", f"({vertex['end']['x']},{vertex['end']['y']})", "angle:", angle1)
                # print("Angle0:", angle0)
                if abs(angle1 - angle0) < threshold:
                    # print('angle1', angle1)
                    # print('v1:', "start:", f"({vertex['start']['x']},{vertex['start']['y']})", "end:", f"({vertex['end']['x']},{vertex['end']['y']})", "angle:", vertex['angle'])
                    # print('v2:', "start:", f"({vertex2['start']['x']},{vertex2['start']['y']})", "end:", f"({vertex2['end']['x']},{vertex2['end']['y']})", "angle:", vertex2['angle'])
                    # print("- - - - - - - - - - - - - - - -")
                    # print(angle1 >  pi / 4)
                    # print(sign(vertex['start']['y']) == sign(vertex2['end']['y']))
                    # print(sign(vertex['start']['x']) == sign(vertex2['end']['x']))
                    if angle1 > pi / 4 and angle1 < 3 * pi / 4:
                        if abs(vertex['x_mean'] - vertex2['x_mean']) < 50:
                            lineCorners.append(vertex2)
                            vertexLines[j]['used'] = True

                        # else:
                        #     break
                    elif abs(vertex['y_mean'] - vertex2['y_mean']) < 50:
                        lineCorners.append(vertex2)
                        vertexLines[j]['used'] = True
                    # else:
                    #     break
            j += 1
            if j == len(vertexLines):
                j = 0
        sameLineCorners.append({
            'corners': lineCorners,
        })

    # find the lines using linear regression
    for line in sameLineCorners:
        points = []
        xs = np.array([])
        ys = np.array([])
        for vertex in line['corners']:
            rgnMean = sum([point['range']
                          for point in vertex['points']]) / len(vertex['points'])
            for point in vertex['points']:
                if not point['range'] < rgnMean * 0.8 and not point['range'] > rgnMean * 1.3:
                    points.append(point)
                    xs = np.append(xs, point['x'])
                    ys = np.append(ys, point['y'])

        line['points'] = points
        line['x_mean'] = xs.mean()
        line['y_mean'] = ys.mean()
        line['point_count'] = len(points)

        # use linear regression to find the line
        m, c = np.polyfit(xs, ys, 1) if len(xs) > 1 else (0, 0)
        line['m'] = m
        line['c'] = c

    # goup the lines that are parallel
    groups = 0
    for i, line in enumerate(sameLineCorners):
        if 'group' in line:
            continue
        if line['m'] == 0 and line['c'] == 0:
            del sameLineCorners[i]
            continue
        line['group'] = groups
        ang1 = atan(line['m'])
        group = []
        for j, line2 in enumerate(sameLineCorners):
            if i != j and not 'group' in line2:
                ang2 = atan(line2['m'])
                if abs(ang1 - ang2) < 0.1:
                    line2['group'] = groups
        groups += 1

    # print the groups
    parrallelLines = {}
    for i in range(groups):
        parrallelLines[i] = []
        for line in sameLineCorners:
            if 'group' in line and line['group'] == i:
                parrallelLines[i].append(line)

    borderLines = {
        'top': {
            'count': 0,
            'maxPoints': 0,
        },
        'bottom': {
            'count': 0,
            'maxPoints': 0,
        },
        'left': {
            'count': 0,
            'maxPoints': 0,
        },
        'right': {
            'count': 0,
            'maxPoints': 0,
        }
    }

    # find the groups with 2 or more lines
    for i in range(groups):
        if len(parrallelLines[i]) > 1:
            print("Group:", i)
            meanX = sum([line['x_mean']
                        for line in parrallelLines[i]]) / len(parrallelLines[i])
            meanY = sum([line['y_mean']
                        for line in parrallelLines[i]]) / len(parrallelLines[i])
    
            for line in parrallelLines[i]:
                if line['m'] > 1:
                    # vertical line, check if it is left or right
                    if sum([point['x'] for point in line['points']]) / len(line['points']) > meanX:
                        line['position'] = 'right'
                        borderLines['right']['count'] += 1
                        if  borderLines['right']['count'] == 1 or line['point_count'] >  borderLines['right']['maxPoints']:
                            borderLines['right']['maxPoints'] = line['point_count']
                            borderLines['right']['line'] = {
                                'm': line['m'],
                                'c': line['c'],
                            }
                    else:
                        line['position'] = 'left'
                        borderLines['left']['count'] += 1
                        if  borderLines['left']['count'] == 1 or line['point_count'] >  borderLines['left']['maxPoints']:
                            borderLines['left']['maxPoints'] = line['point_count']
                            borderLines['left']['line'] = {
                                'm': line['m'],
                                'c': line['c'],
                            }
                else:
                    # vertical line, check if it is the right or left
                    print("meanY:", meanY)
                    print(line['points'])
                    print("len:", len(line['points']))
                    if sum([point['y'] for point in line['points']]) / len(line['points']) > meanY:
                        line['position'] = 'top'
                        borderLines['top']['count'] += 1
                        if  borderLines['top']['count'] == 1 or line['point_count'] >  borderLines['top']['maxPoints']:
                            borderLines['top']['maxPoints'] = line['point_count']
                            borderLines['top']['line'] = {
                                'm': line['m'],
                                'c': line['c'],
                            }
                    else:
                        line['position'] = 'bottom'
                        borderLines['bottom']['count'] += 1
                        if  borderLines['bottom']['count'] == 1 or line['point_count'] >  borderLines['bottom']['maxPoints']:
                            borderLines['bottom']['maxPoints'] = line['point_count']
                            borderLines['bottom']['line'] = {
                                'm': line['m'],
                                'c': line['c'],
                            }
                print("m:", line['m'], "c:", line['c'],
                      "position:", line['position'])
                print("____________________")


    