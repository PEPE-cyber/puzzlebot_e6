#!/usr/bin/env python3
import time
from rplidar import RPLidar, RPLidarException

import cv2 as cv

import numpy as np
from math import cos, sin, pi, atan, atan2

import matplotlib.pyplot as plt

from scipy.stats import linregress

lidar = RPLidar('/dev/ttyUSBLiDAR', baudrate=115200)

import rospy
from puzzle.msg import Obstacles

i = 0
show = False 


def get_borders(angles, ranges, x, y):
    startTime = time.time()
    ranges = [range for _, range in sorted(zip(angles, ranges))]
    angles.sort()

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

    # deleate the vertex with less than 3 points
    vertexLines = [vertex for vertex in vertexLines if len(vertex['points']) > 3]

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
    threshold = pi * 0.1
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
        if len(vertex['points']) == 0:
            line['m'] = 0
            line['c'] = 0
            continue
        points = []
        xs = np.array([])
        ys = np.array([])
        for vertex in line['corners']:
            rgnMean = sum([point['range']
                          for point in vertex['points']]) / len(vertex['points'])
            for point in vertex['points']:
                if not point['range'] < rgnMean * 0.9 and not point['range'] > rgnMean * 1.2:
                    points.append(point)
                    xs = np.append(xs, point['x'])
                    ys = np.append(ys, point['y'])

        line['points'] = points
        line['x_mean'] = xs.mean()
        line['y_mean'] = ys.mean()
        line['point_count'] = len(points)

        # use linear regression to find the line
        m, c, _, _, _ = linregress(xs, ys)  if len(xs) > 1 else (0, 0, 0, 0, 0)
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
            # print("Group:", i)
            meanX = sum([line['x_mean']
                        for line in parrallelLines[i]]) / len(parrallelLines[i])
            meanY = sum([line['y_mean']
                        for line in parrallelLines[i]]) / len(parrallelLines[i])
    
            for line in parrallelLines[i]:
                if len(line['points']) < 3:
                    continue
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
                    # print("meanY:", meanY)
                    # print(line['points'])
                    # print("len:", len(line['points']))
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
                # print("m:", line['m'], "c:", line['c'],
                #       "position:", line['position'])
                # print("____________________")

    print("Time:", time.time() - startTime)
    print("____________________")
    if False:
        # display the corners in a polar plot
        colors = ['r', 'g', 'b', 'y', 'm', 'c', 'k',
                'w', 'r', 'g', 'b', 'y', 'm', 'c', 'k', 'w']
        fig = plt.figure()
        ax = fig.add_subplot(111)
        angles = [i * pi / 180 for i in angles]
        x = [range * cos(angle) for range, angle in zip(ranges, angles)]
        y = [range * sin(angle) for range, angle in zip(ranges, angles)]
        ax.plot(x, y, c='k', marker='o', markersize=5, linestyle='None')
        # for vertex in vertexLines:
        #     ax.plot(vertex['start']['x'], vertex['start']['y'], c='b', marker='x', markersize=10)
        # add border lines
        for key in borderLines:
            cl = colors.pop(0)
            if borderLines[key]['count'] > 0:
                m = borderLines[key]['line']['m']
                c = borderLines[key]['line']['c']
                x = np.array([-300, 300])
                y = x * m + c
                ax.plot(x, y, c=cl)

        colors = ['r', 'g', 'b', 'y', 'm', 'c', 'k',
                'w', 'r', 'g', 'b', 'y', 'm', 'c', 'k', 'w']
        for line in sameLineCorners:
            cl = colors.pop(0)
            # for point in line['points']:
            #     ax.plot(point['x'], point['y'], c=cl, marker='x', markersize=3)
            # for l in line['corners']:
            #     ax.plot([l['start']['x'], l['end']['x']], [l['start']['y'], l['end']['y']], c=cl, marker='x', markersize=15)
            m = line['m']
            x = np.array([-300, 300])
            y = x * m + line['c']
            ax.plot(x, y, c=cl, linestyle='--', linewidth=1)
        # adjust limits
        ax.set_xlim([-300, 300])
        ax.set_ylim([-300, 300])
        i += 1
        plt.show()
    # show evrything 
    # img = np.zeros((600, 600, 3), np.uint8)
    # for i in range(1, len(ranges) - 1):
    #     scan_x = int(ranges[i] * cos(angles[i] * pi / 180) + 300)
    #     scan_y = int(ranges[i] * sin(angles[i] * pi / 180) + 300)
    #     cv.circle(img, (scan_x, scan_y), 1, (255, 255, 255), -1)
    # # for vertex in vertexLines:
    # #     scan_x = int(vertex['start']['x'] + 300)
    # #     scan_y = int(vertex['start']['y'] + 300)
    # #     cv.circle(img, (scan_x, scan_y), 5, (255, 0, 0), -1)
    # for line in sameLineCorners:
    #     m = line['m']
    #     c = line['c']
    #     x = np.array([-300, 300])
    #     y = x * m + c
    #     x = x + 300
    #     y = y + 300
    #     cv.line(img, (int(x[0]), int(y[0])), (int(x[1]), int(y[1])), (0, 255, 0), 1)
    # for key in borderLines:
    #     if borderLines[key]['count'] > 0:
    #         m = borderLines[key]['line']['m']
    #         c = borderLines[key]['line']['c']
    #         x = np.array([-300, 300])
    #         y = x * m + c
    #         x = x + 300
    #         y = y + 300
    #         cv.line(img, (int(x[0]), int(y[0])), (int(x[1]), int(y[1])), (0, 0, 255), 1)
    # cv.imshow("Image", img)
    # cv.waitKey(1)
    return borderLines


def estimatePosition(borderLines):
    # estimate the current position of the robot based on border lines
    if borderLines['top']['count'] >= 1 and borderLines['bottom']['count'] >= 1 and borderLines['left']['count'] >= 1 and borderLines['right']['count'] >= 1:
        # find the intersection of the lines (corners)
        top_line = borderLines['top']['line']
        bottom_line = borderLines['bottom']['line']
        left_line = borderLines['left']['line']
        right_line = borderLines['right']['line']
        # find the intersection of the lines using m and c
        top_x = (bottom_line['c'] - top_line['c']) / (top_line['m'] - bottom_line['m'])
        top_y = top_line['m'] * top_x + top_line['c']
        bottom_x = (bottom_line['c'] - top_line['c']) / (top_line['m'] - bottom_line['m'])
        bottom_y = bottom_line['m'] * bottom_x + bottom_line['c']
        left_x = (right_line['c'] - left_line['c']) / (left_line['m'] - right_line['m'])
        left_y = left_line['m'] * left_x + left_line['c']
        right_x = (right_line['c'] - left_line['c']) / (left_line['m'] - right_line['m'])
        right_y = right_line['m'] * right_x + right_line['c']

        print("Top:", top_x, top_y)
        print("Bottom:", bottom_x, bottom_y)
        print("Left:", left_x, left_y)
        print("Right:", right_x, right_y)

        # find the angle of the walls
        wall_angle = np.arctan2(right_y - left_y, right_x - left_x) * 180 / np.pi
        if wall_angle < 0:
            wall_angle += 360

        # Rotate the points with respect the center (robot position)
        robot_angle = ((wall_angle - 90) % 180)
        correction_angle = -robot_angle
        leftCorners = np.array([[left_x, left_y], [right_x, right_y]])
        rigtCorner = np.array([[right_x, right_y]])
        R = np.array([[cos(correction_angle * pi / 180), -sin(correction_angle * pi / 180)], [sin(correction_angle * pi / 180), cos(correction_angle * pi / 180)]])
        leftCorners = np.dot(R, leftCorners.T).T 
        rigtCorner = np.dot(R, rigtCorner.T).T
        # the distance between the robot and the wall - 20 is the robots x position
        robot_x = (leftCorners[0][0] + leftCorners[1][0]) / 2
        robot_y = (leftCorners[0][1] + leftCorners[1][1]) / 2
        # print("Robot angle: ", robot_angle)
        # print("Robot x: ", robot_x)
        # print("Robot y: ", robot_y)
        return (robot_x, robot_y, robot_angle)
    return (0, 0, 0)   


info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

prev_left = False
prev_right = False
prev_front = False

def send_obstacles(left, right, front):
    global prev_left, prev_right, prev_front
    if left != prev_left or right != prev_right or front != prev_front:
        print("Sending obstacles")
        prev_left = left
        prev_right = right
        prev_front = front
        msg = Obstacles()
        msg.left = left
        msg.right = right
        msg.front = front
        pub.publish(msg)

def getScans(min_len):
    scanBuffer = {
        'angles': [],
        'ranges': [],
    }
    i = 0
    min_distance = 4000
    while True:
        try:
           for scan in lidar.iter_scans():
                front_close_points = 0
                left_close_points = 0
                right_close_points = 0
                for _,angle,distance in scan:
                    scanBuffer['angles'].append(angle - 180)
                    scanBuffer['ranges'].append(distance / 10)
                    if angle > 70 and angle < 110:
                        if distance < 350:
                            right_close_points += 1
                    elif angle > 350 or angle < 20:
                        if distance < 350:
                            front_close_points += 1
                    elif angle > 250 and angle < 290:
                        if distance < 350:
                            left_close_points += 1
                    i += 1
                if i > min_len:
                    yield scanBuffer
                    scanBuffer['angles'].clear()
                    scanBuffer['ranges'].clear()
                    i = 0
                print("Front:", front_close_points)
                print("Left:", left_close_points)
                print("Right:", right_close_points)
                send_obstacles(left_close_points > 3, right_close_points > 3, front_close_points > 3)  
        except RPLidarException:
            lidar.clean_input()  


if __name__ == '__main__':
    rospy.init_node('lidar')
    pub = rospy.Publisher('/obstacle', Obstacles, queue_size=10)
    for i, scan in enumerate(getScans(100)):
        try:
            # borderLines = get_borders(scan['angles'], scan['ranges'], 0, 0)
            # # print(borderLines)
            # x,y,theta = estimatePosition(borderLines)
            print("ok")
        except RPLidarException:
            lidar.stop()
        except KeyboardInterrupt:
            lidar.stop()
