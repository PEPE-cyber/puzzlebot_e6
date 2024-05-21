#!/usr/bin/env python3
import rospy
from rplidar import RPLidar
from geometry_msgs.msg import Pose2D
from math import sin, cos, pi
import cv2 as cv
import numpy as np


from skimage.transform import probabilistic_hough_line
import matplotlib.pyplot as plt
from matplotlib import cm

from scipy.stats import linregress

import time

class Map:
    
    def __init__(self, width, height):
        assert width > 0 
        assert height > 0
        # make sure the width and height are odd numbers
        if width % 2 == 0:
            width += 1
        if height % 2 == 0:
            height += 1
        self.center = (width // 2, height // 2)
        self.width = width
        self.height = height
        self.goal = None
        self.currentPosition = None
        self.mapData = np.ones([height, width], dtype=np.uint8) * 255
    
    def setCoord(self, coord, value):
        [x, y] = coord
        self.mapData[self.center[1] + y][self.center[0] + x] = value
    
    def getCoord(self, coord):
        [x, y] = coord
        return self.mapData[self.center[1] + y][self.center[0] + x]
    
    def show(self):
        img = cv.cvtColor(np.array(self.mapData), cv.COLOR_GRAY2BGR)
        if self.goal:
            cv.circle(img, self.goal, 2, (0, 0, 255), -1)
        if self.currentPosition:
            cv.circle(img, self.currentPosition, 2, (0, 255, 0), -1)
        cv.imshow('map', img)

    def addObject(self, coord):
        [x, y] = coord
        self.mapData[self.center[1] + y][self.center[0] + x] = 0

    def setGoal(self, coord):
        [x, y] = coord
        self.goal = (self.center[0] + x, self.center[1] + y)

    def setCurrentPosition(self, coord):
        [x, y] = coord
        self.currentPosition = (self.center[0] + x, self.center[1] + y)

class PointCloud:
    
    def __init__(self, width, height):
        assert width > 0 
        assert height > 0
        # make sure the width and height are odd numbers
        if width % 2 == 0:
            width += 1
        if height % 2 == 0:
            height += 1
        self.center = (width // 2, height // 2)
        self.width = width
        self.height = height
        self.cloudData = np.ones([height, width], dtype=np.uint8) * 255
        self.lidarPoints = set()
        self.borderLines = []
        self.wallCornersAbolute = None
        self.wallCornersRelative = None
        self.prevWallCornersRelative = None
        self.lidar_bins = {}
        self.bin_width = 30
        self.lidar = RPLidar('/dev/ttyUSBLiDAR')

        
    
    def clean(self):
        self.cloudData = self.cloudData * 0 + 255
        self.lidarPoints.clear()
        self.borderLines.clear()
        self.borderCorners.clear()
        self.lidar_bins = {}

    
    
    def show(self):
        img = cv.cvtColor(np.array(self.cloudData), cv.COLOR_GRAY2BGR)
        for corner in self.borderCorners:
            cv.circle(img, (int(corner["coords"][0]), int(corner["coords"][1])), 8, (0, 0, 255), -1)

        cv.imshow('map', img)

    def showCompare(self, odom_x, odom_y, odom_theta, x, y, theta):
        img = cv.cvtColor(np.array(self.cloudData), cv.COLOR_GRAY2BGR)
        if self.wallCornersAbolute is not None:
            for corner in self.wallCornersAbolute:
                cv.circle(img, (int(corner[0]), int(corner[1])), 8, (0, 0, 255), -1)
        if self.wallCornersRelative is not None:
            for corner in self.wallCornersRelative:
                cv.circle(img, (int(corner[0]), int(corner[1])), 8, (0, 255, 0), -1)
        relativeY = lambda _y : int(self.center[1] + _y)
        relativeX = lambda _x : int(self.center[0] + _x)
        cv.circle(img, (relativeX(odom_x), relativeY(odom_y)), 7, (0, 255, 0), -1)
        cv.circle(img, (relativeX(x), relativeY(y)), 3, (255, 0, 0), -1)
        cv.imshow('map', img)

    def addPoint(self, coord, size=1):
        [x, y] = coord
        relativeY = self.center[1] + y
        relativeX = self.center[0] + x
        if size == 1:
            self.cloudData[relativeY][relativeX] = 0
        else:
            for i in range(-size // 2, size // 2 + 1):
                for j in range(-size // 2, size // 2 + 1):
                    # make sure the object is within the map and not lower than 0
                    if abs(relativeY + i) < self.height and abs(relativeX + j) < self.width and relativeY + i >= 0 and relativeX + j >= 0:
                        self.cloudData[relativeY + i][relativeX + j] = 0
        self.lidarPoints.add((relativeX, relativeY)) 
        bin_index = int(relativeX / self.bin_width)  # Assuming bin_width is pre-defined
        if bin_index not in self.lidar_bins:
            self.lidar_bins[bin_index] = []
        self.lidar_bins[bin_index].append((relativeX, relativeY))

    def addLidarMeasure(self, angle, distance):
        # make the center of the image as the origin
        x = int(distance * cos(angle * pi / 180))
        y = int(distance * sin(angle * pi / 180))
        if abs(x) < self.width and abs(y) < self.height // 2:
            self.addPoint([x, y], size=5)


    def findBorders(self):
        # find the lines in the map, use Hough transform
        start_time = time.time()
        
        image = self.cloudData.copy()
        # invert the image
        image = 255 - image
        # Line finding using the Probabilistic Hough Transform
        lines = probabilistic_hough_line(image, threshold=30, line_length=55,
                                        line_gap=10)
        # group points based on the lines
        
        threshold = 0
        linePoints = []
        for line in lines:
            p0, p1 = line
            linePoints.append([])
            min_x = min(p0[0], p1[0]) - threshold
            max_x = max(p0[0], p1[0]) + threshold
            min_y = min(p0[1], p1[1]) - threshold
            max_y = max(p0[1], p1[1]) + threshold
            # Determine relevant bins
            start_bin = int(min_x / self.bin_width)
            end_bin = int(max_x / self.bin_width)

            # Iterate over relevant bins
            for bin_index in range(start_bin, end_bin + 1):
                if bin_index in self.lidar_bins:
                    # Iterate over lidar points in the bin
                    for x, y in self.lidar_bins[bin_index]:
                        # Check if the point is inside the line
                        if min_x <= x <= max_x and min_y <= y <= max_y:
                            linePoints[-1].append((x, y))
            # for x, y in self.lidarPoints:
            #     # if the point is inside the line
            #     if x >= min_x and x <= max_x and y >= min_y and y <= max_y:
            #         linePoints[-1].append((x, y))


        # Group lines 
        unique_lines = []
        for i in range(0, len(lines)):
            p0, p1 = lines[i]
            # Found m, y-intercept, and x-intercept of the line and angle
            m = (p1[1] - p0[1]) / (p1[0] - p0[0]) if p1[0] - p0[0] != 0 else 9999
            x0 = p0[1] - m * p0[0]
            y0 = p0[0] - (p0[1] / m) if m != 0 else 9999
            angle = np.arctan2(p1[1] - p0[1], p1[0] - p0[0]) * 180 / np.pi
            # make sure the angle is positive
            if angle < 0:
                angle += 180
            found = False
            for j in range(0, len(unique_lines)):
                prev_line = unique_lines[j]
                prev_m = prev_line['m']
                prev_x0 = prev_line['x0']
                prev_y0 = prev_line['y0']
                prev_angle = prev_line['angle']
                # If the angle is close to the previous line
                if abs(angle - prev_angle ) < 10:
                    # if the m is greater than 1, compare the y-intercept
                    if abs(angle) > 45:
                       if abs(y0 - prev_y0) < 30 or abs((y0 - prev_y0) / ((y0 + prev_y0)/2)) < 0.20:
                            # Update the previous line
                            unique_lines[j]['m'] = (prev_line['m'] * unique_lines[j]['weight'] + m) / (unique_lines[j]['weight'] + 1)
                            unique_lines[j]['x0'] = (prev_line['x0'] * unique_lines[j]['weight'] + x0) / (unique_lines[j]['weight'] + 1)
                            unique_lines[j]['y0'] = (prev_line['y0'] * unique_lines[j]['weight'] + y0) / (unique_lines[j]['weight'] + 1)
                            unique_lines[j]['angle'] = (prev_line['angle'] * unique_lines[j]['weight'] + angle) / (unique_lines[j]['weight'] + 1)
                            unique_lines[j]['weight'] += 1
                            unique_lines[j]['points'] += linePoints[i]
                            found = True
                            break
                    else:
                         if abs(x0 - prev_x0) < 10 or abs((x0 - prev_x0) / ((x0 + prev_x0)/2)) < 0.20:
                            # Update the previous line
                            unique_lines[j]['m'] = (prev_line['m'] * unique_lines[j]['weight'] + m) / (unique_lines[j]['weight'] + 1)
                            unique_lines[j]['x0'] = (prev_line['x0'] * unique_lines[j]['weight'] + x0) / (unique_lines[j]['weight'] + 1)
                            unique_lines[j]['y0'] = (prev_line['y0'] * unique_lines[j]['weight'] + y0) / (unique_lines[j]['weight'] + 1)
                            unique_lines[j]['angle'] = (prev_line['angle'] * unique_lines[j]['weight'] + angle) / (unique_lines[j]['weight'] + 1)
                            unique_lines[j]['weight'] += 1
                            unique_lines[j]['points'] += linePoints[i]
                            found = True
                            break
            if not found:
                unique_lines.append({
                    'm': m,
                    'x0': x0,
                    'y0': y0,
                    'weight': 1,
                    'points': linePoints[i],
                    'angle': angle
                })

        borderLines = []
        # merge lines using linear regression
        show = False
        for line in unique_lines:
            xs = []
            ys = []
            for point in line['points']:
                xs.append(point[0])
                ys.append(point[1])

            # if the line is vertical, give it a m of 9999
            if len(set(xs)) == 1:
                m = 9999
                intercept = ys[0] - m * xs[0]
                show = True
            elif not len(line['points']) >  2:
                # skip the line if it has less than 3 points
                print("Skipping line")
                continue
            else: 
                m, intercept, r_value, p_value, std_err = linregress(xs, ys)
            borderLines.append({
                'm': m,
                'intercept': intercept,
                'points': line['points'],
            })
        if len(unique_lines) > 3:
            # for each line, find the r_value comparing to the other lines
            for line in borderLines:
                m = line['m']
                intercept = line['intercept']
                y_mean = sum([y for _,y in line['points']]) / (len(line['points'])) 
                for otherLine in borderLines:
                    if line == otherLine:
                        continue
                    # calculate the sse 
                    sse = 0
                    for point in otherLine['points']:
                        x, y = point
                        # calculate the distance between the point and the line
                        d = abs(m * x - y + intercept) / (m ** 2 + 1) ** 0.5
                        sse += d ** 2
                    # calculate the r_value
                    if sse / y_mean < 0.5:
                        # merge the two lines
                        m2 = otherLine['m']
                        intercept2 = otherLine['intercept']
                        x = (intercept - intercept2) / (m2 - m)
                        y = m * x + intercept
                        new_m = (m + m2) / 2
                        new_intercept = (intercept + intercept2) / 2
                        borderLines.remove(line)
                        borderLines.remove(otherLine)
                        borderLines.append({
                            'm': new_m,
                            'intercept': new_intercept,
                            'points': line['points'] + otherLine['points']
                        })
                        break
        # deleate the points in the lines
        for i in range(len(borderLines)):
            borderLines[i]['points'].clear()
            
        # print("Merged {} into {} lines".format(len(unique_lines),len(borderLines)), end=' ')
        

        intersections = []
        # find the intersections of the borderLines 
        for i in range(0, len(borderLines)):
            for j in range(i + 1, len(borderLines)):
                line1 = borderLines[i]
                line2 = borderLines[j]
                m1 = line1['m']
                intercept1 = line1['intercept']
                m2 = line2['m']
                intercept2 = line2['intercept']
                if m1 == m2:
                    continue
                x = (intercept2 - intercept1) / (m1 - m2)
                y = m1 * x + intercept1
                intersections.append({
                    'coords': (x, y),
                        'lines': [{
                            'm': m1,
                            'intercept': intercept1
                        }, {
                            'm': m2,
                            'intercept': intercept2,
                        }],
                    })
        # print("Found {} intersections".format(len(intersections)))
        # add intersections to the map
        self.borderCorners = intersections
        self.borderLines = borderLines
    
    
    def estimatePosition(self, prevPos, prev_theta):
        # estimate the current position of the robot based on border lines
        prev_x, prev_y = prevPos
        if len(self.borderCorners) > 2:
            #find the distance between each corner
            for i in range(0, len(self.borderCorners)):
                self.borderCorners[i]['distances'] = {}
                for j in range(0, len(self.borderCorners)):
                    if i == j:
                        continue
                    x1, y1 = self.borderCorners[i]['coords']
                    x2, y2 = self.borderCorners[j]['coords']
                    self.borderCorners[i]['distances'][j] = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
                self.borderCorners[i]['shortest_distance'] = sorted(self.borderCorners[i]['distances'].items(), key=lambda x: x[1])[0]
            
            # find the closest corners
            wall_corner = sorted(self.borderCorners, key=lambda x: x['shortest_distance'][1])[0]
            # asume closest corner is the wall
            wall_corner_coords = wall_corner['coords'],  self.borderCorners[wall_corner['shortest_distance'][0]]['coords']
            wall_line = {
                'm': (wall_corner_coords[1][1] - wall_corner_coords[0][1]) / (wall_corner_coords[1][0] - wall_corner_coords[0][0]),
                'intercept': wall_corner_coords[0][1] - (wall_corner_coords[1][1] - wall_corner_coords[0][1]) / (wall_corner_coords[1][0] - wall_corner_coords[0][0]) * wall_corner_coords[0][0]
            }
            
            # compare with the previous wall corners
            if self.prevWallCornersRelative is not None:
                change_distance = ((wall_corner_coords[0][0] - self.prevWallCornersRelative[0][0]) ** 2 + (wall_corner_coords[0][1] - self.prevWallCornersRelative[0][1]) ** 2) ** 0.5
                # print("Change distance: ", change_distance)
                if change_distance > 5:
                    # the wall is too far from the previous wall, ignore the current wall
                    print("Wall is too far from the previous wall")
                    # return prevPos, prev_theta
            self.prevWallCornersRelative = wall_corner_coords
            # find the angle of the wall
            wall_angle = np.arctan2(wall_corner_coords[1][1] - wall_corner_coords[0][1], wall_corner_coords[1][0] - wall_corner_coords[0][0]) * 180 / np.pi
            if wall_angle < 0:
                wall_angle += 360
            
            # if the line is in the right side of the robot, rotate the angle by 180
            if wall_corner_coords[0][0] > self.center[0]:
                wall_angle += 180

            # Rotate the points with respect the center (robot position)
            robot_angle = ((wall_angle - 90) % 180)
            correction_angle = -robot_angle
            wall_corner_coords = np.array(wall_corner_coords) - np.array(self.center)
            R = np.array([[cos(correction_angle * pi / 180), -sin(correction_angle * pi / 180)], [sin(correction_angle * pi / 180), cos(correction_angle * pi / 180)]])
            self.wallCornersAbolute = np.dot(R, wall_corner_coords.T).T + np.array(self.center)
            # the distance between the robot and the wall - 20 is the robots x position
            robot_x = -(self.wallCornersAbolute[0][0] + self.wallCornersAbolute[1][0]) / 2 + 180
            robot_y = (self.wallCornersAbolute[0][1] + self.wallCornersAbolute[1][1]) / 2 - self.center[1]
            # print("Robot angle: ", robot_angle)
            # print("Robot x: ", robot_x)
            # print("Robot y: ", robot_y)
            return (robot_x, robot_y), robot_angle

        elif len(self.borderCorners) == 1:

            perpendicular_lines = []
            print(self.borderLines)
            # find the  perpendicular lines
            for line in self.borderLines:
                angle = np.arctan(line['m']) * 180 / np.pi
                if angle < 0:
                    angle += 180
                for other_line in self.borderLines:
                    if line == other_line:
                        continue
                    other_angle = np.arctan(other_line['m']) * 180 / np.pi
                    if other_angle < 0:
                        other_angle += 180
                    perpendicular_angle = (angle + other_angle) / 2
                    if perpendicular_angle < 0:
                        perpendicular_angle += 180
                    print("Angle: ", angle, "Other angle: ", other_angle, "Perpendicular angle: ", perpendicular_angle)
                    if abs(angle - perpendicular_angle) < 10:
                        perpendicular_lines.append(line)
                
                print(perpendicular_lines)
                


        else:
            print("Not enough corners to estimate the position")
            return prevPos, prev_theta

    def run(self):
        for scan in self.lidar.iter_scans():
            for (_, angle, distance) in scan:
                self.addLidarMeasure(angle, distance)
            self.findBorders()
            self.estimatePosition((0, 0), 0)
            self.show()
            if cv.waitKey(1) & 0xFF == ord('q'):
                break


if __name__ == '__main__':
    node = PointCloud()
    try:
        node.run()
    except rospy.ROSInterruptException:
        node.lidar.disconnect()