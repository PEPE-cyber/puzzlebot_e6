from time import time
from rplidar import RPLidar
lidar = RPLidar('/dev/ttyUSB0')

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

def getScans(min_len):
    scanBuffer = {
        'angles': [],
        'ranges': [],
    }
    i = 0
    for scan in lidar.iter_scans():
        for _,angle,distance in scan:
            scanBuffer['angles'].append(angle)
            scanBuffer['ranges'].append(distance)
            i += 1
        if i > min_len:
            yield scanBuffer
            scanBuffer['angles'].clear()
            scanBuffer['ranges'].clear()
            i = 0
    
startTime = time()
for i, scan in enumerate(getScans(200)):
    print('%d: Got %d measurments' % (i, len(scan['ranges'])))
    print("Time:", time() - startTime)
    startTime = time()
    if i > 10:
        break

lidar.stop()
