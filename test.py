#!/usr/bin/python3

import logging
from rplidar import RPLidar
from math import cos, sin, pi
from PIL import Image, ImageDraw 

MAX_S = 3000 # width and heigth of lidar scan map

# handler for console
sh = logging.StreamHandler()

# handler for file
fh = logging.FileHandler(filename="rplidar.log")

# create formatter
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
# add formatter only for file logging
fh.setFormatter(formatter)

LIDAR = RPLidar('/dev/ttyUSB0')

LIDAR.logger.setLevel(logging.DEBUG)
## suppress to print debug message in console
# LIDAR.logger.addHandler(sh)  
# write debug info to rplidar.log file
LIDAR.logger.addHandler(fh)


# getting info about lidar
INFO = LIDAR.get_info()
print("LIDAR Info: "+str(INFO))

HEALTH = LIDAR.get_health()
print("LIDAR Health status: " + str(HEALTH))

print("LIDAR Motor sontrol support: "+ str(LIDAR.get_MotorCtrlSupport()))
print("Legacy sample rate: "+str(LIDAR.get_SampleRate()))
cnt_modes = LIDAR.get_ScanModeCount()
print("Scan modes count = "+str(cnt_modes))
typ_mode = LIDAR.get_TypicalScanMode()

for i in range(cnt_modes):
    s = "mode #"+str(i)
    if i == typ_mode:
        s += "(* typical)"
    s += ", name:"+LIDAR.get_ScanModeName(i)
    s += ", answer type: "+format(LIDAR.get_ScanModeAnsType(i), '#02x')
    s += ", max distance: "+str(LIDAR.get_MaxDistance(i))
    s += ", sample duration: "+str(LIDAR.get_LidarSampleDuration(i))
    print(s)

# get maximum distanse in mm from typical scan mode
MAX_D = int(LIDAR.get_MaxDistance(typ_mode)*1000)


# prepare lidar map image
R1 = int(MAX_S/40)+1
R2 = int(MAX_S/200)+1

color = (255, 255, 255)
img = Image.new('RGB', (MAX_S+10, MAX_S+10), color)
draw = ImageDraw.Draw(img)

for d in range(R1):
    draw.ellipse([MAX_S/2+5-d*20, MAX_S/2+5-d*20, MAX_S/2+5+d*20, MAX_S/2+5+d*20], None, (223, 223, 223), 1)
    if MAX_D/(R1-1) < 100:
        draw.text((int(MAX_S/2+5), int(MAX_S/2+5-d*20)), "{:.2f}".format(d * MAX_D / (R1-1) / 1000), (0, 63, 127))
    else:
        draw.text((int(MAX_S/2+5), int(MAX_S/2+5-d*20)), "{:.1f}".format(d * MAX_D / (R1-1) / 1000), (0, 63, 127) )
for d in range(R2):
    draw.ellipse([MAX_S/2+5-d*100, MAX_S/2+5-d*100, MAX_S/2+5+d*100, MAX_S/2+5+d*100], None, (203, 203, 203), 4-2*(d%2))
    if MAX_D/(R2-1) < 100:
        draw.text( ( int(MAX_S/2+5+d*100), int(MAX_S/2+5)), "{:.2f}".format(d * MAX_D / (R2-1) / 1000), (0, 63, 127) )
    else:
        draw.text( ( int(MAX_S/2+5+d*100), int(MAX_S/2+5)), "{:.1f}".format(d * MAX_D / (R2-1) / 1000), (0, 63, 127) )
draw.line([5, 5, MAX_S+5, MAX_S+5], (223, 223, 223), 1 )
draw.line([MAX_S+5, 5, 5, MAX_S+5], (223, 223, 223), 1 )
draw.line([MAX_S/2+5, 5, MAX_S/2+5, MAX_S+5], (223, 223, 223), 1 )
draw.line([5, MAX_S/2+5, MAX_S+5, MAX_S/2+5], (223, 223, 223), 1 )


for i, scan in enumerate(LIDAR.iter_scans()):
    print('%d: Got %d measures' % (i, len(scan)))
    for (_, angle, distance) in scan:
        if distance > 0:                  # ignore initially ungathered data points
            radians = angle * pi / 180.0
            x = distance * cos(radians)
            y = distance * sin(radians)
            p_x = int(MAX_S/2 + 5 + x / MAX_D * MAX_S/2)
            p_y = int(MAX_S/2+5 + y / MAX_D * MAX_S/2)
            if p_x > 0 and p_y >0 and p_x < MAX_S+10 and p_y < MAX_S+10:
                # if int(angle) % 10 == 0:
                #     draw.text( (p_x, p_y), str(int(angle)), (255, 0, 127))
                img.putpixel( (p_x, p_y), (127, 0, 255))
            else:
                print(f'out of range - {angle}: {distance} -> {p_x}x{p_y}')
    if i >= 10:
        break
LIDAR.stop()
LIDAR.stop_motor()
LIDAR.disconnect()

img.save("test-lidar-map.png")


