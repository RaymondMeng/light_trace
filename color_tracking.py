# Single Color RGB565 Blob Tracking Example
#
# This example shows off single color RGB565 tracking using the OpenMV Cam.

import sensor, image, time, math
from pyb import UART,Timer

obj_x = 0
obj_y = 0

uart = UART(3,115200)#初始化串口 波特率 115200
def transmit_data():
    bol   = (obj_x + obj_y) % 254
    pack_data = bytearray([0xff, obj_x>>8, obj_x&0x0ff, obj_y>>8, obj_y&0x0ff, bol, 0xfe])

    return pack_data

threshold_index = 0 # 0 for red, 1 for green, 2 for blue

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [((59, 0, 24, 118, 5, 50)), # generic_red_thresholds
              (30, 100, -64, -8, -32, 32), # generic_green_thresholds
              (0, 30, 0, 64, -128, 0)] # generic_blue_thresholds

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

while(True):
    clock.tick()
    img = sensor.snapshot().lens_corr(1.8) #畸变矫正

    if img:
        flag = 0
        for blob in img.find_blobs([thresholds[threshold_index]], pixels_threshold=100, area_threshold=100, merge=True):
        # These values depend on the blob not being circular - otherwise they will be shaky.
            #f blob.elongation() > 0.5:
            #img.draw_circle(blob.cx(), blob.cy(), c.r(), color = (0, 255, 0))#识别到的红色圆形用绿色的圆框出来
            img.draw_rectangle((blob.x(), blob.y(), blob.w(), blob.h()), color=(0,255,0))
            obj_x = blob.cx()
            obj_y = blob.cy()
            flag = 1
            uart.write(transmit_data())
            print(transmit_data())
        # These values are stable all the time.
        if flag == 0:
            package = bytearray([0xff, 0, 0, 0, 0, 0, 0xfe])
            uart.write(package)
            print(package)
    print(clock.fps())
