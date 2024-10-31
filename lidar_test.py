import rplidar
from math import cos, sin, pi
import pygame
import pydoc
# Create an instance of the LidarWrapper
lidar_instance = rplidar.LidarWrapper("/dev/ttyUSB0", 1000000)

# Print module documentation
print(pydoc.render_doc(rplidar))

pygame.init()
lcd = pygame.display.set_mode((1000,1000))
pygame.mouse.set_visible(False)
lcd.fill((0,0,0))
pygame.display.update()
SCALE_FACTOR = 800/10
def process_data(data):
    global max_distance
    lcd.fill((0,0,0))
    for angle in range(360):
        distance = data[angle]
        if distance > 0:                  # ignore initially ungathered data points
            max_distance = 5000
            radians = (angle * pi / 180.0)  - pi/2  # -pi/2 to set the 0 degree up in the screen
            x = distance * cos(radians)
            y = distance * sin(radians)
            point =(int(x*SCALE_FACTOR)+500,int(y*SCALE_FACTOR)+500)
            lcd.set_at(point, pygame.Color(255, 255, 255))
    pygame.display.update()



while True:
    data = lidar_instance.get_scan_data()
    process_data(data)

