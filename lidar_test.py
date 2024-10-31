import rplidar
from math import cos, sin, pi
import pygame

# Create an instance of the LidarWrapper
lidar_instance = rplidar.LidarWrapper("/dev/ttyUSB0", 1000000)

# # Initialize the Lidar
# if lidar_instance.initialize():
#     print("Lidar initialized successfully!")

# # Start scanning
# lidar_instance.start_scan()

# Retrieve scan data
# while True:
#     print(len(lidar_instance.get_scan_data()))





pygame.init()
lcd = pygame.display.set_mode((800,800))
pygame.mouse.set_visible(False)
lcd.fill((0,0,0))
pygame.display.update()
SCALE_FACTOR = 800/5
def process_data(data):
    global max_distance
    lcd.fill((0,0,0))
    for angle in range(360):
        distance = data[angle]
        if distance > 0:                  # ignore initially ungathered data points
            max_distance = 5000
            radians = angle * pi / 180.0
            x = distance * cos(radians)
            y = distance * sin(radians)
            point =(int(x*SCALE_FACTOR)+400,int(y*SCALE_FACTOR)+400)
            lcd.set_at(point, pygame.Color(255, 255, 255))
    pygame.display.update()



while True:
    data = lidar_instance.get_scan_data()
    print(data[0])
    process_data(data)

