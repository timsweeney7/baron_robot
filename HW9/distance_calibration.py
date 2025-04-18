"""
Used for calibrating the distance of a block from the robot
"""

from utilities.vision import camera

cam = camera
areas = []
distances = []

while True:
    key = str(input("Press Enter to collect data or Q to quit: "))
    key = key.lower()
    if key == 'q':
        break
    rgb_image = cam.capture_image()
    block_image, blocks = cam.find_blocks(rgb_image)
    area = blocks[0].bounding_area
    distance = float(input("Measured distance of block: "))
    
    areas.append()
    
# start plottin
    