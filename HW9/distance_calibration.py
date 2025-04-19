"""
Used for calibrating the distance of a block from the robot
"""

from utilities.vision import Camera
import cv2 as cv
import matplotlib.pyplot as plt

cam = Camera()
areas = []
distances = []

while True:
    key = str(input("Press Enter to collect data or Q to quit: "))
    key = key.lower()
    if key == 'q':
        break
    rgb_image = cam.capture_image()
    cv.imwrite("rgb_image.jpg", cv.cvtColor(rgb_image, cv.COLOR_RGB2BGR))
    block_image, blocks = cam.find_blocks(rgb_image)
    cv.imwrite("boxedImageEnd.jpg", cv.cvtColor(block_image, cv.COLOR_RGB2BGR))
    if blocks != None:
        area = blocks[0].bounding_area
        distance = float(input("Measured distance of block (inches): "))
        if distance == '':
            continue
        distance = distance/39.37
        areas.append(area)
        distances.append(distance)
    print()
    
# start plottin
print(f"Areas length: {len(areas)}")
print(f"Distances length: {len(distances)}")

plt.scatter(distances, areas)
plt.title("Distances vs Bounding Box Area")
plt.xlabel("Distance (meters)")
plt.savefig("distance_area_graph.jpg")