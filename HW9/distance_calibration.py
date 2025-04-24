"""
Used for calibrating the distance of a block from the robot
"""

from utilities.vision import Camera
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np


cam = Camera()
heights = []
distances = []

count = 0

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
        height = blocks[0].bounding_height
        distance = float(input("Measured distance of block (inches): "))
        if distance == '':
            continue
        distance = distance/39.37
        heights.append(height)
        distances.append(distance)
    print()
    
# start plottin
print(f"Heights length: {len(heights)}")
print(f"Distances length: {len(distances)}")

plt.scatter(distances, heights)
plt.title("Distances vs Bounding Box Height")
plt.xlabel("Distance (meters)")


# Build design matrix: columns are 1/x and 1
distances = np.array(distances)
X = np.vstack([1/distances, np.ones_like(distances)]).T
y = heights

# Solve for [a, b] using least squares
coeffs = np.linalg.lstsq(X, y, rcond=None)[0]
a, b = coeffs

# Predict values
x_fit = np.linspace(min(distances), max(distances), 100)
y_fit = a * (1/x_fit) + b

# Plot the results
plt.plot(distances, heights, 'o', label='data')
plt.plot(x_fit, y_fit, '-', label='fit')
plt.text(x= max(x_fit)-max(x_fit)*0.85, y=max(y_fit)-max(y_fit)*0.85, s=f"y = {a}/x + {b}")
plt.legend()
plt.show()


plt.savefig("distance_area_graph.jpg")