"""
Used for calibrating the distance of a block from the robot
"""

from utilities.vision import Camera
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import 

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


# Define the function to fit (e.g., a polynomial)
def func(x, a, b):
    return a * 1/(x) + b

# Sample data
x_data = np.array([1, 2, 3, 4, 5])
y_data = np.array([2.1, 3.9, 6.1, 8.2, 12.3])

# Fit the curve
popt, pcov = curve_fit(func, x_data, y_data)

# Extract the optimized parameters
a_opt, b_opt, c_opt = popt

# Generate points for the fitted curve
x_fit = np.linspace(min(x_data), max(x_data), 100)
y_fit = func(x_fit, a_opt, b_opt, c_opt)

# Plot the results
plt.plot(x_data, y_data, 'o', label='data')
plt.plot(x_fit, y_fit, '-', label='fit')
plt.legend()
plt.show()

print("Optimized parameters:", popt)
print("Covariance matrix:", pcov)





plt.savefig("distance_area_graph.jpg")