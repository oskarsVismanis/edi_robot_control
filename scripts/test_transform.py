import numpy as np

fx = 2769.86
fy = 2769.17
cx = 943.19
cy = 585.2

u = 755.0 #550.0
v = 445.0 #790.0
depth = 1.3873167 #1.3681186

K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]])

pixel_coordinates = np.array([u, v, 1])

camera_coordinates = np.linalg.inv(K) @ (pixel_coordinates * depth)

print(f'point in reference to camera {camera_coordinates}')

# testPoint = (790.0, 550.0, 1.3681186)

# X = (testPoint[1]-cx) * testPoint[2]/fx
# Y = (testPoint[0]-cy) * testPoint[2]/fy
# computedPoint = (X, Y, testPoint[2])

# print(f"Camera point: {testPoint}")
# print(f"Computed point: {computedPoint}")