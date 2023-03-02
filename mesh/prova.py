import numpy as np 
import math
from scipy.spatial.transform import Rotation as R

vec1 = np.array([1.0, 1.0, 0.0])
vec2 = np.array([0.0, 1.0, 1.0])
a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
v = np.cross(a, b)
c = np.dot(a, b)
s = np.linalg.norm(v)
kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
print(rotation_matrix)
r11, r12, r13 = rotation_matrix[0]
r21, r22, r23 = rotation_matrix[1]
r31, r32, r33 = rotation_matrix[2]

theta1 = np.arctan(-r23 / r33)
theta2 = np.arctan(r13 * np.cos(theta1) / r33)
theta3 = np.arctan(-r12 / r11)
print(np.round(np.array([theta1,theta2,theta3]),5))