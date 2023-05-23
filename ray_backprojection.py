import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visualize_ray(origin, direction, length):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    t = np.linspace(0, length, 100)
    ray_points = origin[:, np.newaxis] + direction[:, np.newaxis] * t
    
    ax.plot(ray_points[0], ray_points[1], ray_points[2], 'b-')
    ax.scatter(origin[0], origin[1], origin[2], c='r', marker='o')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    plt.show()
    
    
origin = np.array([0, 0, 0])  # 시작점
direction = np.array([1, 1, 1])  # 방향벡터
length = 10  # 광선의 길이

visualize_ray(origin, direction, length)
